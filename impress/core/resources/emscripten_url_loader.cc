// Copyright 2024 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "core/resources/emscripten_url_loader.h"

#include <emscripten/emscripten.h>
#include <emscripten/fetch.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "zetasql/base/atomic_sequence_num.h"
#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/strings/cord.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "core/async/future.h"
#include "core/async/future_common.h"
#include "core/resources/http_utils.h"
#include "core/resources/url_loader.h"

namespace imp {
namespace resources {
namespace {

// Data that needs to live for the duration of a fetch. Anything emscripten
// needs to access right across network loads is held directly by the map inside
// EmscriptenUrlLoaderState.
struct FetchContext {
  // The future that will be resolved with the result of this fetch.
  WeakFuture<absl::Cord> future;
  // HTTP request body.
  std::string body;
  // The emscripten API requires an array of C-style strings for the headers.
  // We store the strings in RequestHeaderMap and then use header_pointers for
  // the API. Both structures need to stay valid for the duration of the fetch.
  UrlLoader::RequestHeaderMap headers;
  std::vector<const char*> header_pointers;

  emscripten_fetch_t* fetch = nullptr;
  // Set to true if the future is cancelled while emscripten_fetch() is still
  // executing (before its return value is stored in `fetch`).
  bool cancelled = false;
};

// Manages state and active fetches safely even if EmscriptenUrlLoader is
// destroyed before callbacks or futures complete.
struct EmscriptenUrlLoaderState {
  absl::Mutex fetch_mutex;
  absl::flat_hash_map<uint64_t, std::unique_ptr<FetchContext>> active_fetches
      ABSL_GUARDED_BY(fetch_mutex);
  base::SequenceNumber unique_fetch_id;

  void OnFetchComplete(uint64_t fetch_id, emscripten_fetch_t* fetch,
                       bool success) {
    std::unique_ptr<FetchContext> context;
    {
      absl::MutexLock lock(fetch_mutex);
      auto it = active_fetches.find(fetch_id);
      if (it == active_fetches.end()) {
        // The fetch request was already cleanly erased due to mid-flight
        // cancellation, CloseAll(), or prior termination.
        return;
      }
      context = std::move(it->second);
      active_fetches.erase(it);
    }

    std::optional<Future<absl::Cord>> future = context->future.Lock();
    if (future.has_value()) {
      if (success) {
        absl::Cord result;
        result.Append(absl::string_view(fetch->data, fetch->numBytes));
        future->Return(result);
      } else {
        future->Return(HttpCodeToStatus(fetch->status));
      }
    }

    emscripten_fetch_close(fetch);
  }

  void CloseAll() {
    std::vector<emscripten_fetch_t*> fetches_to_close;
    {
      absl::MutexLock lock(fetch_mutex);
      fetches_to_close.reserve(active_fetches.size());
      for (const auto& [id, context] : active_fetches) {
        if (context->fetch) {
          fetches_to_close.push_back(context->fetch);
        }
      }
      active_fetches.clear();
    }

    for (emscripten_fetch_t* fetch : fetches_to_close) {
      emscripten_fetch_close(fetch);
    }
  }
};

// Lightweight structure passed through Emscripten's void* userData.
struct FetchHandle {
  std::shared_ptr<EmscriptenUrlLoaderState> state;
  uint64_t fetch_id;
};

class EmscriptenUrlLoader : public UrlLoader {
 public:
  EmscriptenUrlLoader()
      : state_(std::make_shared<EmscriptenUrlLoaderState>()) {}
  ~EmscriptenUrlLoader() override;

  Future<absl::Cord> LoadUrl(const std::string& url) override;
  Future<absl::Cord> LoadUrl(const Request& request) override;

 private:
  std::shared_ptr<EmscriptenUrlLoaderState> state_;
};

EmscriptenUrlLoader::~EmscriptenUrlLoader() {
  if (state_) {
    state_->CloseAll();
  }
}

Future<absl::Cord> EmscriptenUrlLoader::LoadUrl(const std::string& url) {
  Request request{.url = url, .method = Method::kGet};
  return LoadUrl(request);
}

Future<absl::Cord> EmscriptenUrlLoader::LoadUrl(const Request& request) {
  Future<absl::Cord> future;

  auto context = std::make_unique<FetchContext>();
  context->future = future;
  context->body = request.body;

  uint64_t fetch_id = state_->unique_fetch_id.GetNext();

  emscripten_fetch_attr_t attr;
  emscripten_fetch_attr_init(&attr);

  absl::SNPrintF(attr.requestMethod, sizeof(attr.requestMethod), "%s",
                 request.method == Method::kPost ? "POST" : "GET");

  attr.attributes = EMSCRIPTEN_FETCH_LOAD_TO_MEMORY;

  if (!context->body.empty()) {
    attr.requestData = context->body.data();
    attr.requestDataSize = context->body.size();
  }

  // Add headers to the request if there are any.
  const UrlLoader::Config& config = GetConfig();
  if (!config.request_headers.empty() || !request.additional_headers.empty()) {
    for (const auto& [key, value] : config.request_headers) {
      context->headers[key] = value;
    }
    for (const auto& [key, value] : request.additional_headers) {
      context->headers[key] = value;
    }

    // Emscripten expects an array like
    // {key0, value0, key1, value1, ..., nullptr}
    context->header_pointers.reserve(context->headers.size() * 2 + 1);
    for (const auto& [key, value] : context->headers) {
      context->header_pointers.push_back(key.c_str());
      context->header_pointers.push_back(value.c_str());
    }
    context->header_pointers.push_back(nullptr);

    attr.requestHeaders = context->header_pointers.data();
  }

  attr.onsuccess = [](emscripten_fetch_t* fetch) {
    std::unique_ptr<FetchHandle> handle(
        reinterpret_cast<FetchHandle*>(fetch->userData));
    handle->state->OnFetchComplete(handle->fetch_id, fetch, /*success=*/true);
  };

  attr.onerror = [](emscripten_fetch_t* fetch) {
    std::unique_ptr<FetchHandle> handle(
        reinterpret_cast<FetchHandle*>(fetch->userData));
    handle->state->OnFetchComplete(handle->fetch_id, fetch, /*success=*/false);
  };

  attr.userData = new FetchHandle{state_, fetch_id};

  {
    absl::MutexLock lock(state_->fetch_mutex);
    state_->active_fetches[fetch_id] = std::move(context);
  }

  Future<absl::Cord> returned_future = future.Then(
      [state = state_, fetch_id](const absl::StatusOr<absl::Cord>& result) {
        if (result.status().code() != absl::StatusCode::kCancelled) {
          return result;
        }

        emscripten_fetch_t* fetch_to_close = nullptr;
        {
          absl::MutexLock lock(state->fetch_mutex);
          auto it = state->active_fetches.find(fetch_id);
          if (it != state->active_fetches.end()) {
            if (it->second->fetch != nullptr) {
              fetch_to_close = it->second->fetch;
              state->active_fetches.erase(it);
            } else {
              it->second->cancelled = true;
            }
          }
        }
        if (fetch_to_close != nullptr) {
          emscripten_fetch_close(fetch_to_close);
        }
        return result;
      },
      Executor::Type::kImmediate);

  emscripten_fetch_t* fetch = emscripten_fetch(&attr, request.url.c_str());

  {
    absl::MutexLock lock(state_->fetch_mutex);
    auto it = state_->active_fetches.find(fetch_id);
    if (it != state_->active_fetches.end()) {
      if (it->second->cancelled) {
        state_->active_fetches.erase(it);
        if (fetch != nullptr) {
          emscripten_fetch_close(fetch);
        }
      } else {
        it->second->fetch = fetch;
      }
    }
  }

  return returned_future;
}

}  // namespace

std::unique_ptr<UrlLoader> CreateEmscriptenUrlLoader() {
  return std::make_unique<EmscriptenUrlLoader>();
}

}  // namespace resources
}  // namespace imp
