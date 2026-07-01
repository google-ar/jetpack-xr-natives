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

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "absl/strings/cord.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/async/future_common.h"
#include "core/resources/http_utils.h"
#include "core/resources/url_loader.h"

namespace imp {
namespace resources {
namespace {

// Data that needs to live for the duration of a fetch. Anything emscripten
// needs to access must be stored here.
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
};

class EmscriptenUrlLoader : public UrlLoader {
 public:
  Future<absl::Cord> LoadUrl(const std::string& url) override;
  Future<absl::Cord> LoadUrl(const Request& request) override;
};

Future<absl::Cord> EmscriptenUrlLoader::LoadUrl(const std::string& url) {
  Request request{.url = url, .method = Method::kGet};
  return LoadUrl(request);
}

Future<absl::Cord> EmscriptenUrlLoader::LoadUrl(const Request& request) {
  Future<absl::Cord> future;

  auto context = std::make_unique<FetchContext>();
  context->future = future;
  context->body = request.body;

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

    // Emscripten expects a array like
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
    std::unique_ptr<FetchContext> context(
        reinterpret_cast<FetchContext*>(fetch->userData));
    std::optional<Future<absl::Cord>> future = context->future.Lock();
    if (future.has_value()) {
      absl::Cord result;
      result.Append(absl::string_view(fetch->data, fetch->numBytes));
      future->Return(result);
    }
    emscripten_fetch_close(fetch);
  };

  attr.onerror = [](emscripten_fetch_t* fetch) {
    std::unique_ptr<FetchContext> context(
        reinterpret_cast<FetchContext*>(fetch->userData));
    std::optional<Future<absl::Cord>> future = context->future.Lock();
    if (future.has_value()) {
      future->Return(HttpCodeToStatus(fetch->status));
    }
    emscripten_fetch_close(fetch);
  };

  // Transfer ownership of context to emscripten.
  // We handle the deletion in the callbacks.
  attr.userData = reinterpret_cast<void*>(context.release());

  emscripten_fetch(&attr, request.url.c_str());
  return future;
}

}  // namespace

std::unique_ptr<UrlLoader> CreateEmscriptenUrlLoader() {
  return std::make_unique<EmscriptenUrlLoader>();
}

}  // namespace resources
}  // namespace imp
