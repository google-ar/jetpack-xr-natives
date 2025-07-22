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

#include <memory>
#include <optional>
#include <string>

#include "absl/status/status.h"
#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/async/future_common.h"
#include "core/async/future_group.h"
#include "core/resources/url_loader.h"

namespace imp {
namespace resources {
namespace {

class EmscriptenUrlLoader : public UrlLoader {
 public:
  Future<absl::Cord> LoadUrl(const std::string& url,
                             std::optional<FutureGroup> future_group) override;
};

// TODO: Implement a way to add headers to the request.
Future<absl::Cord> EmscriptenUrlLoader::LoadUrl(
    const std::string& url, std::optional<FutureGroup> future_group) {
  Future<absl::Cord> future;
  WeakFuture<absl::Cord>* weak_future = new WeakFuture<absl::Cord>(future);
  emscripten_async_wget_data(
      url.c_str(), weak_future,
      [](void* args, void* data, int size) {
        WeakFuture<absl::Cord>* weak_future =
            static_cast<WeakFuture<absl::Cord>*>(args);
        std::optional<Future<absl::Cord>> future = weak_future->Lock();
        if (future.has_value()) {
          absl::Cord result;
          result.Append(absl::string_view(static_cast<char*>(data), size));
          future->Return(result);
        }
        delete weak_future;
      },
      [](void* args) {
        WeakFuture<absl::Cord>* weak_future =
            static_cast<WeakFuture<absl::Cord>*>(args);
        std::optional<Future<absl::Cord>> future = weak_future->Lock();
        if (future.has_value()) {
          future->Return(
              absl::UnavailableError("Failed to create URLConnection."));
        }
        delete weak_future;
      });
  return future;
}

}  // namespace

std::unique_ptr<UrlLoader> CreateEmscriptenUrlLoader() {
  return std::make_unique<EmscriptenUrlLoader>();
}

}  // namespace resources
}  // namespace imp
