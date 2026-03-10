/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_CORE_RESOURCES_URL_LOADER_H_
#define THIRD_PARTY_IMPRESS_CORE_RESOURCES_URL_LOADER_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include "absl/base/attributes.h"
#include "absl/container/flat_hash_map.h"
#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/async/future_interrupter.h"
#include "core/resources/download_progress_info.h"

namespace imp {
namespace resources {

// Schema prefix to indicate a local file. Supports iOS, Android, and desktop.
constexpr absl::string_view kLocalFileScheme = "file://";

// Represent cases that content length is unknown.
constexpr size_t kUnknownContentLength = SIZE_MAX;

// Base class for asynchronously loading for remote URLs.
// This is used by the AssetManager and ResourceManager to download assets.
//
// By default, there is a separate implementation of this for android, ios,
// wasm, and desktop (curl) that is automatically instantiated by Impress on
// startup. Impress users can also write their own UrlLoader and assign it by
// calling AssetManager::SetUrlLoader.
class UrlLoader {
 public:
  using RequestHeaderMap = absl::flat_hash_map<std::string, std::string>;
  enum class Method { kGet, kPost };

  // Global configuration options for the UrlLoader.
  struct Config {
    RequestHeaderMap request_headers;
  };

  struct Request {
    // The URL to load.
    std::string url ABSL_REQUIRE_EXPLICIT_INIT;
    // The HTTP method to use for the request.
    Method method = Method::kGet;
    // The body to be sent with the request. Can be empty for HTTP methods that
    // don't need a body (e.g. GET).
    std::string body;
    // Additional headers to be sent with the request. These are merged with the
    // global request headers in the config.
    RequestHeaderMap additional_headers;
  };

  UrlLoader()
      : download_progress_info_(std::make_shared<DownloadProgressInfo>()) {}
  virtual ~UrlLoader() = default;

  // TODO Use absl::string_view instead of std::string.
  virtual Future<absl::Cord> LoadUrl(const std::string& url) = 0;

  // Loads a URL using the specified method and post body.
  //
  // NOTE: This is an experimental API and is not implemented on all
  // platforms.
  virtual Future<absl::Cord> LoadUrl(const Request& request);

  // Obtains the current cumulative progress of all downloads. Returns 0 if
  // there are no downloads at all.  download_baseline establishes 0%.
  float GetDownloadProgress(size_t download_baseline);

  // Returns the loading progress of the indicated url as a fraction of its
  // entire requested size. Returns 0 if the url isn't found or the size of the
  // entire url is unknown.
  float GetDownloadProgress(std::string_view url);

  // Returns the total bytes downloaded during this session.
  size_t GetDownloadedSize();

  const Config& GetConfig();

  void SetConfig(Config config);

  // Causes all in-progress futures for loading a url to be cancelled early.
  // Not immediate, since this happens in the background.
  virtual void Shutdown();

 protected:
  FutureInterrupter GetGlobalInterrupter();

  std::shared_ptr<DownloadProgressInfo> download_progress_info_;

 private:
  // Can be used to interrupt all downloads when the url loader is shut down.
  // Currently, only the android implementation uses this, iOS uses a separate
  // mechanism for interrupting downloads, and desktop/wasm have not implemented
  // interruption.
  // TODO: Implement interruption on desktop.
  // TODO: Implement interruption on wasm.
  FutureInterrupter global_interrupter_;
  Config config_;
};

}  // namespace resources
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RESOURCES_URL_LOADER_H_
