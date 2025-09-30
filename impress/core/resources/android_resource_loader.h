// Copyright 2025 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_RESOURCES_ANDROID_RESOURCE_LOADER_H_
#define THIRD_PARTY_IMPRESS_CORE_RESOURCES_ANDROID_RESOURCE_LOADER_H_

#include <optional>
#include <string>

#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/common/context.h"
#include "core/resources/url_loader.h"

namespace imp {
namespace resources {

class AndroidResourceLoader : public UrlLoader {
 public:
  explicit AndroidResourceLoader(const Context& context);

  // Returns true if the URL is an Android asset, resource, or file.
  static bool IsAndroidSpecialUrl(absl::string_view url);

  Future<absl::Cord> LoadUrl(const std::string& url) override;

 private:
  const Context& context_;

  static bool IsAndroidAsset(absl::string_view url);
  static bool IsAndroidResource(absl::string_view url);
  static bool IsAndroidFile(absl::string_view url);

  Future<absl::Cord> LoadAndroidAsset(absl::string_view asset_path);

  Future<absl::Cord> LoadAndroidResource(absl::string_view string_uri);

  Future<absl::Cord> LoadRawFile(absl::string_view path);
};

}  // namespace resources
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RESOURCES_ANDROID_RESOURCE_LOADER_H_
