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

#include "core/resources/resource_manager.h"

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "sandboxed_api/file_toc.h"
#include "absl/base/no_destructor.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/cord.h"
#include "absl/strings/match.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/async/future_group.h"
#include "core/common/buffer_access.h"
#include "core/common/context.h"
#include "core/common/robin_set.h"
#include "core/common/trace.h"
#include "core/config.h"
#include "core/resources/resource_definition.h"
#include "core/resources/url_loader.h"
#include "core/view/utils/string_map.h"

#if IMP_PLATFORM(ANDROID) || IMP_PLATFORM(ROBOLECTRIC)
#include "core/resources/android_url_loader.h"
#elif IMP_PLATFORM(WASM)
#include "core/resources/emscripten_url_loader.h"
#elif IMP_PLATFORM(IOS)
#include "core/resources/ios_url_loader.h"
#else
#include "core/resources/curl_url_loader.h"
#endif

namespace imp {
namespace resources {

BufferAccess Resource::GetData() const {
  std::optional<absl::string_view> flat_data = data_->TryFlat();

  // Note: we assert that the data is flat because we explicitly flatten in
  // ResourceManager::Load.  Using Cord::Flatten here would result in a quiet
  // performance degradation because the Cord might be flattened on the main
  // thread instead of the background.
  assert(flat_data.has_value());

  return BufferAccess::Wrap(reinterpret_cast<const uint8_t*>(flat_data->data()),
                            flat_data->size());
}

bool Resource::operator==(const Resource& other) const {
  return data_ == other.data_;
}

bool Resource::operator!=(const Resource& other) const {
  return data_ != other.data_;
}

ResourceManager::ResourceManager(const Context& context)
    : url_loader_(nullptr) {
  // TODO Remove #if guards by linking platform-specific
  // implementations in BUILD files

#if IMP_PLATFORM(ANDROID) || IMP_PLATFORM(ROBOLECTRIC)
  SetUrlLoader(imp::resources::CreateAndroidLoader(context));
#elif IMP_PLATFORM(WASM)
  SetUrlLoader(imp::resources::CreateEmscriptenUrlLoader());
#elif IMP_PLATFORM(IOS)
  SetUrlLoader(imp::resources::CreateIosUrlLoader());
#else   // non-android, non-wasm, non-ios
  SetUrlLoader(imp::resources::CreateCurlLoader());
#endif  // !IMP_PLATFORM
}

ResourceManager::~ResourceManager() { Cleanup(); }

void ResourceManager::Cleanup() {
  if (url_loader_) {
    url_loader_->Shutdown();
    url_loader_.reset();
  }
}

void ResourceManager::SetUrlLoader(std::unique_ptr<UrlLoader> url_loader) {
  url_loader_ = std::move(url_loader);
}

void ResourceManager::SetUrlLoaderConfig(UrlLoader::Config config) {
  url_loader_->SetConfig(std::move(config));
}

std::unique_ptr<UrlLoader> ResourceManager::MoveUrlLoader() {
  return std::move(url_loader_);
}

bool ResourceManager::IsRemoteUrl(absl::string_view url) {
  return absl::StrContains(url, ":");
}

bool ResourceManager::IsRelativeUrl(absl::string_view url) {
  return absl::StartsWith(url, "/");
}

Future<absl::Cord> ResourceManager::GetContentFuture(
    absl::string_view resource_url, std::optional<FutureGroup> future_group) {
  // Check if an override has been set for this resource_url.
  StringMap<Future<absl::Cord>>& resource_to_content_override =
      ResourceToContentOverride();
  if (auto it = resource_to_content_override.find(resource_url);
      it != resource_to_content_override.end()) {
    return it->second;
  }
  // Check to see if this is a registered resource
  StringMap<absl::string_view>& identifiers_to_file_bytes =
      EmbeddedResourceIdentifierToFileBytes();
  if (auto it = identifiers_to_file_bytes.find(resource_url);
      it != identifiers_to_file_bytes.end()) {
    return Future<absl::Cord>(
        absl::MakeCordFromExternal(it->second, [](absl::string_view) {}));
  }

  // Check to see if this resource is actually an identifier mapped to a remote
  // url.
  StringViewMap<absl::string_view>& identifiers_to_urls =
      RemoteResourceIdentifierToUrl();
  auto itr = identifiers_to_urls.find(resource_url);
  absl::string_view real_resource_url =
      itr != identifiers_to_urls.end() ? itr->second : resource_url;

  if (!real_resource_url.empty() && url_loader_) {
    if (IsRemoteUrl(real_resource_url)) {
      // This was actually a URL and we have a loader.
      return url_loader_->LoadUrl(std::string(real_resource_url), future_group);
    }

#if IMP_PLATFORM(WASM)
    // For WASM, a relative path like "/link/to/icon.png" is also supported.
    if (IsRelativeUrl(real_resource_url)) {
      return url_loader_->LoadUrl(std::string(real_resource_url), future_group);
    }
#endif
  }

  // Tests assume even an invalid resource will post a future, so, do that.
  return Future<absl::Cord>(absl::NotFoundError(
      absl::StrFormat("Unable to find resource. Identifier=%s Url=%s",
                      resource_url, real_resource_url)));
}

Future<Resource> ResourceManager::Load(
    absl::string_view resource_url, std::optional<FutureGroup> future_group) {
  IMP_TRACE();
  return GetContentFuture(resource_url, future_group)
      .Then(
          [](absl::Cord data) {
            IMP_TRACE_BLOCK("Then");
            data.Flatten();
            return Resource(std::move(data));
          },
          {.executor = Executor::Type::kBackground,
           .future_group = future_group});
}

Future<Resource> ResourceManager::Load(
    const ResourceDefinition& resource_definition,
    std::optional<FutureGroup> future_group) {
  return Load(resource_definition.GetUrl(), future_group);
}

float ResourceManager::GetUrlLoaderProgress(size_t download_baseline) {
  return url_loader_->GetDownloadProgress(download_baseline);
}

float ResourceManager::GetUrlLoaderProgress(std::string_view resource_url) {
  return url_loader_->GetDownloadProgress(resource_url);
}

size_t ResourceManager::GetUrlLoaderDownloadedSize() {
  return url_loader_->GetDownloadedSize();
}

void ResourceManager::RegisterResource(absl::string_view name,
                                       absl::string_view data,
                                       bool allow_overwrite) {
  StringMap<absl::string_view>& identifiers_to_file_bytes =
      EmbeddedResourceIdentifierToFileBytes();

  if (!allow_overwrite) {
    auto it = identifiers_to_file_bytes.find(name);
    if (it != identifiers_to_file_bytes.end()) {
      assert(it->second == data);
    }
  }
  identifiers_to_file_bytes[std::string(name)] = data;

#if IMP_RUNTIME(DEV)
  RegisteredResources().insert(std::string(name));
#endif
}

void ResourceManager::UnregisterResource(absl::string_view name) {
  EmbeddedResourceIdentifierToFileBytes().erase(name);
#if IMP_RUNTIME(DEV)
  RegisteredResources().erase(std::string(name));
#endif
}

void ResourceManager::RegisterEmbeddedPackage(const char* package_name,
                                              const FileToc* (*create_fn)()) {
  // Return early if this package of resources has already been registered.
  auto [it, inserted] = RegisteredPackages().insert(package_name);
  if (!inserted) {
    return;
  }
  for (const FileToc* file_toc = create_fn(); file_toc->name != nullptr;
       ++file_toc) {
    absl::string_view data = {file_toc->data, file_toc->size};
    std::string name = file_toc->name;
    RegisterResource(name, data);
  }
}

void ResourceManager::RegisterRemotePackage(
    const char* package_name,
    absl::Span<const ResourceDefinition* const> resources) {
  // Return early if this package of resources has already been registered.
  auto [it, inserted] = RegisteredPackages().insert(package_name);
  if (!inserted) {
    return;
  }

  StringViewMap<absl::string_view>& identifiers_to_urls =
      RemoteResourceIdentifierToUrl();

  for (const ResourceDefinition* const resource_definition : resources) {
    identifiers_to_urls[resource_definition->GetIdentifier()] =
        resource_definition->GetUrl();
  }
}

void ResourceManager::AddFallback(absl::string_view url,
                                  const ResourceDefinition& fallback) {
  // Install a fake entry in the registry mapping url to fallback's data.
  StringMap<absl::string_view>& identifiers_to_file_bytes =
      EmbeddedResourceIdentifierToFileBytes();
  auto it = identifiers_to_file_bytes.find(fallback.GetUrl());
  if (it != identifiers_to_file_bytes.end()) {
    identifiers_to_file_bytes.emplace(url, it->second);
  }
}

void ResourceManager::RemoveFallback(absl::string_view url) {
  EmbeddedResourceIdentifierToFileBytes().erase(url);
}

absl::StatusOr<Resource> ResourceManager::LoadEmbeddedResource(
    absl::string_view resource_url) {
  // Check to see if this is a registered resource
  StringMap<absl::string_view>& identifiers_to_file_bytes =
      EmbeddedResourceIdentifierToFileBytes();
  if (auto it = identifiers_to_file_bytes.find(resource_url);
      it != identifiers_to_file_bytes.end()) {
    absl::Cord data =
        absl::MakeCordFromExternal(it->second, [](absl::string_view) {});
    data.Flatten();
    return Resource(std::move(data));
  }
  return absl::NotFoundError(
      absl::StrCat("Resource not found: %s", resource_url));
}

StringMap<absl::string_view>&
ResourceManager::EmbeddedResourceIdentifierToFileBytes() {
  static absl::NoDestructor<StringMap<absl::string_view>>
      identifiers_to_file_bytes;
  return *identifiers_to_file_bytes;
}

StringViewMap<absl::string_view>&
ResourceManager::RemoteResourceIdentifierToUrl() {
  static absl::NoDestructor<StringViewMap<absl::string_view>>
      identifiers_to_urls;
  return *identifiers_to_urls;
}

RobinSet<std::string>& ResourceManager::RegisteredPackages() {
  static absl::NoDestructor<RobinSet<std::string>> registered_packages;
  return *registered_packages;
}

StringMap<Future<absl::Cord>>& ResourceManager::ResourceToContentOverride() {
  static absl::NoDestructor<StringMap<Future<absl::Cord>>>
      resource_to_content_override;
  return *resource_to_content_override;
}

void ResourceManager::SetResourceContentOverride(
    absl::string_view resource_url, Future<absl::Cord> future_override) {
  StringMap<Future<absl::Cord>>& resource_to_content_override =
      ResourceToContentOverride();
  resource_to_content_override.erase(resource_url);
  resource_to_content_override.emplace(resource_url, future_override);
}

void ResourceManager::RemoveResourceContentOverride(
    absl::string_view resource_url) {
  ResourceToContentOverride().erase(resource_url);
}

#if IMP_RUNTIME(DEV)
absl::btree_set<std::string>& ResourceManager::RegisteredResources() {
  static absl::NoDestructor<absl::btree_set<std::string>> registered_resources;
  return *registered_resources;
}

const absl::btree_set<std::string>& ResourceManager::GetRegisteredResources()
    const {
  return RegisteredResources();
}
#endif

}  // namespace resources
}  // namespace imp
