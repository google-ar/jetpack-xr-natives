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

#ifndef THIRD_PARTY_IMPRESS_CORE_RESOURCES_RESOURCEMANAGER_H_
#define THIRD_PARTY_IMPRESS_CORE_RESOURCES_RESOURCEMANAGER_H_

#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "sandboxed_api/file_toc.h"
#include "core/common/log.h"
#include "absl/status/statusor.h"
#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "core/async/background_delete.h"
#include "core/async/future.h"
#include "core/common/buffer_access.h"
#include "core/common/context.h"
#include "core/common/robin_set.h"
#include "core/config.h"
#include "core/resources/resource_definition.h"
#include "core/resources/url_loader.h"
#include "core/view/utils/string_map.h"

#if IMP_RUNTIME(DEV)
#include "absl/container/btree_set.h"
#endif

namespace imp {
namespace resources {

class ResourceManager;

// Represents a resource that has been requested via ResourceManager::Load.
//
// Note that a Resource is a lightweight handle object connected with
// an in-memory representation of the loaded resource. Once resource
// data is no longer required by an application, the associated
// Resource object will be destroyed.
//
// Resources can only be obtained from the ResourceManager, or by copying
// another resource.  Resources are always valid.
class Resource final {
 public:
  explicit Resource(absl::Cord&& data)
      : data_(
            imp::MakeSharedWithBackgroundDeleter<absl::Cord>(std::move(data))) {
  }

  Resource() = delete;

  Resource(const Resource&) = default;
  Resource(Resource&&) = default;

  Resource& operator=(const Resource&) = default;
  Resource& operator=(Resource&&) = default;

  ~Resource() = default;

  bool operator==(const Resource& other) const;
  bool operator!=(const Resource& other) const;

  // Returns a BufferAccess object for reading the Resource's data.
  // Note: The BufferAccess does not own the data, and is only valid during
  // the lifetime of the resource.
  BufferAccess GetData() const;

 private:
  std::shared_ptr<absl::Cord> data_;
};

// Provides cross-platform support for loading and caching network-based
// resources using built-in platform implementations under the hood.
//
// Example Usage:
//    // large_model header generated from imp_resource definition in app BUILD.
//    // See imp/resources/resources.bzl for reference.
//    #include "my/imp/app/large_model.h"
//
//    // Long lived manager object, single instance per app.
//    resources::ResourceManager resource_manager_{Context()};
//
//    // Load a model file obtaining a future representing the eventual data.
//    future_large_model_ = resource_manager_.Load(my::imp::app::kLargeModel);
//
//    // Register for completion callbacks.
//    future_large_model_.Then([](const absl::StatusOr<Resource>& status) {
//      if (!status.ok()) {
//        // Load completed with error.
//        IMP_LOG(imp::ERROR) << status;
//      } else {
//        IMP_LOG(imp::INFO) << "Success";
//      }
//      ...
//    }).KeptBy(view);
//
// Threading: The ResourceManager is thread safe.
class ResourceManager final {
 public:
  // Creates a resource manager and configure it with a context.  The
  // context is provided so that a URL loader appropriate for the
  // current application can be created.
  explicit ResourceManager(const Context& context);

  // Cleans up the resource manager.  Any pending resource loads will be
  // cancelled when the associated resource manager is cleaned up.
  ~ResourceManager();

  ResourceManager(const ResourceManager&) = delete;
  ResourceManager& operator=(const ResourceManager&) = delete;

  // Sets the url loader to be used for loading from the network with this
  // resource manager.  If unset, network requests will fail.
  void SetUrlLoader(std::unique_ptr<UrlLoader> url_loader);

  // Sets a config globally for the url loader. The config will be applied to
  // all urls handled by the url loader if relevant.
  void SetUrlLoaderConfig(UrlLoader::Config config);

  // Moves the Url Loader out of the resource manager and returns it.
  std::unique_ptr<UrlLoader> MoveUrlLoader();

  // Loads the given resource asynchronously into memory on a thread based
  // on the background executor.  The resource is loaded from the packaged
  // location if applicable, or from the url if the packaged resource is
  // unavailable or undefined (i.e. the src is nullptr).
  Future<Resource> Load(const ResourceDefinition& resource_definition);

  // Loads the given URI into memory on a thread based on the background
  // executor.
  Future<Resource> Load(absl::string_view resource_url);

  // Returns the loading progress of pending downloads as a fraction, with
  // download_baseline establishing 0%.
  float GetUrlLoaderProgress(size_t download_baseline);

  // Returns the loading progress of the indicated url as a fraction of its
  // entire requested size. Returns 0 if the url isn't found or the size of the
  // entire url is unknown.
  float GetUrlLoaderProgress(std::string_view resource_url);

  // Returns the number of bytes downloaded so far by the session.
  size_t GetUrlLoaderDownloadedSize();

  // Registers a resource with the given name and externally-stored data.
  // If allow_overwrite is true, the resource with the given name will be
  // replaced with new data (asserts data is the same otherwise). The old
  // resource will continue to work for anyone who has already loaded it.
  static void RegisterResource(absl::string_view name, absl::string_view data,
                               bool allow_overwrite = false);

  // Unregisters the resource with the given name.
  // TODO : Investigate why AddFallback and RegisterResource are
  // two separate methods and determine if they can be merged.
  static void UnregisterResource(absl::string_view name);

  // Registers embedded resources with the ResourceManager.
  static void RegisterEmbeddedPackage(const char* package_name,
                                      const FileToc* (*create_fn)());

  // Registers remote resources with the ResourceManager.
  // Maps the identifier to the url so that a remote resource can be referenced
  // with the same string identifier both when it's embedded and when it isn't.
  static void RegisterRemotePackage(
      const char* package_name,
      absl::Span<const ResourceDefinition* const> resources);

  // Adds a fallback for url.  If url cannot be loaded, fallback will be
  // loaded instead.  Fallback must be a registered resource.
  static void AddFallback(absl::string_view url,
                          const ResourceDefinition& fallback);

  // Removes the fallback mapping for url.
  static void RemoveFallback(absl::string_view url);

  // Sets an override of the future to be returned when certain resource needs
  // to be loaded. This function is mainly meant to help with testing certain
  // scenarios. WARNING: Remember to call RemoveResourceContentOverride so other
  // tests are not affected if not desired.
  static void SetResourceContentOverride(absl::string_view resource_url,
                                         Future<absl::Cord> future_override);

  // Removes the future override for a resource.
  static void RemoveResourceContentOverride(absl::string_view resource_url);

  // Returns true if the given resource url is remote - i.e. must be downloaded.
  static bool IsRemoteUrl(absl::string_view url);

  // Returns true if the given resource url is relative remote url that should
  // be downloaded.
  static bool IsRelativeUrl(absl::string_view url);

  // Load an embedded resource by url.
  static absl::StatusOr<Resource> LoadEmbeddedResource(
      absl::string_view resource_url);

  void Cleanup();

#if IMP_RUNTIME(DEV)
  const absl::btree_set<std::string>& GetRegisteredResources() const;
#endif

 private:
  Future<Resource> Load(
      const std::function<absl::StatusOr<absl::Cord>()>& load_fn);

  Future<absl::Cord> GetContentFuture(absl::string_view resource_url);

  // Maps the identifier of a ResourceDefinition to its file bytes.
  static StringMap<absl::string_view>& EmbeddedResourceIdentifierToFileBytes();

  // Maps the identifier of a ResourceDefinition to its url.
  static StringViewMap<absl::string_view>& RemoteResourceIdentifierToUrl();

  static RobinSet<std::string>& RegisteredPackages();

  static StringMap<Future<absl::Cord>>& ResourceToContentOverride();

#if IMP_RUNTIME(DEV)
  static absl::btree_set<std::string>& RegisteredResources();
#endif

  std::unique_ptr<UrlLoader> url_loader_;
};

}  // namespace resources
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RESOURCES_RESOURCEMANAGER_H_
