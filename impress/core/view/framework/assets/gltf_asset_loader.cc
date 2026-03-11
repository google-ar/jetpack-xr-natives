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

#include "core/view/framework/assets/gltf_asset_loader.h"

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "filament/filament/include/filament/Engine.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/async/future_common.h"
#include "core/collision/collision_accelerator_provider.h"
#include "core/common/registry.h"
#include "core/common/trace.h"
#include "core/config.h"
#include "core/loader/loader.h"
#include "core/loader/loader_creator.h"
#include "core/loader/loader_in_process.h"
#include "core/loader/loader_options.h"
#include "core/material_library/generic_material.h"
#include "core/material_library/material_package.h"
#include "core/material_library/material_param_value.h"
#include "core/model/model_data.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/resources/resource_manager.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/gltf_asset.h"
#include "core/view/framework/assets/gltf_state.proto.imp.h"
#include "core/view/framework/generic_lite_materials.h"
#include "core/view/framework/generic_materials.h"
#include "core/window/filament_host.h"
#include "mediapipe/framework/port/status_macros.h"
#include "mediapipe/framework/deps/clock.h"

namespace imp {
using absl::StatusOr;
using model::ModelData;
using resources::Resource;

Future<std::unique_ptr<GltfAsset>> GltfAssetLoader::Load(
    BaseView* view, absl::string_view asset_url,
    Future<resources::Resource> resource_future, GltfAsset::LoadOptions options,
    mediapipe::Clock* clock /*= nullptr*/) {
  IMP_TRACE();
  // If it hasn't already been created, create the material package for the type
  // of materials being loaded. This is done right away so that the zip file
  // containing the materials can be loaded while the glTF is loaded/parsed.
  MaterialPackage* material_package = nullptr;
  if (options.materials_url_override) {
    auto itr = custom_material_packages_.find(*options.materials_url_override);
    if (itr != custom_material_packages_.end()) {
      material_package = itr->second.get();
    } else {
      Future<resources::Resource> materials_zip =
          view->GetAssetManager().LoadResource(*options.materials_url_override);
      material_package =
          custom_material_packages_
              .emplace(*options.materials_url_override,
                       std::make_unique<MaterialPackage>(
                           materials_zip,
                           GltfAsset::kDefaultMaterialPreCompileOptions))
              .first->second.get();
    }
  } else if (options.use_lite_materials) {
    if (!lite_material_package_) {
      lite_material_package_ = std::make_unique<MaterialPackage>(
          view->GetAssetManager().LoadResource(
              materials::kCompiledImpDefaultLiteGltfMaterialsZip),
          GltfAsset::kDefaultMaterialPreCompileOptions);
    }
    material_package = lite_material_package_.get();
  } else {
    if (!material_package_) {
      material_package_ = std::make_unique<MaterialPackage>(
          view->GetAssetManager().LoadResource(
              materials::kCompiledImpDefaultGltfMaterialsZip),
          GltfAsset::kDefaultMaterialPreCompileOptions);
    }
    material_package = material_package_.get();
  }

  // Remove the query params from the Url because when a gltf references other
  // assets we must append the referenced asset relative to the base url before
  // the query params.
  size_t query_start_index = asset_url.find_first_of('?');
  std::string id(asset_url);
  std::string query_string = "";
  if (query_start_index != std::string::npos) {
    query_string = id.substr(query_start_index, id.size() - query_start_index);
    id = id.substr(0, query_start_index);
  }

  if (clock == nullptr) {
    clock = mediapipe::Clock::RealClock();
  }

  // Create a LoadEvent to share between future stages and populate with data.
  // The event gets sent during the final future stage.
  // Don't love using a shared_ptr for this, but don't have a better solution.
  auto load_event = std::make_shared<GltfAsset::LoadEvent>();
  load_event->asset_id = std::string(asset_url);
  load_event->start_download_materials_and_model_time = load_event->start_time =
      clock->TimeNow();

  // Treat local URLs (e.g. embedded assets) as trusted.
  bool kUseSandboxedLoader = IMP_PLATFORM(ANDROID) &&
                             resources::ResourceManager::IsRemoteUrl(asset_url);

  auto transcode_compression_type =
      loader::LoaderOptions::TextureTranscodeCompressionType::Unknown;
  if (view->GetHost()) {
    filament::Engine* engine = BaseView::GetSharedEngine();
    if (engine) {
      transcode_compression_type =
          loader::Loader::GetTextureTranscodeCompressionType(*engine);
    }
  }

  // Track when shaders finish downloading.
  Future<absl::Time> end_download_materials_time_future =
      material_package->GetMaterialsZipFuture().Then(
          [clock](const absl::StatusOr<Resource>& status_or_shaders) {
            return clock->TimeNow();
          },
          {.executor = Executor::Type::kImmediate});

  Future<loader::GetLoaderFn> get_loader_future;
  if (kUseSandboxedLoader && sandboxed_gltf_loader_creator_) {
    get_loader_future = sandboxed_gltf_loader_creator_->Create(*view);
  } else {
    get_loader_future.Return(&loader::CreateLoaderInProcess);
  }

  LoadAssetFn load_missing_asset = [view,
                                    query_string](absl::string_view asset) {
    std::string asset_with_query_string = std::string(asset) + query_string;
    return Future<resources::Resource>::Schedule(
        [view, asset_with_query_string]() {
          return view->GetAssetManager().LoadResource(asset_with_query_string);
        });
  };

  loader::LoaderOptions loader_options;
  loader_options.compression_type = transcode_compression_type;
  loader_options.use_lite_materials = options.use_lite_materials;
  loader_options.exclude_excess_nodes = options.exclude_excess_nodes;
  loader_options.remove_shadow_planes = options.remove_shadow_planes;
  loader_options.vertex_access_flags = options.vertex_access_flags;
  if (options.collider_mode ==
          GltfState::ColliderMode::GLTF_COLLIDER_TRIANGLES_PER_MESH ||
      options.collider_mode ==
          GltfState::ColliderMode::GLTF_COLLIDER_MESH_COLLISION_ACCELERATOR) {
    loader_options.vertex_access_flags |=
        GltfAsset::VertexAccessFlags::kPosition;
  }

  return get_loader_future.Merge(resource_future)
      .Then([view, id, material_package, load_event, clock, loader_options](
                std::tuple<loader::GetLoaderFn, Resource> tuple) mutable
                -> absl::StatusOr<
                    std::shared_ptr<GltfAssetLoader::LoadInProgress>> {
        IMP_TRACE_BLOCK("Then");
        auto [get_loader_fn, resource] = std::move(tuple);

        load_event->end_download_model_time = clock->TimeNow();
        load_event->num_bytes_downloaded += resource.GetData().Size();

        absl::StatusOr<std::unique_ptr<loader::Loader>> loader =
            get_loader_fn(*view, id, resource.GetData(), material_package,
                          std::move(loader_options));
        MP_RETURN_IF_ERROR(loader.status());

        return std::make_shared<GltfAssetLoader::LoadInProgress>(
            view->GetContext(), *std::move(loader), std::move(resource));
      })
      // On a background thread, call loader.Load.
      .Then(
          [load_missing_asset, clock](
              std::shared_ptr<GltfAssetLoader::LoadInProgress> load_in_progress)
              -> Future<std::shared_ptr<GltfAssetLoader::LoadInProgress>> {
            IMP_TRACE_BLOCK("Then");
            load_in_progress->start_parse_time_ = clock->TimeNow();
            Future<absl::Status> load_in_progress_future =
                load_in_progress->Load(load_missing_asset, clock,
                                       load_in_progress);
            return load_in_progress_future.Then(
                [load_in_progress]()
                    -> StatusOr<
                        std::shared_ptr<GltfAssetLoader::LoadInProgress>> {
                  MP_RETURN_IF_ERROR(load_in_progress->LoadAnimations());
                  return std::move(load_in_progress);
                },
                {.executor = Executor::Type::kBackground});
          },
          {
              .executor = Executor::Type::kBackground,
          })
      // On the foreground thread, call loader.CreateGltfAsset.
      .Then([view, load_event, collider_mode = options.collider_mode](
                std::shared_ptr<GltfAssetLoader::LoadInProgress>
                    load_in_progress) {
        // Copy loading information over from the GltfAssetLoader. This is
        // done here so that we only access the load_event from a single
        // thread.
        IMP_TRACE_BLOCK("Then");
        load_event->start_parse_time = load_in_progress->start_parse_time_;
        load_event->start_download_deps_time =
            load_in_progress->start_download_deps_time_;
        load_event->end_download_deps_time =
            load_in_progress->end_download_deps_time_;
        {
          absl::ReaderMutexLock lock(
              load_in_progress->num_bytes_downloaded_mutex_);
          load_event->num_bytes_downloaded +=
              load_in_progress->num_bytes_downloaded_;
        }

        return load_in_progress->CreateGltfAsset(view, load_in_progress,
                                                 collider_mode);
      })
      .Merge(end_download_materials_time_future)
      .Then(
          [load_event, clock, view](
              absl::StatusOr<std::tuple<std::unique_ptr<GltfAsset>, absl::Time>>
                  tuple_or) -> absl::StatusOr<std::unique_ptr<GltfAsset>> {
            IMP_TRACE_BLOCK("Then");
            // Report the GltfAsset load by dispatching the LoadEvent.
            // Handle the StatusOr to ensure we do this on both success and
            // failure.
            load_event->status = tuple_or.status();
            load_event->end_time = load_event->end_parse_time =
                clock->TimeNow();

            if (!tuple_or.ok()) {
              view->GetDispatcher().Send(*load_event);
              return tuple_or.status();
            }

            auto [loaded_asset, end_download_materials_time] =
                std::move(tuple_or.value());

            load_event->end_download_materials_time =
                end_download_materials_time;

            view->GetDispatcher().Send(*load_event);

            return std::move(loaded_asset);
          });
}

void GltfAssetLoader::SetSandboxedGltfLoader(
    std::unique_ptr<loader::LoaderCreator> sandboxed_gltf_loader_creator) {
  sandboxed_gltf_loader_creator_ = std::move(sandboxed_gltf_loader_creator);
}

GltfAssetLoader::LoadInProgress::LoadInProgress(
    const Context& context, std::unique_ptr<loader::Loader> loader,
    Resource resource)
    : num_bytes_downloaded_(0),
      context_(context),
      loader_(std::move(loader)),
      resource_(std::move(resource)) {}

Future<absl::Status> GltfAssetLoader::LoadInProgress::LoadHelper(
    GltfAssetLoader::LoadAssetFn load_missing_asset,
    std::shared_ptr<LoadInProgress> load_in_progress) {
  std::vector<std::string> missing_assets;
  return loader_->TryLoad(&missing_assets, [load_in_progress]() {})
      .Then([this, load_missing_asset, missing_assets, load_in_progress](
                absl::Status missing_assets_status) -> Future<absl::Status> {
        // No missing resources means everything loaded successfully
        if (missing_assets_status.ok() && missing_assets.empty()) {
          return Future<absl::Status>(absl::OkStatus());
        }
        // There aren't any missing assets but there was an error loading
        // the assets
        if (missing_assets.empty()) {
          return Future<absl::Status>(missing_assets_status);
        }
        // `missing_assets_status` can contain an error such as `Resource
        // not found`, in which case we should load the resources with the
        // provided LoadAssetFn and try again

        std::vector<Future<absl::Status>> missing_assets_futures;
        missing_assets_futures.reserve(missing_assets.size());
        for (const std::string& missing_resource : missing_assets) {
          missing_assets_futures.push_back(
              load_missing_asset(missing_resource)
                  .Then([this, missing_resource](resources::Resource resource) {
                    // The loaded resource needs to outlive the Load future
                    // such as for use in DecodeImage, thus we need to
                    // retain a reference to it.
                    missing_asset_resources_.push_back(resource);
                    {
                      absl::WriterMutexLock lock(num_bytes_downloaded_mutex_);
                      num_bytes_downloaded_ += resource.GetData().Size();
                    }
                    return loader_->AddMissingResource(missing_resource,
                                                       resource.GetData());
                  }));
        }

        return Future<absl::Status>::CombineList(missing_assets_futures)
            .Then([this, load_missing_asset, load_in_progress]() {
              // If missing_assets status is ok then we should continue,
              // otherwise we let the error status bubble up because the
              // load actually failed.
              return LoadHelper(load_missing_asset, load_in_progress);
            });
      });
}

Future<absl::Status> GltfAssetLoader::LoadInProgress::Load(
    GltfAssetLoader::LoadAssetFn load_missing_asset, mediapipe::Clock* clock,
    std::shared_ptr<LoadInProgress>& load_in_progress) {
  Future<absl::Status> assets_loaded_future;
  start_download_deps_time_ = clock->TimeNow();
  assets_loaded_future =
      LoadHelper(load_missing_asset, load_in_progress).Then([this, clock]() {
        end_download_deps_time_ = clock->TimeNow();
      });
  return assets_loaded_future;
}

absl::Status GltfAssetLoader::LoadInProgress::LoadAnimations() {
  IMP_TRACE();
  const auto animation_names = loader_->GetAnimationNames();
  for (size_t index = 0, count = animation_names.size(); index < count;
       ++index) {
    MP_ASSIGN_OR_RETURN(std::unique_ptr<animation::GltfAnimation> animation,
                     loader_->CreateAnimation(index));
    loaded_animations_.push_back(std::move(animation));
  }
  return absl::OkStatus();
}

Future<std::unique_ptr<GltfAsset>>
GltfAssetLoader::LoadInProgress::CreateGltfAsset(
    BaseView* view, std::shared_ptr<LoadInProgress>& load_in_progress,
    GltfState::ColliderMode collider_mode) {
  window::FilamentHost* filament_host = view->GetHost();
  IMP_TRACE();
  if (filament_host->IsCleaningUp()) {
    return Future<std::unique_ptr<GltfAsset>>(
        absl::CancelledError("Host Cleaning Up"));
  }

  // The loader must stay alive until the gltf asset is fully loaded
  // by Filament's rendering thread, which outlives the lifetime of
  // the returned future. Pass a callback through the loader all the way to
  // Filament that Filament calls after loading has completed.
  // The callback captures the loader to ensure it stays alive until
  // Filament has finished loading the asset.
  // Unfortunately, this requires the loader to be a shared_ptr
  // because a unique_ptr cannot be captured in an std::function.
  //
  // This is an anti-pattern, beware of creating circular references that
  // can't get cleaned up with the std::shared_ptr.
  // TODO: Explore refactoring loader API to remove this.
  Future<std::unique_ptr<GltfAsset>> gltf_asset_future =
      loader_->CreateModel(filament_host->GetEngine(), [load_in_progress]() {})
          .Then([load_in_progress](
                    absl::StatusOr<std::unique_ptr<ModelData>> model_data)
                    -> absl::StatusOr<std::unique_ptr<GltfAsset>> {
            IMP_TRACE_BLOCK("Then");
            if (!model_data.ok()) {
              // When loading is cancelled, it's possible that the callback
              // passed into CreateModel will never be called because
              // loading never finishes and the resources aren't all
              // uploaded to filament. When this happens, there is a
              // circular reference between load_in_progress and loader_
              // that causes a memory leak. Explicitly remove the callback
              // to clean up the circular reference and prevent the leak.
              load_in_progress->loader_->RemoveWhenFullyLoadedCallback();
              return model_data.status();
            }

            const auto animation_names =
                load_in_progress->loader_->GetAnimationNames();
            auto builder = GltfAsset::Builder(animation_names.size());

            GenericMaterialListing shared_materials;
            shared_materials.reserve((*model_data)->Materials().size());

            for (const GenericMaterialPtr& generic_material :
                 (*model_data)->Materials()) {
              shared_materials.emplace_back(generic_material->Duplicate());
            }

            builder.SharedMaterials(std::move(shared_materials));

            builder.Model(*std::move(model_data));

            for (auto& animation_name : animation_names) {
              size_t animation_index =
                  &animation_name - &animation_names.front();

              if (animation_index <
                  load_in_progress->loaded_animations_.size()) {
                builder.Animation(
                    animation_index, animation_name,
                    std::move(
                        load_in_progress->loaded_animations_[animation_index]));
              }
            }

            return builder.Build();
          });

  if (collider_mode ==
      GltfState::ColliderMode::GLTF_COLLIDER_MESH_COLLISION_ACCELERATOR) {
    gltf_asset_future =
        gltf_asset_future.Then([view](std::unique_ptr<GltfAsset> gltf_asset) {
          gltf_asset->BuildMeshCollisionAccelerators(
              view->GetRegistry().GetOrCreate<CollisionAcceleratorProvider>());
          return gltf_asset;
        });
  }

  return gltf_asset_future;
}

}  // namespace imp
