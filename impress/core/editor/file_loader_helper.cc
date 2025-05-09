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

#include "core/editor/file_loader_helper.h"

#include <functional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/cord.h"
#include "absl/strings/match.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_split.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "core/assets/asset_ptr.h"
#include "core/assets/gltf/gltf_behavior_extension.h"
#include "core/async/future.h"
#include "core/common/buffer_access.h"
#include "core/common/file_helpers.h"
#include "core/common/invocable.h"
#include "core/common/platform_helpers.h"
#include "core/common/registry.h"
#include "core/config.h"
#include "core/editor/editor.h"
#include "core/editor/editor_touch.h"
#include "core/editor/events.h"
#include "core/editor/widgets/asset_library.h"
#include "core/lighting/environment_light.h"
#include "core/lighting/environment_light_factory.h"
#include "core/lighting/image_based_lighting_asset.h"
#include "core/lighting/image_based_lighting_asset_iblprefilter_loader.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/scene_metadata.h"
#include "core/resources/resource_manager.h"
#include "core/view/base_view.h"
#include "core/view/framework/animation/gltf_animator.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/gltf_asset.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/framework/assets/gltf_scene.h"
#include "core/view/framework/lighting/light_manager.h"
#include "core/view/framework/scene/scene_system.h"

namespace imp::editor {

namespace {

static constexpr absl::string_view kFileSuffix = "file://";
static constexpr absl::string_view kExtensionGltf = ".gltf";
static constexpr absl::string_view kExtensionGlb = ".glb";
static constexpr absl::string_view kExtensionIsf = ".isf";
static constexpr absl::string_view kExtensionTextproto = ".textproto";
static constexpr absl::string_view kExtensionExr = ".exr";
static constexpr absl::string_view kExtensionHdr = ".hdr";
static constexpr absl::string_view kExtensionTexturePng = ".png";
static constexpr absl::string_view kExtensionTextureJpg = ".jpg";
static constexpr absl::string_view kExtensionPly = ".ply";

static constexpr float kDefaultIndirectIntensity = 220.0f;

}  // namespace

FileType GetFileTypeFromName(absl::string_view filename) {
  if (absl::EndsWith(filename, kExtensionGltf) ||
      absl::EndsWith(filename, kExtensionGlb)) {
    return FileType::kGltf;
  } else if (absl::EndsWith(filename, kExtensionIsf)) {
    return FileType::kIsf;
  } else if (absl::EndsWith(filename, kExtensionTextproto)) {
    return FileType::kIsfTextproto;
  } else if (absl::EndsWith(filename, kExtensionExr) ||
             absl::EndsWith(filename, kExtensionHdr)) {
    return FileType::kHdrImage;
  } else if (absl::EndsWith(filename, kExtensionTexturePng) ||
             absl::EndsWith(filename, kExtensionTextureJpg)) {
    return FileType::kTexture;
  } else if (absl::EndsWith(filename, kExtensionPly)) {
    return FileType::kGSplat;
  }
  return FileType::kUnsupported;
}

// Adds the file to the AssetLibrary if it is not already present.
//
// Returns the path to the file in the AssetLibrary.
Future<std::string> AddFileToAssetLibraryIfNeeded(Editor& editor,
                                                  BaseView& view,
                                                  absl::string_view path,
                                                  absl::string_view file_name,
                                                  absl::string_view extension,
                                                  LoadFileSource source) {
  if (std::holds_alternative<absl::Cord>(source)) {
    absl::Cord& cord = std::get<absl::Cord>(source);
    // The data is coming from a Cord, so add it to the AssetLibrary.
    return Future<std::string>(
        editor.GetAssetLibrary()->AddResourceInCurrentDirectory(
            file_name, extension, cord.Flatten()));
  } else {
    LoadFileFromPathSource from_path_source =
        std::get<LoadFileFromPathSource>(source);
    switch (from_path_source) {
      case LoadFileFromPathSource::kAsset:
        // The data is already available in the asset library, so just return
        // the path.
        return Future<std::string>(std::string(path));
        break;
      case LoadFileFromPathSource::kLocalFile:
        std::string load_path = absl::StrCat(kFileSuffix, path);

        // Load the local file.
        return view.GetAssetManager().LoadResource(load_path).Then(
            [file_name = std::string(file_name),
             extension = std::string(extension),
             &editor](resources::Resource resource) -> std::string {
              // Add the local file to the AssetLibrary.
              return editor.GetAssetLibrary()->AddResourceInCurrentDirectory(
                  file_name, extension, resource.GetData().StringView());
            });
        break;
    }
  }
}

void BeginLoadingNode(Editor& editor, BaseView& view) {
  view.GetDispatcher().Send(editor::ModelLoadingEvent());
  editor.GetDispatcher().Send(editor::ModelLoadingEvent());
}

void EndLoadingNode(Editor& editor, BaseView& view, absl::string_view path,
                    NodeHandle node, Invocable<void(NodeHandle)> placement_func,
                    Invocable<void(NodeHandle)> post_loaded_func = {}) {
  EditorTouch(node);

  if (node->GetName().empty()) {
    // Split the path on '/' characters to extract the filename.
    std::vector<std::string> path_split = absl::StrSplit(path, '/');
    // Remove the file extension.
    std::vector<std::string> file = absl::StrSplit(path_split.back(), '.');
    node->SetName(file.front());
  }

  if (placement_func) {
    placement_func(node);
  }

  if (post_loaded_func) {
    post_loaded_func(node);
  }

  view.GetDispatcher().Send(editor::ModelLoadedEvent(node));
  editor.GetDispatcher().Send(editor::ModelLoadedEvent(node));
}

Future<absl::Status> LoadGltf(Editor& editor, BaseView& view,
                              absl::string_view path,
                              Invocable<void(NodeHandle)> placement_func) {
  BeginLoadingNode(editor, view);

  return view.GetAssetManager()
      .LoadModel(path, editor.GetGltfLoadOptions())
      .Then([&editor, &view, path = std::string(path),
             placement_func =
                 std::move(placement_func)](NodeHandle node) mutable {
        EndLoadingNode(
            editor, view, path, node, std::move(placement_func),
            [](NodeHandle node) {
              auto gltf_renderer = node->GetComponent<GltfRenderer>();
              auto gltf_scene = node->GetComponent<GltfScene>();
              auto scene_metadata = node->GetComponent<SceneMetadata>();

              scene_metadata->SetComponentAuthored(
                  GltfRenderer::IsfInfo::kTypeUrlHash, /*is_authored=*/true);

              gltf_scene->CreateAllNodes();

              if (!node->GetComponent<GltfBehaviorExtension>()) {
                // Start animation by default unless it contains
                // the KHR_behavior extension which may be controlling the
                // animation. If it is an ISF, we should assume that the
                // animations are set as needed.
                AssetPtr<GltfAsset> gltf_asset = gltf_renderer->GetGltfAsset();
                if (gltf_asset && gltf_asset->AnimationCount() != 0) {
                  GltfAnimator::PlayCommand play_command;
                  play_command.options.looping = true;
                  node->AddComponentWithState<GltfAnimator>(
                      {.starting_animation = play_command});
                  scene_metadata->SetComponentAuthored(
                      GltfAnimator::IsfInfo::kTypeUrlHash,
                      /*is_authored=*/true);
                }
              }
            });
      });
}

Future<absl::Status> LoadIsf(Editor& editor, BaseView& view,
                             absl::string_view path,
                             Invocable<void(NodeHandle)> placement_func) {
  BeginLoadingNode(editor, view);

  return view.GetSceneSystem()
      .LoadScene(path, {.metadata_mode = SceneSystem::MetadataMode::kInclude})
      .Then([&editor, &view, path = std::string(path),
             placement_func =
                 std::move(placement_func)](NodeHandle node) mutable {
        EndLoadingNode(editor, view, path, node, std::move(placement_func));
      });
}

#if IMP_RUNTIME(DEV)
Future<absl::Status> LoadTextproto(Editor& editor, BaseView& view,
                                   absl::string_view path,
                                   Invocable<void(NodeHandle)> placement_func) {
  BeginLoadingNode(editor, view);

  return view.GetAssetManager()
      .LoadResource(path)
      .Then([&view, path](resources::Resource resource) -> Future<NodeHandle> {
        return view.GetSceneSystem().LoadSceneFromTextproto(
            resource, path,
            {.metadata_mode = SceneSystem::MetadataMode::kInclude});
      })
      .Then([&editor, &view, path = std::string(path),
             placement_func =
                 std::move(placement_func)](NodeHandle node) mutable {
        EndLoadingNode(editor, view, path, node, std::move(placement_func));
      });
}
#endif

Future<absl::Status> LoadIbl(BaseView& view, absl::string_view path) {
  IblPrefilterLoader runtime_ibl_loader = IblPrefilterLoader();

  return view.GetAssetManager()
      .LoadAsset<ImageBasedLightingAsset>(path, runtime_ibl_loader)
      .Then([&view](AssetPtr<ImageBasedLightingAsset> ibl_asset) {
        // Create the EnvironmentLight from the asset.
        EnvironmentLightPtr environment_light_ptr =
            view.GetEnvironmentLightFactory().CreateEnvironmentLight(
                ibl_asset, kDefaultIndirectIntensity);

        // Assign the EnvironmentLight to the LightManager.
        view.GetLightManager().SetEnvironmentLight(
            std::move(environment_light_ptr));
      });
}

Future<absl::Status> LoadGSplat(Editor& editor, BaseView& view,
                                absl::string_view path,
                                Invocable<void(NodeHandle)> placement_func) {
  return imp::Future<absl::Status>(
      absl::UnimplementedError("GSplat loading is not supported yet."));
}

Future<absl::Status> LoadFile(BaseView& view, absl::string_view path,
                              LoadFileSource source,
                              Invocable<void(NodeHandle)> placement_func) {
  Editor& editor = *view.GetRegistry().Get<Editor>();

  absl::string_view file_name = RemoveDirectoryAndExtensionFromFilename(path);
  absl::string_view extension = GetExtensionFromFilename(path).substr(1);
  FileType file_type = GetFileTypeFromName(path);

  Future<std::string> added_asset = AddFileToAssetLibraryIfNeeded(
      editor, view, path, file_name, extension, std::move(source));

  return added_asset.Then(
      [&editor, &view, file_type, placement_func = std::move(placement_func)](
          const std::string& path) mutable -> Future<absl::Status> {
        switch (file_type) {
          case FileType::kGltf:
            return LoadGltf(editor, view, path, std::move(placement_func));
          case FileType::kIsf:
            return LoadIsf(editor, view, path, std::move(placement_func));
          case FileType::kIsfTextproto:
#if IMP_RUNTIME(DEV)
            return LoadTextproto(editor, view, path, std::move(placement_func));
#else
            return Future<absl::Status>(absl::InternalError(
                "Textproto loading is only supported in dev mode."));
#endif
          case FileType::kHdrImage:
            return LoadIbl(view, path);
          case FileType::kGSplat:
            return LoadGSplat(editor, view, path, std::move(placement_func));
          default:
            return Future<absl::Status>(absl::InternalError(
                absl::StrFormat("Unsupported file type: %s", path)));
        }
      });
}

}  // namespace imp::editor
