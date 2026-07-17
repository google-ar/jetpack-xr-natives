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

#include "core/editor/widgets/file_drag_and_drop.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "dear_imgui/imgui_internal.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/common/buffer_access.h"
#include "core/common/file_helpers.h"
#include "core/common/registry.h"
#include "core/config.h"
#include "core/editor/editor.h"
#include "core/editor/file_loader_helper.h"
#include "core/editor/file_type_loader.h"
#include "core/editor/file_type_registry.h"
#include "core/editor/ui/drag_and_drop.h"
#include "core/editor/widgets/asset_library.h"
#include "core/materials/compiler/runtime_material_compiler.h"
#include "core/materials/compiler/runtime_material_compiler_creator.h"
#include "core/materials/compiler/schemas/material_compiler_ipc_generated.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node.h"
#include "core/view/base_view.h"
#include "core/view/framework/collision/collision_manager.h"
#include "core/view/view_events.h"

namespace imp::editor {

namespace {}  // namespace

FileDragAndDrop::FileDragAndDrop(BaseView& view) : view_(view) {
#if IMP_PLATFORM(DESKTOP) || IMP_PLATFORM(WASM)
  // Handle drag-and-drop from the OS using the SDL DropFileEvent. This code
  // path does not handle ImGui-based drag-and-drop. See below for that path.

  view_.GetDispatcher().Connect(
      [this, &view](const DropFileEvent& drop_file_event) {
        std::string filename = drop_file_event.filename;
        FileTypeRegistry& file_type_registry =
            view_.GetRegistry().GetOrCreate<FileTypeRegistry>();

        FileTypeLoader* file_type_loader =
            file_type_registry.GetFileTypeLoaderByPath(
                drop_file_event.filename);

        if (kFileTypeTexture.PathMatchesFileType(drop_file_event.filename)) {
          pending_drag_and_drop_payload_ =
              std::make_pair(DragAndDropType::kTexture,
                             absl::StrCat("file://", drop_file_event.filename));
        } else if (kFileTypeMat.PathMatchesFileType(filename)) {
#if IMP_PLATFORM(DESKTOP)
          GetOrCreateMaterialCompiler(view_)
              .Then(
                  [this, filename](
                      RuntimeMaterialCompiler* runtime_material_compiler) {
                    ProcessMatFile(filename);
                  },
                  Executor::Type::kBackground)
              .KeptBy(&view);
#else
          IMP_LOG(imp::ERROR) << ".mat files not supported on this platform";
#endif
        } else if (file_type_loader != nullptr) {
          // Note: when dropping from the OS, the cursor location is often
          // incorrect since the Impress app may not have focus. By default,
          // LoadAssetFileAtCursor will use the screen center as the "cursor"
          // location in this case.
          LoadAssetFileAtCursor(
              view_, drop_file_event.filename,
              drop_file_event.data.has_value()
                  ? LoadAssetFileSource(std::move(*drop_file_event.data))
                  : LoadAssetFileFromPathSource::kLocalFile);
        } else {
          IMP_LOG(imp::ERROR) << "Unsupported file dropped: "
                     << drop_file_event.filename;
        }
      },
      this);
#endif
}

void FileDragAndDrop::DrawImGui() {
  if (pending_drag_and_drop_payload_.has_value()) {
    // Create a momentary drag-and-drop payload of the proper type.
    if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceExtern)) {
      SetDragAndDropPayload(pending_drag_and_drop_payload_->first, "",
                            pending_drag_and_drop_payload_->second);
      pending_drag_and_drop_payload_.reset();
      ImGui::EndDragDropSource();
    }
  }
  if (ImGui::IsDragDropActive()) {
    // Create a drag-and-drop target over the entire main scene, underneath all
    // the rest of the UI. This allows models and isfs to be dropped into the
    // world and placed at the cursor location. This code path only supports
    // drag-and-drop from other ImGui sources. For OS-specific drag-and-drop,
    // see the DropFileEvent event handler code above.
    ImGuiIO& io = ImGui::GetIO();
    ImVec2 window_size = ImVec2(io.DisplaySize.x, io.DisplaySize.y);
    ImVec2 window_position = ImVec2(0, 0);
    ImGui::SetNextWindowSize(window_size);
    ImGui::SetNextWindowPos(window_position, ImGuiCond_Always, ImVec2(0, 0));
    ImGui::SetNextWindowBgAlpha(0.0f);
    if (ImGui::Begin("##drag-and-drop-target-main-scene", nullptr,
                     ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar |
                         ImGuiWindowFlags_NoResize |
                         ImGuiWindowFlags_AlwaysAutoResize |
                         ImGuiWindowFlags_NoFocusOnAppearing |
                         ImGuiWindowFlags_NoBringToFrontOnFocus)) {
      ImGuiWindow* window = ImGui::GetCurrentWindow();
      if (ImGui::BeginDragDropTargetCustom(window->ContentRegionRect,
                                           window->ID)) {
        std::optional<std::string> payload =
            AcceptDragAndDropPayload(DragAndDropType::kNodeAsset);
        if (payload.has_value()) {
          LoadAssetFileAtCursor(
              view_, *payload, LoadAssetFileFromPathSource::kAsset,
              float2(ImGui::GetIO().MousePos.x, ImGui::GetIO().MousePos.y));
        }
        ImGui::EndDragDropTarget();
      }

      ImGui::End();
    }
  }
}

Future<RuntimeMaterialCompiler*> FileDragAndDrop::GetOrCreateMaterialCompiler(
    BaseView& view) {
  

  // If we are not yet waiting for a compiler to be created, then create one.
  if (!runtime_material_compiler_future_.has_value()) {
    runtime_material_compiler_future_ =
        RuntimeMaterialCompilerCreator::Create(view).Then(
            [this](absl::StatusOr<std::unique_ptr<RuntimeMaterialCompiler>>
                       compiler) -> absl::StatusOr<RuntimeMaterialCompiler*> {
              if (!compiler.ok()) {
                // In a failure case, reset the future to indicate that we are
                // no longer waiting for the compiler to be created.
                runtime_material_compiler_future_ = std::nullopt;
                runtime_material_compiler_.reset();
                IMP_LOG(imp::ERROR) << "Failed to create runtime material compiler: "
                           << absl::StrCat(compiler.status().ToString());
                return compiler.status();
              }
              runtime_material_compiler_ = *std::move(compiler);
              return runtime_material_compiler_.get();
            });
  }

  return runtime_material_compiler_future_.value();
}

#if IMP_PLATFORM(DESKTOP)
void FileDragAndDrop::ProcessMatFile(std::string filename) {
  absl::StatusOr<BufferAccess> buffer_access = LoadFile(filename);
  if (!buffer_access.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to load .mat file: " << buffer_access.status();
    return;
  }
  if (buffer_access.ok()) {
    IMP_LOG(imp::INFO) << "Compiling .mat file: " << filename;
    runtime_material_compiler_
        ->CompileMaterialToBytes(
            buffer_access->StringView(), schemas::Platform::Desktop,
            schemas::TargetApi::OpenGL | schemas::TargetApi::Vulkan |
                schemas::TargetApi::Metal)
        .Then([filename, this](std::vector<uint8_t> bytes) {
          IMP_LOG(imp::INFO) << "Compiled .mat file: " << filename;
          auto editor = view_.GetRegistry().Get<Editor>();
          if (AssetLibrary* asset_library = editor->get().GetAssetLibrary()) {
            std::string name = filename;
            size_t slash = name.find_last_of('/');
            if (slash != std::string::npos) {
              name = name.substr(slash + 1);
            }
            size_t dot = name.find_last_of('.');
            if (dot != std::string::npos) {
              name = name.substr(0, dot);
            }
            asset_library->AddResourceInCurrentDirectory(
                name, ".cmat",
                absl::string_view(reinterpret_cast<const char*>(bytes.data()),
                                  bytes.size()));
          }
        })
        .KeptBy(&view_);
  }
}
#endif  // IMP_PLATFORM(DESKTOP)

}  // namespace imp::editor
