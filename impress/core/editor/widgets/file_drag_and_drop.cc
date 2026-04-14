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

#include <optional>
#include <string>
#include <utility>

#include "core/common/log.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "dear_imgui/imgui_internal.h"
#include "filament/libs/math/include/math/TVecHelpers.h"
#include "core/common/registry.h"
#include "core/config.h"
#include "core/editor/editor_utils.h"
#include "core/editor/file_loader_helper.h"
#include "core/editor/file_type_loader.h"
#include "core/editor/file_type_registry.h"
#include "core/editor/ui/drag_and_drop.h"
#include "core/math/math.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node.h"
#include "core/view/base_view.h"
#include "core/view/framework/collision/collision_manager.h"
#include "core/view/framework/collision/ray_hit.h"
#include "core/view/view_events.h"

namespace imp::editor {

namespace {

// The threshold to snap the spawn_point to the origin.
constexpr float kSnapThreshold = 0.1f;

// Load a node from file and places the node at the cursor location.
void LoadFileAtCursor(BaseView& view, absl::string_view filename,
                      LoadFileSource source,
                      std::optional<float2> cursor = std::nullopt) {
  if (!cursor.has_value()) {
    cursor = view.GetSize() / 2.0f;
  }
  std::optional<RayHit> hit = std::nullopt;
  std::optional<float3> spawn_point =
      GetPointerIntersectionWithGroundPlane(view, *cursor);

  if (spawn_point.has_value()) {
    if (length(*spawn_point) < kSnapThreshold) {
      spawn_point = kZero3;
    }
    hit.emplace(0, kIdentityQuatf, *spawn_point, NodeHandle(), kUp);
  }

  LoadFile(view, filename, std::move(source), [hit](NodeHandle scene) {
    if (hit.has_value()) {
      // Place the scene at the orientation and
      // position of the cursor hit location.
      float3 target_center = hit->world_point;
      scene->SetWorldPosition(target_center);
    }
  }).KeptBy(&view);
}

}  // namespace

FileDragAndDrop::FileDragAndDrop(BaseView& view) : view_(view) {
#if IMP_PLATFORM(DESKTOP) || IMP_PLATFORM(WASM)
  // Handle drag-and-drop from the OS using the SDL DropFileEvent. This code
  // path does not handle ImGui-based drag-and-drop. See below for that path.

  view_.GetDispatcher().Connect(
      [this](const DropFileEvent& drop_file_event) {
        FileTypeRegistry& file_type_registry =
            view_.GetRegistry().GetOrCreate<FileTypeRegistry>();

        FileTypeLoader* file_type_loader =
            file_type_registry.GetFileTypeLoaderByPath(
                drop_file_event.filename);

        if (kFileTypeTexture.PathMatchesFileType(drop_file_event.filename)) {
          pending_drag_and_drop_payload_ =
              std::make_pair(DragAndDropType::kTexture,
                             absl::StrCat("file://", drop_file_event.filename));
        } else if (file_type_loader != nullptr) {
          // Note: when dropping from the OS, the cursor location is often
          // incorrect since the Impress app may not have focus. By default,
          // LoadFileAtCursor will use the screen center as the "cursor"
          // location in this case.
          LoadFileAtCursor(
              view_, drop_file_event.filename,
              drop_file_event.data.has_value()
                  ? LoadFileSource(std::move(*drop_file_event.data))
                  : LoadFileFromPathSource::kLocalFile);
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
          LoadFileAtCursor(
              view_, *payload, LoadFileFromPathSource::kAsset,
              float2(ImGui::GetIO().MousePos.x, ImGui::GetIO().MousePos.y));
        }
        ImGui::EndDragDropTarget();
      }

      ImGui::End();
    }
  }
}

}  // namespace imp::editor
