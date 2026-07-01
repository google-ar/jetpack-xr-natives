/*
 * Copyright 2026 Google LLC
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

#include "core/editor/ui/drag_and_drop_material.h"

#include <optional>
#include <string>

#include "absl/strings/str_cat.h"
#include "core/common/registry.h"
#include "core/editor/ui/drag_and_drop.h"
#include "core/editor/widgets/asset_type_helpers.h"
#include "core/proto/proto_reader.h"
#include "core/render/material_registry.h"
#include "core/scene_handles/material_handle.h"
#include "core/view/base_view.h"

namespace imp::editor {

std::optional<DragAndDropMaterialResult> AcceptDragAndDropPayloadMaterial(
    BaseView* view) {
  if (!view) return std::nullopt;

  auto& registry = view->GetRegistry().GetOrCreate<MaterialRegistry>();

  // Try MaterialHandle payload first.
  std::string payload_type = absl::StrCat(
      kProtoDragAndDropScheme, GetPayloadTypeForMessage<MaterialHandle>());

  std::optional<std::string> payload = AcceptDragAndDropPayload(payload_type);
  if (payload.has_value()) {
    MaterialHandle handle;
    if (proto::ParseMessage(*payload, &handle)) {
      std::string url = std::string(handle.GetUrl());
      if (!url.empty()) {
        return DragAndDropMaterialResult(url, registry.GetMaterial(url));
      }
    }
  }

  return std::nullopt;
}

}  // namespace imp::editor
