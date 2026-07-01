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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_UI_DRAG_AND_DROP_MATERIAL_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_UI_DRAG_AND_DROP_MATERIAL_H_

#include <optional>
#include <string>
#include <utility>

#include "core/materials/material.h"
#include "core/view/base_view.h"

namespace imp::editor {

struct DragAndDropMaterialResult {
  std::string url;
  BorrowedMaterialPtr material;

  DragAndDropMaterialResult(std::string u, BorrowedMaterialPtr m)
      : url(std::move(u)), material(m) {}
};

// Returns the DragAndDropMaterialResult from the current drag and drop payload
// if one exists.
std::optional<DragAndDropMaterialResult> AcceptDragAndDropPayloadMaterial(
    BaseView* view);

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_UI_DRAG_AND_DROP_MATERIAL_H_
