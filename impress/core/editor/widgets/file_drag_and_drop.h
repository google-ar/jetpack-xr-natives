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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_FILE_DRAG_AND_DROP_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_FILE_DRAG_AND_DROP_H_

#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/common/rememberer.h"
#include "core/config.h"
#include "core/editor/ui/drag_and_drop.h"
#include "core/editor/widget.h"
#include "core/materials/compiler/runtime_material_compiler.h"
#include "core/view/base_view.h"

namespace imp::editor {

class FileDragAndDrop : public Widget, public Rememberer {
 public:
  explicit FileDragAndDrop(BaseView& view);
  // Do not use a name to avoid display.
  absl::string_view GetName() const override { return "##File Drag And Drop"; }
  void DrawImGui() override;

 private:
  Future<RuntimeMaterialCompiler*> GetOrCreateMaterialCompiler(BaseView& view);
#if IMP_PLATFORM(DESKTOP)
  void ProcessMatFile(std::string filename);
#endif  // IMP_PLATFORM(DESKTOP)

  BaseView& view_;
  std::optional<std::pair<DragAndDropType, std::string>>
      pending_drag_and_drop_payload_ = std::nullopt;
  std::optional<Future<RuntimeMaterialCompiler*>>
      runtime_material_compiler_future_;
  std::unique_ptr<RuntimeMaterialCompiler> runtime_material_compiler_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_FILE_DRAG_AND_DROP_H_
