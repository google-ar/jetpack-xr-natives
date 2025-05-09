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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_COMPONENTS_GRID_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_COMPONENTS_GRID_H_

#include "absl/status/status.h"
#include "core/ncsb/component.h"
#include "core/view/framework/render/primitive_shape_renderer.h"

namespace imp::editor {

// Create a quad with a grid shader to easily view the model size.
class Grid : public Component {
 public:
  // Sets up the grid. Fails if the grid material cannot be loaded.
  Future<absl::Status> Setup();
  void ShowGrid(bool enabled);

  void Update(const FrameTime& frame_time);

 private:
  ComponentHandle<PrimitiveShapeRenderer> grid_renderer_;
  AssetPtr<imp::MaterialAsset> grid_material_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_COMPONENTS_GRID_H_
