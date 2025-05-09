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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_RECIPES_LINEAR_NODE_PLACEMENTS_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_RECIPES_LINEAR_NODE_PLACEMENTS_H_

#include "core/common/robin_map.h"
#include "core/editor/widgets/recipes/node_placements.h"
#include "core/editor/widgets/recipes/recipe_editor_graph.h"
#include "core/math/vec.h"

namespace imp::editor::recipe_internal {

// Arranges the nodes linearly
class LinearNodePlacements : public NodePlacements {
 public:
  LinearNodePlacements() = default;

  RobinMap<int, float2> GeneratePlacements(
      const RecipeEditorGraph& recipe_editor_graph) override;
};

}  // namespace imp::editor::recipe_internal

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_RECIPES_LINEAR_NODE_PLACEMENTS_H_
