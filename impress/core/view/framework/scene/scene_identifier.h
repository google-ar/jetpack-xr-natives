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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_SCENE_SCENE_IDENTIFIER_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_SCENE_SCENE_IDENTIFIER_H_

#include "core/ncsb/component.h"
#include "core/view/framework/scene/scene_system.h"

namespace imp {

// Component representing the unique id of a node created from .isf files.
//
// Added to any node in a .isf file that has a unique_id field assigned.
// See:
// third_party/impress/core/ncsb/node_data.proto
//
// Used to save the unique id back out when saving nodes into NodeData at
// runtime.
class SceneIdentifier : public Component {
 public:
  static constexpr bool kExcludeFromEditor = true;

  void Setup(SceneSystem::NodeUniqueId id);

  void SetId(SceneSystem::NodeUniqueId id);
  SceneSystem::NodeUniqueId GetId() const;

 private:
  SceneSystem::NodeUniqueId id_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_SCENE_SCENE_IDENTIFIER_H_
