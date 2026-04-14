/*
 * Copyright 2025 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_GLTF_BOUNDS_H_
#define THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_GLTF_BOUNDS_H_

// #include <scene/Node.h>

#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "core/common/paired_vector.h"
#include "core/math/math.h"
#include "core/model/model_data.h"
#include "core/model/skeleton_data.h"
#include "core/ncsb/component.h"
#include "core/ncsb/dispatcher/event.h"
#include "core/ncsb/node_handle.h"

namespace svxr {

// Keeps track of the bounds of a GltfRenderer component, updating when
// necessary, and possibly using a subset of the GltfRenderer's bounds if bounds
// overrides have been specified.
class GltfBounds : public imp::Component {
 public:
  struct BoundsUpdated : public imp::Event {};

  void Setup();
  const imp::Box& GetLocalBounds() const;

 private:
  void UpdateBounds();

  // List of nodes that contribute to bounds calculations.
  std::vector<imp::NodeHandle> bounds_nodes_;
  imp::Box bounds_;
};
}  // namespace svxr
#endif  // THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_GLTF_BOUNDS_H_
