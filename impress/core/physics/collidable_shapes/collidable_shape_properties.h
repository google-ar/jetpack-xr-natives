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

#ifndef THIRD_PARTY_IMPRESS_CORE_PHYSICS_COLLIDABLE_SHAPES_COLLIDABLE_SHAPE_PROPERTIES_H_
#define THIRD_PARTY_IMPRESS_CORE_PHYSICS_COLLIDABLE_SHAPES_COLLIDABLE_SHAPE_PROPERTIES_H_

#include "core/ncsb/component.h"
#include "core/ncsb/isf_info.h"
#include "core/physics/collidable_shapes/collidable_shape_properties_state.proto.imp.h"

namespace imp {

// This component can be used to parameterize collidable shapes creation.
// For example, it can be used to set the mass of a collidable shape, when that
// shape is used as part of a compound shape.
// In that case if the mass is not set, the collidable shape associated
// with that node will have a default mass of 1 kg.
class CollidableShapeProperties : public Component {
 public:
  const CollidableShapePropertiesState& GetState() const { return state_; }
  CollidableShapePropertiesState& EditState() { return state_; }

 private:
  CollidableShapePropertiesState state_;

 public:
  using IsfInfo = IsfInfo<&CollidableShapeProperties::state_>;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PHYSICS_COLLIDABLE_SHAPES_COLLIDABLE_SHAPE_PROPERTIES_H_
