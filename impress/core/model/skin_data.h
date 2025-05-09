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

#ifndef THIRD_PARTY_IMPRESS_CORE_MODEL_SKIN_DATA_H_
#define THIRD_PARTY_IMPRESS_CORE_MODEL_SKIN_DATA_H_

#include <cstddef>
#include <cstdint>

#include "filament/filament/include/filament/Box.h"
#include "core/common/bit_vector.h"
#include "core/common/paired_vector.h"
#include "core/common/typed_id.h"
#include "core/common/typed_set_vector.h"
#include "core/common/typed_vector.h"
#include "core/math/math.h"
#include "core/model/joint_data.h"
#include "core/model/shared_data.h"

namespace imp::model {

struct SkinData;
using SkinId = TypedId<SkinData, int16_t>;
template <typename T>
using SkinLookup = PairedVector<T, SkinData>;

// SkinnedEntityData defines an application of a skin to a particular entity.
// It tracks the target entity, the bounds of each sampled bone, and a
// bitvector describing which sampled bones are actually sampled by the
// geometry in the target entity.
struct SkinnedEntityData;
using SkinnedEntityId = TypedId<SkinnedEntityData, uint16_t>;
struct SkinnedEntityData {
  using ArrayType =
      StructureOfArrays<EntityId, SampledJointLookup<filament::Aabb>,
                        PairedBitVector<SampledJointData>>;
  enum Fields {
    kTarget,
    kSampledJointBounds,
    kSampledJointInUse,
  };
  struct Proxy {
    template <size_t E>
    using Field = ArrayType::Field<E>;
    // Use the array's 'Field' type to create field accessors.
    union {
      // All union members have an identical storage type.
      Field<kTarget> target;
      Field<kSampledJointBounds> sampled_joint_bounds;
      Field<kSampledJointInUse> sampled_joint_in_use;
    };
  };
};

// A SkinData defines the mapping from a posed skeleton to a set of shader
// constants on a set of entities.  AnimationTarget uses it to call
// RenderableManager::setBones().
struct SkinData {
  // The array of uploaded joints (i.e. a 'sampled_bone_index' retrieved from
  // a vertex actually samples sampled_joints[sampled_bone_index].joint).
  TypedVector<SampledJointData> sampled_joints;
  // Stores sampled_bind_bone_from_pose_root for each sampled bone.
  SampledJointLookup<filament::math::mat4f> inverse_bind_poses;
  // SampledJointLookup<filament::Aabb> bounds;
  TypedSetVector<JointData> joints;
  // Information about each entity which will have setBones called on it
  TypedSetVector<SkinnedEntityData> skinned_entities;
  // Per the glTF spec, the pose root isn't strictly necessary to perform
  // skinning and is often not specified; if it is, it can be considered a
  // local origin of the skeleton.
  WeakEntityId pose_root;
};

}  // namespace imp::model

#endif  // THIRD_PARTY_IMPRESS_CORE_MODEL_SKIN_DATA_H_
