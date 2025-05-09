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

#ifndef THIRD_PARTY_IMPRESS_CORE_MODEL_JOINT_DATA_H_
#define THIRD_PARTY_IMPRESS_CORE_MODEL_JOINT_DATA_H_

#include <cstddef>
#include <cstdint>

#include "core/common/paired_vector.h"
#include "core/common/typed_id.h"
#include "core/common/typed_set_vector.h"
#include "core/common/typed_tree.h"
#include "core/model/skeleton_data.h"

namespace imp::model {

struct JointData;
using JointId = TypedId<JointData, uint16_t>;
using WeakJointId = TypedId<JointData, int32_t>;
using JointParentId = TypedParentId<JointData, uint16_t>;
using JointChildId = TypedDescendantId<JointData, uint16_t>;
template <typename T>
using JointLookup = PairedVector<T, JointData>;

// Skeletons and Animations support a much larger number of bones than can be
// practically supported by skinning systems.  Filament supports up to 256
// bones per entity; SampledJointId represents a filament-output index, and
// the primary type SampledJointData provides the source bone that is sampled.
struct SampledJointData {
  explicit SampledJointData(JointId in_joint) : joint(in_joint) {}
  JointId joint;
};
using SampledJointId = TypedId<SampledJointData, uint8_t>;
using WeakSampledJointId = TypedId<SampledJointData, int16_t>;
template <typename T>
using SampledJointLookup = PairedVector<T, SampledJointId::ReferredType>;

// Evaluating the skinning matrices for an entity requires convolving up
// through the skeleton, even over bones that aren't sampled directly (imagine
// a hand skin on an articulated skeleton).  JointData (referred to by
// JointId) is held in a TypedSetVector which describes the subset of the
// full skeleton required to compute the (often smaller) set of SampledJoints.
struct JointData {
  using ArrayType = StructureOfArrays<uint16_t, JointParentId, JointChildId,
                                      JointChildId, BoneId, WeakSampledJointId>;
  // Declare an enum to access fields (e.g. the iterator type in
  // utils::StructureOfArrays uses a tuple style get<>() API).
  enum Fields {
    kNumChildren,
    kParent,
    kFirstChild,
    kNextSibling,
    kSource,
    kTarget,
  };
  struct Proxy {
    template <size_t E>
    using Field = ArrayType::Field<E>;
    // Use the array's 'Field' type to create field accessors.
    union {
      // All union members have an identical storage type.
      Field<kNumChildren> num_children;
      Field<kParent> parent;
      Field<kFirstChild> first_child;
      Field<kNextSibling> next_sibling;
      Field<kSource> source;
      Field<kTarget> target;
    };
  };
};

}  // namespace imp::model

#endif  // THIRD_PARTY_IMPRESS_CORE_MODEL_JOINT_DATA_H_
