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

#ifndef THIRD_PARTY_IMPRESS_CORE_MODEL_SKELETON_DATA_H_
#define THIRD_PARTY_IMPRESS_CORE_MODEL_SKELETON_DATA_H_

#include <cstdint>
#include <string>

#include "core/common/hash.h"
#include "core/common/paired_vector.h"
#include "core/common/robin_map.h"
#include "core/common/typed_id.h"
#include "core/common/typed_set_vector.h"
#include "core/common/typed_tree.h"
#include "core/math/math.h"

namespace imp::model {

// Forward declared to allow the structure to contain Ids to itself.
struct BoneData;
// The ID type uses 16 bits, to allow for up to 65k bones.
using BoneId = TypedId<BoneData, uint16_t>;
using WeakBoneId = TypedId<BoneData, int32_t>;
using BoneParentId = TypedParentId<BoneData, uint16_t>;
using BoneChildId = TypedDescendantId<BoneData, uint16_t>;
template <typename T>
using BoneLookup = PairedVector<T, BoneData>;

// Define the proxy type used to represent an indexed structure.
struct BoneData {
  // Declare the SoA type (with field types for each field).
  using ArrayType =
      StructureOfArrays<uint16_t, BoneParentId, BoneChildId, BoneChildId,
                        PreciseTransform, mat4, std::string, uint16_t>;
  // Declare an enum to access fields (e.g. the iterator type in
  // utils::StructureOfArrays uses a tuple style get<>() API).
  enum Fields {
    kNumChildren,
    kParent,
    kFirstChild,
    kNextSibling,
    kLocalTransform,
    kRootTransform,
    kName,
    kNodeIndex,
  };
  struct Proxy {
    // Use the array's 'Field' type to create field accessors.
    template <size_t E>
    using Field = ArrayType::Field<E>;
    union {
      // All union members have an identical storage type.
      Field<kNumChildren> num_children;
      Field<kParent> parent;
      Field<kFirstChild> first_child;
      Field<kNextSibling> next_sibling;
      Field<kLocalTransform> local_transform;
      Field<kRootTransform> root_transform;
      Field<kName> name;
      Field<kNodeIndex> node_index;
    };
  };
};

struct SkeletonData {
  TypedSetVector<BoneData> bones;
  BoneLookup<HashValue> hashes;
  RobinMap<HashValue, BoneId> first_bone_from_hash;
  // Optional because generally there are no hash collisions or duplicate names.
  absl::optional<BoneLookup<BoneChildId>> next_bone_from_hash;
};

}  // namespace imp::model
#endif  // THIRD_PARTY_IMPRESS_CORE_MODEL_SKELETON_DATA_H_s
