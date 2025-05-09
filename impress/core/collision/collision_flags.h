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

#ifndef THIRD_PARTY_IMPRESS_CORE_COLLISION_COLLISION_FLAGS_H_
#define THIRD_PARTY_IMPRESS_CORE_COLLISION_COLLISION_FLAGS_H_

#include <stdint.h>
namespace imp {
constexpr int kNumExistingCollisionMask = 10;
constexpr int kCollisionMaskCountMax = 32;

// Presets for collision flags of built in collision types.
// When testing for a collision a mask can be provided to include/exclude
// various collider groups. In addition, users can add bits to model colliders
// to categorize them for different interaction types. For instance excluding
// background or static geometry when interacting with some foreground or
// dynamic object.
enum class CollisionMask : uint32_t {
  // A mask where all 'groups' fail.
  kNone = 0,
  // The default bit for any box collider.
  kColliderBox = (1 << 0),
  // The default bit for any Ar Plane.
  kColliderArPlane = (1 << 1),
  // The default bit for any Ar Point.
  kColliderArPoint = (1 << 2),
  // Bit for hit test on vertical plane.
  kColliderArMagicalSurfacePoint = (1 << 7),
  // Bit for hit test on a sphere.
  kColliderSphere = (1 << 8),
  // Bit for hit test on text colliders.
  kTextCollider = (1 << 9),
  // The default bit for SpriteColliders
  kSpriteCollider = (1 << 10),
  // Optional user bits for categorizing components.
  kStatic = (1 << 3),
  kDynamic = (1 << 4),
  kForeground = (1 << 5),
  kBackground = (1 << 6),
  // Default masking bits used by hit detection.
  kDefault = kColliderBox | kColliderArPlane | kColliderSphere | kTextCollider |
             kSpriteCollider,
  // A mask where all 'groups' pass.
  kAll = static_cast<uint32_t>(~kNone),
};

// Collider visualization mode.
enum class VisualizationStyle {
  kNotSelected,
  kSelected,
  kSelectedDescendent,
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COLLISION_COLLISION_FLAGS_H_
