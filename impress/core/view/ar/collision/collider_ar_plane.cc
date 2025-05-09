// Copyright 2024 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "core/view/ar/collision/collider_ar_plane.h"

#include "core/ar/ar_hit_result.h"
#include "core/view/framework/collision/ray_hit.h"

namespace imp {
ColliderArPlane::ColliderArPlane()
    : BaseArComponent<ar::ArPlane>(CollisionMask::kColliderArPlane) {}

}  // namespace imp
