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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_COLLISION_COLLIDER_AR_PLANE_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_COLLISION_COLLIDER_AR_PLANE_H_

#include "core/view/ar/base_ar_component.h"

namespace imp {

// Defines an ArPlane in an Impress scene.
class ColliderArPlane : public BaseArComponent<ar::ArPlane> {
 public:
  ColliderArPlane();
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_COLLISION_COLLIDER_AR_PLANE_H_
