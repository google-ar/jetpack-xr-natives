/*
 * Copyright 2025 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_VIEW_VIEW_UPDATE_PARAMS_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_VIEW_VIEW_UPDATE_PARAMS_H_

#include "core/math/quat.h"
#include "core/math/vec.h"

namespace android_xr {

struct Fov {
  float angle_left;
  float angle_right;
  float angle_up;
  float angle_down;
};

struct Pose {
  imp::float3 translation;
  imp::quatf rotation;
};

struct ViewProjection {
  Fov fov;
  Pose pose;
};

struct ViewUpdateParams {
  ViewProjection* left_eye;
  ViewProjection* right_eye;
};

}  // namespace android_xr

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_VIEW_VIEW_UPDATE_PARAMS_H_
