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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_GRAPHICS_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_GRAPHICS_HELPERS_H_

#include <jni.h>

#include "core/math/vec.h"

namespace imp::android {

// Encodes a float4 in an RGBA format where each element is between 0-1 into an
// android color int.
jint ToColorInt(float4 color);

}  // namespace imp::android

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_GRAPHICS_HELPERS_H_
