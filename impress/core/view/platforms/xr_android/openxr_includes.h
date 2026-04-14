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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_OPENXR_INCLUDES_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_OPENXR_INCLUDES_H_

#define XR_USE_PLATFORM_ANDROID 1

// IWYU pragma: begin_exports

// <jni.h> is required since openxr_platform.h uses jobject.
#include <jni.h>

#include "core/config.h"

#if IMP_PLATFORM(ANDROID)
// The order of these includes is critical and clang format wants to reorder.
// clang-format off
// XR_USE_ variables need to be defined before including openxr_platform. These
// specify which functions and variable types are to be included.
#if IMP_MATERIAL_API(VULKAN)
#define XR_USE_GRAPHICS_API_VULKAN 1
#include <vulkan/vulkan_core.h>
#include "filament/filament/backend/include/backend/platforms/VulkanPlatform.h"
#elif IMP_MATERIAL_API(OPENGL)
#define XR_USE_GRAPHICS_API_OPENGL_ES 1
#include <GLES3/gl31.h>
#include <GLES2/gl2ext.h>
#include "filament/filament/backend/include/backend/platforms/PlatformEGLAndroid.h"
#else
#error "Material API not specified"
#endif
// clang-format on

#else
#define XR_USE_GRAPHICS_API_OPENGL_ES 1
#include <EGL/egl.h>
#include <EGL/eglext.h>

#include "filament/filament/backend/include/backend/platforms/OpenGLPlatform.h"
#endif

// Now it is safe to include OpenXR headers.
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>
#if IMP_PLATFORM(ANDROID)
#include <openxr/public/xr_androidx_spatial_interaction.h>
#endif

// IWYU pragma: end_exports

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_OPENXR_INCLUDES_H_
