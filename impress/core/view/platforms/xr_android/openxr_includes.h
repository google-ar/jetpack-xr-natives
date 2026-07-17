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

// <jni.h> is required since openxr_platform.h uses jobject.
#include <jni.h>

#include "core/config.h"  // IWYU pragma: keep

// IWYU pragma: begin_exports

#if IMP_PLATFORM(ANDROID)
#define XR_USE_PLATFORM_ANDROID 1
#endif

#if IMP_MATERIAL_API(VULKAN)
#define XR_USE_GRAPHICS_API_VULKAN 1
#include <vulkan/vulkan_core.h>  // NOLINT

#include "filament/filament/backend/include/backend/platforms/VulkanPlatform.h"
#elif IMP_MATERIAL_API(OPENGL)
#define XR_USE_GRAPHICS_API_OPENGL_ES 1
#if IMP_PLATFORM(ANDROID)
#include <GLES2/gl2.h>     // NOLINT
#include <GLES2/gl2ext.h>  // NOLINT
#include <GLES3/gl31.h>    // NOLINT

#include "filament/filament/backend/include/backend/platforms/PlatformEGLAndroid.h"
#else
#include <EGL/egl.h>     // NOLINT
#include <EGL/eglext.h>  // NOLINT

#include "filament/filament/backend/include/backend/platforms/OpenGLPlatform.h"
#endif
#elif IMP_MATERIAL_API(METAL)
// Metal is not supported in this package, but we handle the case to avoid
// falling through to the #error or legacy blocks.
#else
#error "Material API not specified"
#endif

// Now it is safe to include OpenXR headers.
#include <openxr/openxr.h>           // NOLINT
#include <openxr/openxr_platform.h>  // NOLINT
#if IMP_PLATFORM(ANDROID)
#include <openxr/public/xr_android_global_passthrough_dimming.h>
#include <openxr/public/xr_androidx_spatial_interaction.h>
#endif

// IWYU pragma: end_exports

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_OPENXR_INCLUDES_H_
