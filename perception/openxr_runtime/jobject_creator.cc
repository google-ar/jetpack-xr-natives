// Copyright 2026 Google LLC
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

#include "openxr_runtime/jobject_creator.h"

#include <jni.h>
#include <openxr/openxr.h>

#include <string>

#include "common/namespace_util.h"

namespace androidx::xr::openxr {

using ::androidx::xr::common::GetJxrClass;
using ::androidx::xr::common::Package;

jobject CreateJavaDisplayBlendMode(JNIEnv* env,
                                   XrEnvironmentBlendMode blend_mode) {
  jclass blend_mode_class =
      GetJxrClass(env, Package::PACKAGE_CORE_INTERFACES, "DisplayBlendMode");
  const char* blend_mode_name;
  switch (blend_mode) {
    case XR_ENVIRONMENT_BLEND_MODE_OPAQUE:
      blend_mode_name = "NO_DISPLAY";
      break;
    case XR_ENVIRONMENT_BLEND_MODE_ADDITIVE:
      blend_mode_name = "ADDITIVE";
      break;
    case XR_ENVIRONMENT_BLEND_MODE_ALPHA_BLEND:
      blend_mode_name = "ALPHA_BLEND";
      break;
    default:
      return nullptr;
  }
  jfieldID blend_mode_field =
      env->GetStaticFieldID(blend_mode_class,
        blend_mode_name,
        "Landroidx/xr/runtime/interfaces/DisplayBlendMode;");
  if (blend_mode_field == nullptr) {
    env->ExceptionClear();
    return nullptr;
  }
  return env->GetStaticObjectField(blend_mode_class, blend_mode_field);
}

}  // namespace androidx::xr::openxr
