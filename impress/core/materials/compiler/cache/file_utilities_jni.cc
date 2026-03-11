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
#include "core/materials/compiler/cache/file_utilities_jni.h"

#include <jni.h>

#include <string>

#include "core/common/context.h"
#include "core/common/jni_helpers.h"

namespace imp {

JavaFileUtilities::JavaFileUtilities(const Context& context)
    : JavaWrapper(context, "com/google/ar/imp/materialcompiler/FileUtilities",
                  "(Landroid/content/Context;)V", context.GetActivityContext()),
      activity_context_(context.GetActivityContext()) {
  // Set up method handles.
  get_cache_directory_method_ =
      GetMethodHandle("getCacheDirectory", "()Ljava/lang/String;");
}

JavaFileUtilities::~JavaFileUtilities() = default;

std::string JavaFileUtilities::GetCacheDirectory() {
  JNIEnv* env = context_.GetJniEnv();
  JniUniquePtr<jstring> java_cache_dir =
      WrapJni(env, static_cast<jstring>(CallObjectMethod(
                       get_cache_directory_method_, activity_context_)));
  return GetString(env, java_cache_dir.get());
}

}  // namespace imp
