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
#ifndef THIRD_PARTY_IMPRESS_CORE_MATERIALS_COMPILER_FILE_UTILITIES_JNI_H_
#define THIRD_PARTY_IMPRESS_CORE_MATERIALS_COMPILER_FILE_UTILITIES_JNI_H_

#include <jni.h>

#include <string>

#include "core/common/context.h"
#include "core/common/jni_helpers.h"

namespace imp {

// Wraps the Java side FileUtilities class.
// Provides methods to interact with the Android file system.
class JavaFileUtilities : public JavaWrapper {
 public:
  explicit JavaFileUtilities(const Context& context);
  ~JavaFileUtilities() override;

  // Returns the cache directory path on the Android file system.
  std::string GetCacheDirectory();

 private:
  jobject activity_context_;
  JniHandle get_cache_directory_method_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MATERIALS_COMPILER_FILE_UTILITIES_JNI_H_
