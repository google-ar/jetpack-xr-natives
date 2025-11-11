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
#ifndef THIRD_PARTY_IMPRESS_CORE_MATERIALS_COMPILER_ANDROID_RUNTIME_MATERIAL_COMPILER_H_
#define THIRD_PARTY_IMPRESS_CORE_MATERIALS_COMPILER_ANDROID_RUNTIME_MATERIAL_COMPILER_H_

#include <memory>
#include <utility>

#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/materials/compiler/material_compiler_client.h"
#include "core/materials/compiler/material_compiler_client_jni.h"
#include "core/materials/compiler/runtime_material_compiler.h"
#include "core/view/base_view.h"

namespace imp {

inline constexpr absl::string_view kDefaultNativeLibraryOverride =
    "imp_view_jni";

// Compiles source material for Android.
// It internally creates the Java and C++ clients, and compiles materials in an
// isolated process.
class AndroidRuntimeMaterialCompiler : public RuntimeMaterialCompiler {
 public:
  static Future<std::unique_ptr<RuntimeMaterialCompiler>> Create(
      BaseView& view, absl::string_view native_library_override =
                          kDefaultNativeLibraryOverride);

 private:
  AndroidRuntimeMaterialCompiler(
      BaseView& view, std::unique_ptr<JavaMaterialCompilerClient> java_client,
      std::unique_ptr<MaterialCompilerClient> native_client)
      : RuntimeMaterialCompiler(view, std::move(native_client)),
        java_client_(std::move(java_client)) {}

  std::unique_ptr<JavaMaterialCompilerClient> java_client_;
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MATERIALS_COMPILER_ANDROID_RUNTIME_MATERIAL_COMPILER_H_
