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
#ifndef THIRD_PARTY_IMPRESS_CORE_MATERIALS_COMPILER_RUNTIME_MATERIAL_COMPILER_CREATOR_H_
#define THIRD_PARTY_IMPRESS_CORE_MATERIALS_COMPILER_RUNTIME_MATERIAL_COMPILER_CREATOR_H_

#include <memory>

#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/materials/compiler/runtime_material_compiler.h"
#include "core/view/base_view.h"

namespace imp {

class RuntimeMaterialCompilerCreator {
 public:
  // Creates a RuntimeMaterialCompiler based on the target platform.
  // Note that the native library override param is only relevant to Android. If
  // nothing is provided, it will use the default Impress so library.
  static Future<std::unique_ptr<RuntimeMaterialCompiler>> Create(
      BaseView& view, absl::string_view native_library_override = "");
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MATERIALS_COMPILER_RUNTIME_MATERIAL_COMPILER_CREATOR_H_
