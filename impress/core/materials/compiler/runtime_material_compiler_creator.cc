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
#include "core/materials/compiler/runtime_material_compiler_creator.h"

#include <memory>

#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/config.h"
#if IMP_PLATFORM(ANDROID)
#include "core/materials/compiler/android_runtime_material_compiler.h"
#endif
#include "core/materials/compiler/desktop_runtime_material_compiler.h"
#include "core/materials/compiler/runtime_material_compiler.h"
#include "core/view/base_view.h"

namespace imp {

Future<std::unique_ptr<RuntimeMaterialCompiler>>
RuntimeMaterialCompilerCreator::Create(
    BaseView& view, absl::string_view native_library_override) {
#if IMP_PLATFORM(ANDROID)
  if (!native_library_override.empty()) {
    return AndroidRuntimeMaterialCompiler::Create(view,
                                                  native_library_override);
  }
  return AndroidRuntimeMaterialCompiler::Create(view);
#endif
  return DesktopRuntimeMaterialCompiler::Create(view);
}

}  // namespace imp
