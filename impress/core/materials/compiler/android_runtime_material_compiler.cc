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
#include "core/materials/compiler/android_runtime_material_compiler.h"

#include <memory>
#include <utility>

#include "absl/memory/memory.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/common/context.h"
#include "core/materials/compiler/material_compiler_client.h"
#include "core/materials/compiler/material_compiler_client_jni.h"
#include "core/materials/compiler/runtime_material_compiler.h"
#include "core/view/base_view.h"

namespace imp {

Future<std::unique_ptr<RuntimeMaterialCompiler>>
AndroidRuntimeMaterialCompiler::Create(
    BaseView& view, absl::string_view native_library_override) {
  const Context& context = view.GetContext();

  std::unique_ptr<JavaMaterialCompilerClient> java_material_compiler_client =
      std::make_unique<JavaMaterialCompilerClient>(context,
                                                   native_library_override);

  return java_material_compiler_client->StartService(context).Then(
      [&view,
       java_client = std::move(java_material_compiler_client)](int fd) mutable
          -> absl::StatusOr<std::unique_ptr<RuntimeMaterialCompiler>> {
        auto native_client = std::make_unique<MaterialCompilerClient>(fd);
        // Bind java and native clients.
        java_client->SetNativeClient(*native_client);

        return absl::WrapUnique(new AndroidRuntimeMaterialCompiler(
            view, std::move(java_client), std::move(native_client)));
      });
}

}  // namespace imp
