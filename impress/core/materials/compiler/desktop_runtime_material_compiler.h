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
#ifndef THIRD_PARTY_IMPRESS_CORE_MATERIALS_COMPILER_DESKTOP_RUNTIME_MATERIAL_COMPILER_H_
#define THIRD_PARTY_IMPRESS_CORE_MATERIALS_COMPILER_DESKTOP_RUNTIME_MATERIAL_COMPILER_H_

#include <memory>
#include <utility>

#include "core/async/future.h"
#include "core/materials/compiler/cache/material_cache.h"
#include "core/materials/compiler/material_compiler_client.h"
#include "core/materials/compiler/runtime_material_compiler.h"
#include "core/view/base_view.h"

namespace imp {

// Runtime material compiler for desktop mode, which spawns a new process to
// compile materials.
class DesktopRuntimeMaterialCompiler : public RuntimeMaterialCompiler {
 public:
  static Future<std::unique_ptr<RuntimeMaterialCompiler>> Create(
      BaseView& view);
  ~DesktopRuntimeMaterialCompiler() override;

 private:
  DesktopRuntimeMaterialCompiler(
      BaseView& view,
      std::unique_ptr<MaterialCompilerClient> material_compiler_client,
      pid_t material_service_pid, std::unique_ptr<MaterialCache> cache)
      : RuntimeMaterialCompiler(view, std::move(material_compiler_client),
                                std::move(cache)),
        material_service_pid_(material_service_pid) {}

  pid_t material_service_pid_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MATERIALS_COMPILER_DESKTOP_RUNTIME_MATERIAL_COMPILER_H_
