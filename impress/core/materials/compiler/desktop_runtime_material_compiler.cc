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
#include "core/materials/compiler/desktop_runtime_material_compiler.h"

#include <sys/socket.h>

#include <memory>

#include "core/common/log.h"
#include "absl/memory/memory.h"
#include "core/materials/compiler/material_compiler_client.h"
#include "core/materials/compiler/material_compiler_service.h"
#include "core/materials/compiler/runtime_material_compiler.h"
#include "core/view/base_view.h"

namespace imp {

std::unique_ptr<RuntimeMaterialCompiler> DesktopRuntimeMaterialCompiler::Create(
    BaseView& view) {
  int fds[2];
  if (socketpair(AF_UNIX, SOCK_STREAM, 0, fds) != 0) {
    IMP_LOG(imp::FATAL) << "Failed to create socket pair";
  }

  return absl::WrapUnique(new DesktopRuntimeMaterialCompiler(
      view, std::make_unique<MaterialCompilerService>(fds[0]),
      std::make_unique<MaterialCompilerClient>(fds[1])));
}

}  // namespace imp
