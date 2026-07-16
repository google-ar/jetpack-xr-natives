/*
 * Copyright 2026 Google LLC
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
#ifndef THIRD_PARTY_IMPRESS_CORE_MATERIALS_COMPILER_MATERIAL_COMPILER_CONFIG_H_
#define THIRD_PARTY_IMPRESS_CORE_MATERIALS_COMPILER_MATERIAL_COMPILER_CONFIG_H_

#include "core/materials/compiler/schemas/material_compiler_ipc_generated.h"

namespace imp {

// Simple configuration struct for requesting material compilation.
struct MaterialCompilerConfig {
  schemas::Platform platform = schemas::Platform::All;
  schemas::TargetApi target_api = schemas::TargetApi::ANY;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MATERIALS_COMPILER_MATERIAL_COMPILER_CONFIG_H_
