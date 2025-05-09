// Copyright 2024 Google LLC
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

#include "core/common/filament_engine_helpers.h"

#include <filament/RenderableManager.h>

#include "absl/memory/memory.h"
#include "core/common/buffer_access.h"
#include "core/common/optional_error.h"
#include "core/common/platform_helpers.h"
#include "core/common/resource_helpers.h"
#include "filament/filament/include/filament/Fence.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {

void SynchronizePendingFrames(filament::Engine* engine) {
  using filament::Fence;
  Fence* fence = engine->createFence();
  fence->wait(Fence::Mode::FLUSH, Fence::FENCE_WAIT_FOR_EVER);
  engine->destroy(fence);
}

void FlushEngineAndWait(filament::Engine* engine) {
#if !defined(__EMSCRIPTEN__) && !defined(FILAMENT_SINGLE_THREADED)
  // TODO intentionally called twice until next filament drop.
  engine->flushAndWait();
  engine->flushAndWait();
#else
  engine->execute();
#endif  // __EMSCRIPTEN__
}

absl::StatusOr<filament::Material*> LoadPackagedMaterial(
    filament::Engine* engine, absl::string_view path) {
  BufferAccess material_code;

  MP_RETURN_IF_ERROR(LoadPackagedFile(path, &material_code));

  return filament::Material::Builder()
      .package(material_code.Data(), material_code.Size())
      .build(*engine);
}

}  // namespace imp
