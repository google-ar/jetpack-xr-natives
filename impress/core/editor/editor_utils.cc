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

#include "core/editor/editor_utils.h"

#include <optional>

#include "core/collision/collision_helpers.h"
#include "core/editor/editor.h"

namespace imp::editor {

std::optional<float3> GetPointerIntersectionWithGroundPlane(BaseView& view,
                                                            float2 pointer) {
#if IMP_RUNTIME(DEV)
  Editor& editor = view.GetRegistry().Get<Editor>()->get();
  Ray ray = editor.GetCamera()->WorldRayFromPixelPoint(pointer);
  float3 hit;
  if (collision::PlaneIntersectsRay(Plane({0, 1, 0}, 0), ray, &hit) ==
      collision::Result::kDoesIntersect) {
    return hit;
  } else {
    return std::nullopt;
  }
#else
  return std::nullopt;
#endif
}

}  // namespace imp::editor
