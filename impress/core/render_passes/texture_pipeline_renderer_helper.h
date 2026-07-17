// Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_RENDER_PASSES_TEXTURE_PIPELINE_RENDERER_HELPER_H_
#define THIRD_PARTY_IMPRESS_CORE_RENDER_PASSES_TEXTURE_PIPELINE_RENDERER_HELPER_H_

#include "core/view/base_view.h"

namespace imp {

// Returns the number of frames the TPR should be delayed on initialization.
//
// The TPR has a race condition.  If the TPR is used before the first frame is
// displayed the graphics driver can crash.
// The solution is a fence we've added to the TPR.
// The fence is disabled by default until we are sure it works with all clients.
// If the fence is enabled, this will return 0.
// Remove this function once the fence is enabled by default.
// TODO: (broken link) - Remove this function once the fence is enabled by
// default.
int GetFrameDelayForTPR(BaseView& view);

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RENDER_PASSES_TEXTURE_PIPELINE_RENDERER_HELPER_H_
