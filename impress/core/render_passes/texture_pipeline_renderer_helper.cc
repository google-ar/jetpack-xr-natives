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

#include "core/render_passes/texture_pipeline_renderer_helper.h"

#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "core/view/base_view.h"
#include "core/view/utils/proto/view_config.proto.imp.h"

namespace imp {
int GetFrameDelayForTPR(BaseView& view) {
  const int kFrameDelayWithoutFence = 3;

  if (filament::backend::StereoscopicType::MULTIVIEW !=
      view.GetHost()->GetEngine()->getConfig().stereoscopicType) {
    // No delay needed if not multiview.
    return 0;
  }
  const ExperimentalFeatureFlags& feature_flags =
      view.GetConfig().experimental_feature_flags.Value();
  if (feature_flags.enable_texture_pipeline_renderer_first_frame_fence) {
    // No delay if the flag is enabled.
    return 0;
  }
  return kFrameDelayWithoutFence;
}

}  // namespace imp
