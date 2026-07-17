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

#include "core/split_engine/materials/builtin/gsplat/gsplat_material_helpers.h"

#include "core/common/log.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "core/resources/resource_definition.h"
#include "core/split_engine/materials/builtin/gsplat/gsplat_material_assets.h"
#include "core/split_engine/materials/builtin/gsplat/magic_window_panel_material_assets.h"
#include "core/view/base_view.h"
#include "core/window/filament_host.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

resources::ResourceDefinition GsplatDefaultRenderResource(
    BaseView& view, android_xr::schemas::GsplatMode material_mode) {
  // In multiview mode, materials must accept texture arrays.
  bool is_multiview =
      view.GetHost()->GetEngine()->getConfig().stereoscopicType ==
      filament::backend::StereoscopicType::MULTIVIEW;

#if defined(IMP_INCLUDE_STEREO_VARIANT_BY_DEFAULT) && \
    IMP_INCLUDE_STEREO_VARIANT_BY_DEFAULT
  switch (material_mode) {
    case android_xr::schemas::GsplatMode::UNSPECIFIED:
    case android_xr::schemas::GsplatMode::GSPLAT:
      return is_multiview ? kBuiltinGsplatStereoMatCmat
                          : kBuiltinGsplatMonoMatCmat;
    case android_xr::schemas::GsplatMode::MAGIC_WINDOW:
      return is_multiview ? kBuiltinMagicWindowPanelStereoMatCmat
                          : kBuiltinMagicWindowPanelMonoMatCmat;
  }
#else
  if (is_multiview) {
    IMP_LOG(imp::DFATAL) << "Stereo variant is not included in this build.";
  }
  switch (material_mode) {
    case android_xr::schemas::GsplatMode::UNSPECIFIED:
    case android_xr::schemas::GsplatMode::GSPLAT:
      return kBuiltinGsplatMonoMatCmat;
    case android_xr::schemas::GsplatMode::MAGIC_WINDOW:
      return kBuiltinMagicWindowPanelMonoMatCmat;
  }
#endif
}

}  // namespace imp::split_engine
