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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_TRANSFORM_WIDGET_ASPECT_SCALE_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_TRANSFORM_WIDGET_ASPECT_SCALE_H_

#include <memory>
#include <vector>

#include "core/editor/command.h"
#include "core/editor/widgets/transform_widget_aspect.h"
#include "core/editor/widgets/transform_widget_aspect_state.proto.imp.h"
#include "core/math/vec.h"
#include "core/ncsb/isf_info.h"
#include "core/view/framework/assets/gltf_renderer.h"

namespace imp::editor {

// Component that handles the scale portion of the Transform Widget.
class TransformWidgetAspectScale : public TransformWidgetAspect {
 protected:
  AspectType GetAspectType() const override { return AspectType::kScale; }
  void UpdateWidgetTransform() override;
  void UpdateAspect(bool commit,
                    std::vector<std::unique_ptr<Command>>& commands) override;
  float3 GetAxis() const override;

 private:
  TransformWidgetAspectScaleState state_;

 public:
  using IsfInfo = imp::IsfInfo<&TransformWidgetAspectScale::state_,
                               imp::IsfDependencies<GltfRenderer>>;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_TRANSFORM_WIDGET_ASPECT_SCALE_H_
