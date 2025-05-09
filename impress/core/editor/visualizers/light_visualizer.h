/*
 * Copyright 2024 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_VISUALIZERS_LIGHT_VISUALIZER_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_VISUALIZERS_LIGHT_VISUALIZER_H_

#include "absl/strings/string_view.h"
#include "core/common/robin_map.h"
#include "core/editor/visualizers/visualizer.h"
#include "core/view/framework/lighting/light_state.proto.imp.h"

namespace imp::editor {

// Controls the position, rotation and type of light visualizers
// and whether it is enabled.
class LightVisualizer : public Visualizer {
 public:
  void Setup(NodeHandle light) override;
  void Update(const FrameTime& frame_time) override;

 private:
  enum class LightModelType {
    kNone,
    kDirectionalLight,
    kPointLight,
    kSpotLight
  };
  RobinMap<LightState::Type, LightModelType> light_state_map_ = {
      {LightState::Type::SUN, LightModelType::kDirectionalLight},
      {LightState::Type::DIRECTIONAL, LightModelType::kDirectionalLight},
      {LightState::Type::POINT, LightModelType::kPointLight},
      {LightState::Type::FOCUSED_SPOT, LightModelType::kSpotLight},
      {LightState::Type::SPOT, LightModelType::kSpotLight}};
  RobinMap<absl::string_view, LightModelType> light_model_map_ = {
      {"directional_light", LightModelType::kDirectionalLight},
      {"point_light", LightModelType::kPointLight},
      {"spot_light", LightModelType::kSpotLight}};
  void UpdateVisualizer(LightState::Type visualizer);

  NodeHandle light_visualizer_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_VISUALIZERS_LIGHT_VISUALIZER_H_
