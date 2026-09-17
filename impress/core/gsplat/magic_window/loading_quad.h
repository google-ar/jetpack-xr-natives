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

#ifndef THIRD_PARTY_IMPRESS_CORE_GSPLAT_MAGIC_WINDOW_LOADING_QUAD_H_
#define THIRD_PARTY_IMPRESS_CORE_GSPLAT_MAGIC_WINDOW_LOADING_QUAD_H_

#include "core/material_library/material_param_value.h"
#include "core/math/vec.h"
#include "core/ncsb/component.h"
#include "core/render/mesh_renderer.h"
#include "imp.h"

namespace imp {

class LoadingQuad : public imp::Component {
 public:
  void Setup(imp::float2 size, float corner_radius);
  void Cleanup();
  void Fade(float start_opacity, float end_opacity);
  void Update(const imp::FrameTime& frame_time);
  float GetMaxOpacity() const;

 private:
  void OnMaterialCreated(imp::GenericMaterialPtr material, imp::float2 size,
                         float corner_radius);

  imp::GenericMaterialPtr material_;
  imp::ComponentHandle<imp::MeshRenderer> renderer_;

  bool is_animating_ = false;
  float current_opacity_ = 0.0f;
  float fade_duration_seconds_;
  float fade_timer_seconds_ = 0.0f;
  float fade_start_opacity_ = 0.0f;
  float fade_end_opacity_ = 0.0f;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_GSPLAT_MAGIC_WINDOW_LOADING_QUAD_H_
