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

#ifndef THIRD_PARTY_IMPRESS_EXTENSIONS_SKYBOX_SKYBOX_RENDERER_H_
#define THIRD_PARTY_IMPRESS_EXTENSIONS_SKYBOX_SKYBOX_RENDERER_H_

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/isf_info.h"
#include "core/render/texture.h"
#include "core/view/framework/render/mesh_renderer.h"
#include "core/view/utils/frame_time.h"

namespace imp {

// Displays the texture of the lighting environment in the background.
class SkyboxRenderer : public Component {
 public:
  static constexpr bool kRunInEditMode = true;

  Future<absl::Status> Setup();

  void Update(const FrameTime& frame_time);

 private:
  static constexpr absl::string_view kSkyboxTypeUrl = "SkyboxRenderer";

  ComponentHandle<MeshRenderer> renderer_;
  imp::Texture* reflections_texture_ = nullptr;

 public:
  using IsfInfo = imp::StatelessIsfInfo<SkyboxRenderer, kSkyboxTypeUrl>;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_EXTENSIONS_SKYBOX_SKYBOX_RENDERER_H_
