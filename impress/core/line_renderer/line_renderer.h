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

#ifndef THIRD_PARTY_IMPRESS_CORE_LINE_RENDERER_LINE_RENDERER_H_
#define THIRD_PARTY_IMPRESS_CORE_LINE_RENDERER_LINE_RENDERER_H_

#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "core/line_renderer/line_renderer.proto.imp.h"
#include "imp.h"

namespace imp {

// Component that generates a line mesh and adds a MeshRenderer + material.
class LineRenderer : public imp::Component {
 public:
  Future<absl::Status> Setup();
  void OnIsfStateChanged();

  // Sets the color of the line material.
  void SetColor(const float4& color);

 private:
  LineRendererState state_;

 public:
  using IsfInfo = imp::IsfInfo<&LineRenderer::state_>;
  static constexpr bool kRunInEditMode = true;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_LINE_RENDERER_LINE_RENDERER_H_
