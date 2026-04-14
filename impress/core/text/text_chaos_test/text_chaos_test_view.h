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

#ifndef THIRD_PARTY_IMPRESS_CORE_TEXT_TEXT_CHAOS_TEST_TEXT_CHAOS_TEST_VIEW_H_
#define THIRD_PARTY_IMPRESS_CORE_TEXT_TEXT_CHAOS_TEST_TEXT_CHAOS_TEST_VIEW_H_

#include <random>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/editor/widget.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "core/text/text_renderer_state.proto.imp.h"
#include "core/view/base_view.h"
#include "core/view/framework/gestures/drag_gesture.h"
#include "core/view/framework/view.h"
#include "core/view/utils/proto/view_config.proto.imp.h"
#include "samples/text_stress_test/text_stress_test_dictionaries.h"

namespace imp {

class TextChaosTestView : public View {
 public:
  void Setup() override;

 private:
  TextRendererState GenerateRandomState(std::mt19937* external_gen);
  Future<absl::Status> CreateTextNodeInternal(const TextRendererState& state,
                                              float3 pos);

  NodeHandle text_root_;

  ViewConfig view_config_;

  // Number of text nodes to create.
  int count_ = 100;
  // Min and max font size in pixels
  float min_size_ = 8.0f;
  float max_size_ = 32.0f;
  // Min and max stroke width in pixels
  float min_stroke_ = 2.0f;
  float max_stroke_ = 8.0f;

  Language language_ = Language::kEnglish;

  int seed_ = 42;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_TEXT_TEXT_CHAOS_TEST_TEXT_CHAOS_TEST_VIEW_H_
