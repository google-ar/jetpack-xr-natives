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

#include "core/text/text_chaos_test/text_chaos_test_view.h"

#include <algorithm>
#include <cstddef>
#include <random>
#include <string>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/text/text_collider.h"
#include "core/text/text_renderer.h"
#include "core/text/text_renderer_state.proto.imp.h"
#include "core/view/base_view.h"
#include "core/view/framework/client_api.h"
#include "core/view/framework/gestures/drag_gesture.h"
#include "core/view/framework/scene/scene_system.h"
#include "core/view/utils/frame_time.h"
#include "core/view/utils/proto/view_config.proto.imp.h"
#include "samples/text_stress_test/text_stress_test_dictionaries.h"

namespace imp {

const bool kIsCreateViewAssigned =
    client_api::SetCreateViewFn([](absl::string_view identifier) {
      return View::Create<TextChaosTestView>(std::string(identifier));
    });

void TextChaosTestView::Setup() {
  // The ViewConfig is setup from TextChaosActivityTest
  view_config_ = GetConfig();

  GetSceneSystem().RegisterComponentIsfInfo<TextRenderer>();

  text_root_ = CreateNode();

  elapsed_time_ = 0.0f;
  gen_.seed(seed_);

  std::uniform_real_distribution<float> dist_x(-2.0f, 2.0f);
  std::uniform_real_distribution<float> dist_y(-3.0f, 3.0f);
  float fixed_z = -5.0f;

  for (int i = 0; i < min_count_; ++i) {
    float3 pos(dist_x(gen_), dist_y(gen_), fixed_z);
    CreateTextNodeInternal(GenerateRandomState(&gen_), pos).KeptBy(this);
  }
}

void TextChaosTestView::Update(const FrameTime& frame_time) {
  elapsed_time_ += frame_time.GetDeltaSeconds();
  if (elapsed_time_ >= test_duration_) {
    return;
  }

  // Randomly do nothing on some frames
  std::uniform_int_distribution<int> skip_dist(0, 100);
  if (skip_dist(gen_) < 10) {  // 10% chance to skip
    return;
  }

  std::uniform_real_distribution<float> dist_x(-2.0f, 2.0f);
  std::uniform_real_distribution<float> dist_y(-3.0f, 3.0f);
  float fixed_z = -5.0f;

  // Determine current target count with some fluctuation
  std::uniform_int_distribution<int> curr_target_dist(min_count_, max_count_);
  int curr_target = curr_target_dist(gen_);

  // Randomly destroy some nodes. Only start deleting properly after a few
  // seconds to build up some state.
  if (elapsed_time_ > 2.0f) {
    std::uniform_int_distribution<int> destroy_dist(0, 10);
    int num_destroy =
        destroy_dist(gen_) +
        std::max(0, static_cast<int>(text_nodes_.size()) - curr_target);
    for (int i = 0; i < num_destroy && text_nodes_.size() > min_count_; ++i) {
      std::uniform_int_distribution<size_t> node_dist(0,
                                                      text_nodes_.size() - 1);
      size_t idx = node_dist(gen_);
      DestroyNode(text_nodes_[idx]);
      text_nodes_[idx] = text_nodes_.back();
      text_nodes_.pop_back();
    }
  }

  // Randomly create some nodes
  std::uniform_int_distribution<int> create_dist(0, 15);
  // Create more aggressively if we're under the target count
  int num_create =
      create_dist(gen_) +
      std::max(0, curr_target - static_cast<int>(text_nodes_.size()));
  for (int i = 0; i < num_create && text_nodes_.size() < max_count_; ++i) {
    float3 pos(dist_x(gen_), dist_y(gen_), fixed_z);
    CreateTextNodeInternal(GenerateRandomState(&gen_), pos).KeptBy(this);
  }
}

Language TextChaosTestView::GetRandomLanguage(std::mt19937* external_gen) {
  std::uniform_int_distribution<int> lang_dist(0, 4);
  return static_cast<Language>(lang_dist(*external_gen));
}

TextRendererState TextChaosTestView::GenerateRandomState(
    std::mt19937* external_gen) {
  static std::mt19937 static_gen;
  std::mt19937& gen = external_gen ? *external_gen : static_gen;

  TextRendererState state;
  Language lang = GetRandomLanguage(&gen);
  const auto& dict = GetDictionary(lang);
  std::uniform_int_distribution<size_t> dist_noun(0, dict.size() - 1);
  std::uniform_real_distribution<float> dist_size(min_size_, max_size_);
  std::uniform_real_distribution<float> dist_stroke(min_stroke_, max_stroke_);
  std::uniform_real_distribution<float> dist_color(0.0f, 1.0f);

  state.text = dict[dist_noun(gen)];
  state.font_size_pixels = dist_size(gen);

  state.color = float4(dist_color(gen), dist_color(gen), dist_color(gen), 1.0f);
  state.stroke_width_pixels = dist_stroke(gen);
  state.stroke_color =
      float4(dist_color(gen), dist_color(gen), dist_color(gen), 1.0f);

  // Randomly use batch rendering
  std::uniform_int_distribution<int> batch_dist(0, 1);
  state.batch = batch_dist(gen) == 1;

  return state;
}

Future<absl::Status> TextChaosTestView::CreateTextNodeInternal(
    const TextRendererState& state, imp::float3 pos) {
  NodeHandle node = text_root_->CreateChildNode();
  node->SetLocalPosition(pos);
  text_nodes_.push_back(node);

  return node->AddComponentWithState<TextRenderer>(state).Then(
      [node](ComponentHandle<TextRenderer> _) {
        node->AddComponent<TextCollider>().IgnoreError();
        return absl::OkStatus();
      });
}

}  // namespace imp
