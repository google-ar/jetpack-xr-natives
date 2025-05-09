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

#ifndef THIRD_PARTY_IMPRESS_CORE_EFFECTS_SCREEN_FADER_SCREEN_FADER_H_
#define THIRD_PARTY_IMPRESS_CORE_EFFECTS_SCREEN_FADER_SCREEN_FADER_H_

#include "absl/status/status.h"
#include "core/async/future.h"
#include "core/common/smooth.h"
#include "core/ncsb/component.h"
#include "core/view/framework/render/mesh_renderer.h"
#include "core/view/utils/frame_time.h"

namespace imp {

// Fades the entire impress view in/out to a color over time.
class ScreenFader : public Component {
 public:
  enum class State {
    // The impress view is currently completely faded in.
    kFadedIn,
    // The impress view is currently in the process of fading in over time.
    kFadingIn,
    // The impress view is currently in the process of fading out over time.
    kFadingOut,
    // The impress view is currently fully faded out.
    kFadedOut
  };

  // Configures the settings for the screen fader.
  struct Params {
    // The color that the view should fade out to.
    float3 color;
    // The time it takes to fully fade.
    float duration_seconds;
  };

  // Event sent to the node this Component is attached to when the state of the
  // fader changes.
  struct StateChangedEvent : public Event {
    StateChangedEvent() {}
    StateChangedEvent(State state, State previous_state,
                      ComponentHandle<ScreenFader> fader)
        : state(state), previous_state(previous_state), fader(fader) {}
    State state;
    State previous_state;
    ComponentHandle<ScreenFader> fader;
  };

  Future<absl::Status> Setup(Params params);

  void Update(const FrameTime& frame_time);

  // Fades the screen out to the color over the specified amount of time.
  // If currently fading in, will interrupt the fade in and start fading out.
  void FadeOut();

  // Fades the screen back in over the specified amount of time.
  // If currently fading out, will interrupt the fade out and start fading in.
  void FadeIn();

  // Returns the current state of the fader.
  State GetState() const;

 private:
  void SetState(State state);

  ComponentHandle<MeshRenderer> mesh_renderer_;
  Smooth<float> alpha_;
  State state_ = State::kFadedIn;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_EFFECTS_SCREEN_FADER_SCREEN_FADER_H_
