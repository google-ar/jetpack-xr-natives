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

#ifndef THIRD_PARTY_IMPRESS_CORE_AR_PLAYBACK_AR_PLAYBACK_DATA_H_
#define THIRD_PARTY_IMPRESS_CORE_AR_PLAYBACK_AR_PLAYBACK_DATA_H_

#include "core/ar/playback/playback_scene.pb.h"
#include "core/async/future.h"
#include "core/ncsb/component_handle.h"
#include "core/video/video_controller.h"

namespace imp::ar {

// Holds data required for an AR playback session.
struct ArPlaybackData {
  ArPlaybackScene playback_scene;
  Future<ComponentHandle<VideoController>> video_controller_future;
};

}  // namespace imp::ar
#endif  // THIRD_PARTY_IMPRESS_CORE_AR_PLAYBACK_AR_PLAYBACK_DATA_H_
