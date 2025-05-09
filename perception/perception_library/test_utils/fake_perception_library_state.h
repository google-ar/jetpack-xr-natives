// Copyright 2024 Google LLC
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

#ifndef JETPACK_XR_NATIVES_PERCEPTION_LIBRARY_TEST_UTILS_FAKE_PERCEPTION_LIBRARY_STATE_H_
#define JETPACK_XR_NATIVES_PERCEPTION_LIBRARY_TEST_UTILS_FAKE_PERCEPTION_LIBRARY_STATE_H_

#include <jni.h>
#include <openxr/openxr.h>

#include <cstdint>
#include <vector>

#include "absl/base/no_destructor.h"
#include "absl/container/flat_hash_map.h"

namespace vr::realitycore {

class FakePerceptionLibraryState {
 public:
  static FakePerceptionLibraryState& GetFakePerceptionLibraryState() {
    static absl::NoDestructor<FakePerceptionLibraryState>
        kFakePerceptionLibraryState;
    return *kFakePerceptionLibraryState;
  }

  bool initialize_success_ = false;
  XrReferenceSpaceType reference_space_type_ = XR_REFERENCE_SPACE_TYPE_LOCAL;
  jobject anchor_binder_ = nullptr;
  XrSpace anchor_id_ = XR_NULL_HANDLE;
  std::vector<uint8_t> anchor_uuid_bytes_;
  XrAnchorPersistStateANDROID persist_state_;
  jobject current_head_pose_ = nullptr;
  jobject left_view_ = nullptr;
  jobject right_view_ = nullptr;
  absl::flat_hash_map<XrTrackableANDROID, XrTrackablePlaneANDROID> planes_;
};

}  // namespace vr::realitycore
#endif  // JETPACK_XR_NATIVES_PERCEPTION_LIBRARY_TEST_UTILS_FAKE_PERCEPTION_LIBRARY_STATE_H_
