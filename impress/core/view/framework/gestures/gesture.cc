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

#include "core/view/framework/gestures/gesture.h"

#include <vector>

#include "core/common/log.h"

namespace imp {

Gesture::Gesture(Dispatcher* dispatcher, GesturePointerUtils* pointer_utils,
                 const PointerHitEvent& pointer_hit)
    : dispatcher_(dispatcher),
      pointer_utils_(pointer_utils),
      state_(State::kUnstarted),
      id_(kEmptyId) {
  target_node_ = pointer_hit.GetHitNode();
  all_intersecting_nodes_ = pointer_hit.GetAllIntersectingNodes();
}

Gesture::~Gesture() {}

void Gesture::InitializeId(Id id) {
  if (id_ != kEmptyId) {
    IMP_LOG(imp::FATAL)
        << "Unable to initialize gesture Id, it has already been initialized.";
  }

  id_ = id;
}

Dispatcher& Gesture::GetDispatcher() const { return *dispatcher_; }

GesturePointerUtils* Gesture::GetPointerUtils() const { return pointer_utils_; }

NodeHandle Gesture::GetTargetNode() const { return target_node_; }

const std::vector<NodeHandle>& Gesture::GetAllIntersectingNodes() const {
  return all_intersecting_nodes_;
}

Gesture::Id Gesture::GetId() const { return id_; }

bool Gesture::Started() const {
  return state_ != State::kUnstarted && state_ != State::kUnstartedCancelled;
}

bool Gesture::Finished() const {
  return state_ == State::kCancelFinished || state_ == State::kFinished;
}

bool Gesture::Cancelled() const {
  return state_ == State::kCancelFinished ||
         state_ == State::kUnstartedCancelled;
}

void Gesture::OnPointerHitEvent(const PointerHitEvent& pointer_hit) {
  if (state_ == State::kUnstarted) {
    Start(pointer_hit);
    return;
  } else if (state_ == State::kStarted) {
    OnUpdate(pointer_hit);
  }
}

void Gesture::Start(const PointerHitEvent& pointer_hit) {
  // Verify that the id has already been set at this point.
  if (id_ == kEmptyId) {
    IMP_LOG(imp::FATAL) << "Unable to start gesture, id has not been initialized.";
  }

  if (TryStart(pointer_hit)) {
    state_ = State::kStarted;
  }
}

void Gesture::Finish(const PointerHitEvent& pointer_hit) {
  state_ = State::kFinished;
  OnFinish(pointer_hit);
}

void Gesture::Cancel(const PointerHitEvent& pointer_hit) {
  if (state_ == State::kUnstarted) {
    state_ = State::kUnstartedCancelled;
  } else {
    state_ = State::kCancelFinished;
    OnFinish(pointer_hit);
  }
  OnCancel();
}

Gesture::CancelFn Gesture::MakeCancelFn(Gesture* gesture,
                                        const PointerHitEvent& hit) {
  return [gesture, &hit]() { gesture->Cancel(hit); };
}

}  // namespace imp
