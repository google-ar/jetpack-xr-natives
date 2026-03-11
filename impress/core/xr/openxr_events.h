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

#ifndef THIRD_PARTY_IMPRESS_CORE_XR_OPENXR_EVENTS_H_
#define THIRD_PARTY_IMPRESS_CORE_XR_OPENXR_EVENTS_H_

#include <cstdint>

#include "core/math/transform.h"
#include "core/ncsb/dispatcher/event.h"
#include "core/view/platforms/xr_android/openxr_includes.h"

namespace imp {
// Event sent after a successful xrBeginSession.  Calls such as
// xrCreateActionSpace and xrAttachSessionActionSets must happen after this
// event.
struct OpenXrSessionBeginEvent : public Event {};
struct OpenXrReferenceSpaceCreatedEvent : public Event {};
struct OpenXrInteractionProfileChangedEvent : public Event {};
// Event sent when a reference space is pending a change. Note that this event
// is only available for reference spaces that are explicitly created.
struct OpenXrSpaceChangePendingEvent : public Event {
  // Default constructor for backward compatibility; some Impress clients
  // manually create this event to fake space changes.
  OpenXrSpaceChangePendingEvent() {};
  OpenXrSpaceChangePendingEvent(XrReferenceSpaceType reference_space_type,
                                XrTime change_time, XrBool32 pose_valid,
                                XrPosef pose_in_previous_space)
      : reference_space_type(reference_space_type),
        change_time(change_time),
        pose_valid(pose_valid),
        pose_in_previous_space(pose_in_previous_space) {}
  XrReferenceSpaceType reference_space_type;
  XrTime change_time;
  XrBool32 pose_valid;
  XrPosef pose_in_previous_space;
};

// Event sent when a reference space change has occurred.
struct OpenXrSpaceChangedEvent : public Event {
  OpenXrSpaceChangedEvent(XrReferenceSpaceType reference_space_type,
                          XrBool32 pose_valid, XrPosef pose_in_previous_space)
      : reference_space_type(reference_space_type),
        pose_valid(pose_valid),
        pose_in_previous_space(pose_in_previous_space) {}
  XrReferenceSpaceType reference_space_type;
  XrBool32 pose_valid;
  XrPosef pose_in_previous_space;
};

// Event sent after a successful xrWaitFrame if the session is focused. Calls
// such as xrLocateSpace and xrGetActionState* should only be made while the
// app is focused.
struct OpenXrFocusedWaitFrameEvent : public Event {};

// Event sent after the visibility mask for a given view has changed.
struct OpenXrVisibilityMaskChangedEvent : public Event {
 public:
  OpenXrVisibilityMaskChangedEvent(uint32_t view_index)
      : view_index_(view_index) {}
  uint32_t getViewIndex() const { return view_index_; }

 private:
  uint32_t view_index_;
};

// Event sent for all OpenXr events we receive from the OpenXr runtime.
// Note that event_data reference will not outlive the lifetime of the event.
struct OpenXrGenericEvent : public Event {
 public:
  OpenXrGenericEvent(const XrEventDataBuffer& event_data)
      : event_data_(event_data) {}
  const XrEventDataBuffer& event_data_;
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_XR_OPENXR_EVENTS_H_
