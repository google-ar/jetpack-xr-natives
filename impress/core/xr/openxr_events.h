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

#include "core/math/transform.h"
#include "core/ncsb/dispatcher/event.h"

namespace imp {
// Event sent after a successful xrBeginSession.  Calls such as
// xrCreateActionSpace and xrAttachSessionActionSets must happen after this
// event.
struct OpenXrSessionBeginEvent : public Event {};
struct OpenXrReferenceSpaceCreatedEvent : public Event {};
struct OpenXrInteractionProfileChangedEvent : public Event {};
struct OpenXrSpaceChangePendingEvent : public Event {};

// Event sent after a successful xrWaitFrame if the session is focused. Calls
// such as xrLocateSpace and xrGetActionState* should only be made while the
// app is focused.
struct OpenXrFocusedWaitFrameEvent : public Event {};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_XR_OPENXR_EVENTS_H_
