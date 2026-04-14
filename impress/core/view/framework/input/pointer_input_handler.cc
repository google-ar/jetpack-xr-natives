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

#include "core/view/framework/input/pointer_input_handler.h"

#include <cstddef>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "core/common/log.h"
#include "absl/types/optional.h"
#include "absl/types/variant.h"
#include "core/input/input_manager.h"
#include "core/input/keyboard_event.h"
#include "core/input/pointer_event.h"
#include "core/input/wheel_event.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node_handle.h"
#include "core/view/framework/collision/collision_manager.h"
#include "core/view/framework/collision/ray_hit.h"

namespace imp {

size_t PointerHitEvent::GetPointerCount() const {
  return absl::visit([](const auto& hit_list) { return hit_list.size(); },
                     hits);
}

NodeHandle PointerHitEvent::GetHitNode(size_t index) const {
  return absl::visit(
      [index](const auto& hit_list) {
        if (index >= hit_list.size()) {
          IMP_LOG(imp::FATAL) << "Size of hits is " << hit_list.size()
                     << " but index accessed was " << index;
        }
        return hit_list[index].empty() ? NodeHandle()
                                       : hit_list[index].front().node;
      },
      hits);
}

std::vector<NodeHandle> PointerHitEvent::GetAllIntersectingNodes(
    size_t index) const {
  return absl::visit(
      [index](const auto& hit_list) {
        if (index >= hit_list.size()) {
          IMP_LOG(imp::FATAL) << "Size of hits is " << hit_list.size()
                     << " but index accessed was " << index;
        }
        std::vector<NodeHandle> hit_nodes(hit_list[index].size());
        absl::c_transform(hit_list[index], hit_nodes.data(),
                          [](auto& hit) { return hit.node; });
        return hit_nodes;
      },
      hits);
}

absl::optional<RayHit> PointerHitEvent::GetTruncatedRayHit(size_t index) const {
  if (index >= GetPointerCount()) {
    IMP_LOG(imp::FATAL) << "Size of hits is " << GetPointerCount()
               << " but index accessed was " << index;
  }

  if (absl::holds_alternative<std::vector<std::vector<RayHit>>>(hits)) {
    auto ray_hits = absl::get<std::vector<std::vector<RayHit>>>(hits)[index];
    if (!ray_hits.empty()) {
      return ray_hits.front();
    }
  } else {
    auto double_ray_hits =
        absl::get<std::vector<std::vector<DoubleRayHit>>>(hits)[index];
    if (!double_ray_hits.empty()) {
      return RayHit(double_ray_hits.front());
    }
  }
  return {};
}

absl::variant<absl::monostate, RayHit, DoubleRayHit>
PointerHitEvent::GetRayHitOrDoubleRayHit(size_t index) const {
  if (index >= GetPointerCount()) {
    IMP_LOG(imp::FATAL) << "Size of hits is " << GetPointerCount()
               << " but index accessed was " << index;
  }

  if (absl::holds_alternative<std::vector<std::vector<RayHit>>>(hits)) {
    const std::vector<RayHit>& ray_hits =
        absl::get<std::vector<std::vector<RayHit>>>(hits)[index];
    if (!ray_hits.empty()) {
      return ray_hits.front();
    }
  } else if (absl::holds_alternative<std::vector<std::vector<DoubleRayHit>>>(
                 hits)) {
    const std::vector<DoubleRayHit>& double_ray_hits =
        absl::get<std::vector<std::vector<DoubleRayHit>>>(hits)[index];
    if (!double_ray_hits.empty()) {
      return double_ray_hits.front();
    }
  }
  return {};
}
PointerInputHandler::PointerInputHandler(BaseView* view, Dispatcher* dispatcher)
    : view_(view), dispatcher_(*dispatcher) {}

PointerInputHandler::PointerInputHandler(BaseView* view)
    : view_(view), dispatcher_(view->GetDispatcher()) {}

PointerInputHandler::~PointerInputHandler() = default;

std::vector<RayHit> PointerInputHandler::IntersectPointer(const Pointer& p) {
  return view_->GetCollisionManager().IntersectAll(p.point);
}

std::vector<DoubleRayHit> PointerInputHandler::IntersectPointerPrecise(
    const Pointer& p) {
  return view_->GetCollisionManager().IntersectAllPrecise(p.point);
}

void PointerInputHandler::Update(InputManager* input_manager) {
  std::vector<PointerEvent> pointer_events = input_manager->PopPointerEvents();
  for (auto& event : pointer_events) {
    DispatchHitEvents(event);
  }

  std::vector<WheelEvent> wheel_events = input_manager->PopWheelEvents();
  for (auto& event : wheel_events) {
    dispatcher_.Send(WheelScrollEvent(event));
  }

  std::vector<KeyboardEvent> keyboard_events =
      input_manager->PopKeyboardEvents();
  for (const KeyboardEvent& event : keyboard_events) {
    dispatcher_.Send(event);
  }

  // Defaults to clearing out keyboard input events.
  keyboard_events.clear();
}

void PointerInputHandler::DispatchHitEvents(const PointerEvent& event) {
  auto pointers = event.GetPointers();
  if (view_->IsPreciseTranslationEnabled()) {
    std::vector<std::vector<DoubleRayHit>> hits(pointers.size());
    absl::c_transform(pointers, hits.data(), [this](const Pointer& p) {
      return IntersectPointerPrecise(p);
    });
    dispatcher_.Send(PointerHitEvent(event, std::move(hits)));
  } else {
    std::vector<std::vector<RayHit>> hits(pointers.size());
    absl::c_transform(pointers, hits.data(),
                      [this](const Pointer& p) { return IntersectPointer(p); });
    dispatcher_.Send(PointerHitEvent(event, std::move(hits)));
  }
}

}  // namespace imp
