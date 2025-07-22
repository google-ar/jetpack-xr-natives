// Copyright 2025 Google LLC
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

#include "split_engine/split_engine_subspace_manager_impl.h"

#include <cstdint>
#include <string>
#include <tuple>
#include <utility>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/async/future_common.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "split_engine/input/split_engine_input_event.h"
#include "split_engine/subspace_events.h"
#include "split_engine/subspace_root.h"

namespace android_xr {

SplitEngineSubspaceManagerImpl::SplitEngineSubspaceManagerImpl(
    imp::BaseView& view)
    : view_(view), foreground_executor_(imp::Executor::ForegroundExecutor()) {}

void SplitEngineSubspaceManagerImpl::RegisterSubspace(
    uint32_t subspace_id, uint32_t existing_root_entity_id) {
  imp::Future<absl::Status>::Schedule(
      [this, subspace_id, existing_root_entity_id]() {
        imp::NodeHandle node;
        if (existing_root_entity_id != 0) {
          utils::Entity entity = utils::Entity::import(existing_root_entity_id);
          node = imp::NodeHandle(entity);
          if (!node.IsValid()) {
            LOG(FATAL) << "Attempt to create subspace with invalid existing "
                          "root entity id: "
                       << existing_root_entity_id;
          }
        } else {
          node = view_.CreateNode();
        }
        subspace_map_.emplace(std::piecewise_construct,
                              std::forward_as_tuple(subspace_id),
                              std::forward_as_tuple(view_, node));
        if (view_.GetSplitEngineSerializer()) {
          view_.GetSplitEngineSerializer()->AssignUserId(node.GetEntity(),
                                                         subspace_id);
        }
        return absl::OkStatus();
      },
      {.executor = foreground_executor_})
      .KeptBy(this);
}

void SplitEngineSubspaceManagerImpl::CreateSubspace(
    uint32_t subspace_id, std::string app_name) {
  imp::Future<absl::Status>::Schedule(
      [this, subspace_id, app_name]() {
        auto it = subspace_map_.find(subspace_id);
        if (it == subspace_map_.end()) {
          LOG(ERROR) << "Subspace not found";
          return absl::NotFoundError("Subspace not found");
        }
        auto event = OnSubspaceCreatedEvent{app_name, it->second, subspace_id};
        view_.GetDispatcher().Send(event);
        return absl::OkStatus();
      },
      {.executor = foreground_executor_})
      .KeptBy(this);
}

void SplitEngineSubspaceManagerImpl::DestroySubspace(
    uint32_t subspace_id) {
    imp::Future<absl::Status>::Schedule(
       [this, subspace_id]() {
        auto it = subspace_map_.find(subspace_id);
        if (it == subspace_map_.end()) {
          LOG(ERROR) << "Subspace not found";
          return absl::NotFoundError("Subspace not found");
        }
        auto event = OnSubspaceDestroyedEvent{subspace_id};
        view_.GetDispatcher().Send(event);

        subspace_map_.erase(it);
        return absl::OkStatus();
      },
      {.executor = foreground_executor_,
       // If on the frame thread, execute immediately. Otherwise, schedule on
       // the frame thread. We need to do this because in the case that the app
       // is currently being torn down, we must ensure all resources created
       // on subspace creation are destroyed before the app is torn down.
       .executor_mode =
        imp::FutureExecutorMode::kScheduleIfNotOnExecutorThread})
      .KeptBy(this);
}

// Call only from frame thread.
absl::Status SplitEngineSubspaceManagerImpl::ForwardInputEvent(
    uint32_t subspace_id, android_xr::SplitEngineInputEvent& input_event) {
  if (subspace_map_.count(subspace_id) == 0) {
    return absl::NotFoundError(
        absl::StrFormat("Subspace with %d not found", subspace_id));
  }

  bool hit_node_is_valid = input_event.hit_node && input_event.hit_node->target;
  bool secondary_hit_node_is_valid =
      input_event.secondary_hit_node && input_event.secondary_hit_node->target;

  imp::mat4f world_from_subspace;
  imp::NodeHandle subspace_root_node;
  auto it = subspace_map_.find(subspace_id);
  if (it != subspace_map_.end()) {
    subspace_root_node = it->second.GetNode();
    if (subspace_root_node.IsValid()) {
      world_from_subspace = it->second.GetWorldFromSubspaceTransform();
    }
  }

  if (!hit_node_is_valid && !secondary_hit_node_is_valid) {
    if (subspace_root_node.IsValid()) {
      subspace_root_node->Send(input_event);
    } else {
      LOG(ERROR) << "Subspace not found or no hit node is valid";
      return absl::FailedPreconditionError(
          "Subspace not found or no hit node is valid");
    }
  }
  if (hit_node_is_valid) {
    if (subspace_root_node.IsValid()) {
      input_event.hit_node->world_hit_position =
          (world_from_subspace * input_event.hit_node->hit_position).xyz;
    }
    input_event.hit_node->target->Send(input_event);
  }
  if (secondary_hit_node_is_valid) {
    if (subspace_root_node.IsValid()) {
      input_event.secondary_hit_node->world_hit_position =
          (world_from_subspace * input_event.secondary_hit_node->hit_position)
              .xyz;
    }
    input_event.secondary_hit_node->target->Send(input_event);
  }
  return absl::OkStatus();
}

void SplitEngineSubspaceManagerImpl::ForwardSubspaceTransform(
    uint32_t subspace_id, const imp::mat4f& subspace_transform) {
  imp::Future<absl::Status>::Schedule(
      [this, subspace_id, subspace_transform]() {
        auto it = subspace_map_.find(subspace_id);
        if (it == subspace_map_.end()) {
          LOG(ERROR) << "Subspace not found";
          return absl::NotFoundError("Subspace not found");
        }
        if (!it->second.GetNode().IsValid()) {
          LOG(ERROR) << "Subspace is not valid";
          return absl::NotFoundError("Subspace is not valid");
        }
        return it->second.UpdateSubspaceTransform(subspace_transform);
      },
      {.executor = foreground_executor_})
      .KeptBy(this);
}

void SplitEngineSubspaceManagerImpl::UpdateSubspaceAnchor(
    uint32_t subspace_id, SubspaceRoot::AnchorType anchor_type) {
  imp::Future<absl::Status>::Schedule(
      [this, subspace_id, anchor_type]() {
        auto it = subspace_map_.find(subspace_id);
        if (it == subspace_map_.end()) {
          LOG(ERROR) << "Subspace not found";
          return absl::NotFoundError("Subspace not found");
        }
        return it->second.UpdateAnchor(anchor_type);
      },
      {.executor = foreground_executor_})
      .KeptBy(this);
}

}  // namespace android_xr
