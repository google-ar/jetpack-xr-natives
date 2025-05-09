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

#include "core/recipes/recipe_runner.h"

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "core/common/registry.h"
#include "core/common/robin_set.h"
#include "core/config.h"
#include "core/input/pointer_event.h"
#include "core/ncsb/node_handle.h"
#include "core/recipes/language/recipe_graph.proto.imp.h"
#include "core/recipes/language/recipe_runtime_event.h"
#include "core/recipes/language/recipe_runtime_graph.h"
#include "core/recipes/language/recipe_scope.h"
#include "core/recipes/language/recipe_system.h"
#include "core/recipes/language/recipe_utils.h"
#include "core/recipes/recipe_event.h"
#include "core/recipes/recipe_runner_state.proto.imp.h"
#include "core/view/framework/gestures/tap_gesture.h"
#include "core/view/utils/frame_time.h"
#include "mediapipe/framework/port/status_macros.h"

#if IMP_RUNTIME(DEV)
#include "core/editor/editor_info.h"
#endif

namespace imp {

using AsyncExecutionHandle = RecipeRuntimeGraph::AsyncExecutionHandle;
using ExecutionResult = RecipeRuntimeGraph::ExecutionResult;
using RuntimeState = RecipeRunner::RuntimeState;

namespace {

NodeHandle GetTapTarget(
    const TapGesture::TapEvent& tap_event,
    const std::optional<RobinSet<NodeHandle>>& tap_targets) {
  NodeHandle tap_target;

  for (NodeHandle node : tap_event.all_intersecting_nodes) {
    // all_intersecting_nodes are already sorted by distance. We can just
    // grab the first one.
    if (!tap_targets) {
      // If tap_targets is not set, we trigger the OnTapEvent with the
      // closest node.
      tap_target = node;
      break;
    }

    NodeHandle current_node = node;
    // Check if the node is one of the tap targets.
    // If not, go up the node hierarchy until we find one.
    while (current_node) {
      if (tap_targets->contains(current_node)) {
        tap_target = current_node;
        break;
      }
      current_node = current_node->GetParent();
    }

    if (tap_target) {
      break;
    }
  }

  return tap_target;
}

}  // namespace

absl::Status RecipeRunner::Setup() {
  MP_ASSIGN_OR_RETURN(runtime_graph_, RecipeRuntimeGraph::CreateRuntimeGraph(
                                       GetView(), state_.graph));

  // Defaults to true if `start_on_load` is not set.
  if (state_.start_on_load.value_or(true)) {
    // Schedules the Start() call to the next frame.
    runtime_state_ = RuntimeState::kReady;
  }

  return absl::OkStatus();
}

void RecipeRunner::Update(const FrameTime& frame_time) {
#if IMP_RUNTIME(DEV)
  // Since RecipeRunner runs in Draft/Edit mode, we need to handle stopping
  // updates here.
  if (editor::ShouldNotUpdate(GetView().GetRegistry())) {
    return;
  }
#endif

  if (runtime_state_ == RuntimeState::kReady) {
    absl::Status start_status = Start();
    if (!start_status.ok()) {
      IMP_LOG(imp::ERROR) << "Failed to start RecipeRunner: " << start_status.message();
      return;
    }
  }

  elapsed_time_ += frame_time.GetDeltaTime();

  if (runtime_state_ != RuntimeState::kRunning) {
    return;
  }

  // Pending events from the previous round of execution are now executed.
  std::vector<RecipeRuntimeEvent> pending_queue;
  std::swap(pending_queue, runtime_event_queue_);
  for (const RecipeRuntimeEvent& event : pending_queue) {
    TriggerEventAndHandleExecutionResult(event);
    // Events are also sent to the Dispatcher.
    GetView().GetDispatcher().Send(RecipeEvent(event.name, event.arguments));
  }

  RecipeRuntimeEvent on_update_event{
      .name = std::string(recipe::kOnUpdateEventName)};
  on_update_event.arguments[std::string(recipe::kDeltaSecondsSocketName)] =
      frame_time.GetDeltaSeconds();
  on_update_event.arguments[std::string(recipe::kElapsedSecondsSocketName)] =
      absl::ToDoubleSeconds(elapsed_time_);

  TriggerEventAndHandleExecutionResult(on_update_event);

  auto it = scheduled_executions_.begin();
  while (it != scheduled_executions_.end()) {
    ScheduledExecution& scheduled_execution = *it;
    TryResumeScheduledExecution(scheduled_execution);
    if (scheduled_execution.async_execution_handles.empty()) {
      it = scheduled_executions_.erase(it);
    } else {
      ++it;
    }
  }
}
void RecipeRunner::TriggerEventAndHandleExecutionResult(
    const RecipeRuntimeEvent& event) {
  std::unique_ptr<RecipeScope> scope =
      std::make_unique<RecipeScope>(scope_.get());
  absl::StatusOr<ExecutionResult> execution_result =
      runtime_graph_->TriggerEvent(event, scope.get());

  if (!execution_result.ok()) {
    IMP_LOG(imp::ERROR) << "RecipeEvent " << event.name
               << " exeuction failed: " << execution_result.status();
    return;
  }

  output::Recipe("RecipeEvent %s execution succeeded.", event.name);

  if (!execution_result->async_execution_handles.empty()) {
    ScheduledExecution scheduled_execution;
    // Gather info from execution result.
    for (const AsyncExecutionHandle& handle :
         execution_result->async_execution_handles) {
      scheduled_execution.async_execution_handles.push_back(handle);
    }

    scheduled_execution.scope = std::move(scope);
    scheduled_executions_.push_back(std::move(scheduled_execution));
  }
}

void RecipeRunner::TryResumeScheduledExecution(
    ScheduledExecution& scheduled_execution) {
  std::vector<AsyncExecutionHandle> new_handles;
  auto it = scheduled_execution.async_execution_handles.begin();
  while (it != scheduled_execution.async_execution_handles.end()) {
    const AsyncExecutionHandle& handle = *it;
    if (handle.Ready()) {
      absl::StatusOr<ExecutionResult> execution_result =
          runtime_graph_->ResumeExecution(handle,
                                          scheduled_execution.scope.get());
      if (execution_result.ok()) {
        output::Recipe("Async execution succeed.");
        for (const AsyncExecutionHandle& handle :
             execution_result->async_execution_handles) {
          new_handles.push_back(handle);
        }
      } else {
        IMP_LOG(imp::ERROR) << "Async execution failed: " << execution_result.status();
      }

      it = scheduled_execution.async_execution_handles.erase(it);
    } else {
      ++it;
    }
  }

  for (const AsyncExecutionHandle& new_handle : new_handles) {
    scheduled_execution.async_execution_handles.push_back(new_handle);
  }
}

absl::Status RecipeRunner::Start() {
  if (runtime_state_ == RuntimeState::kRunning) {
    return absl::OkStatus();
  }

  RecipeSystem& recipe_system =
      GetView().GetRegistry().GetOrCreate<RecipeSystem>(GetView());

  // Creates the local RecipeScope.
  scope_ = std::make_unique<RecipeScope>(&recipe_system.GetRootScope());
  // Declare default variables.
  VariableDeclaration node_self_variable_declaration{
      .name = std::string(recipe::kNodeSelfVariableName),
      .type = VariableDeclaration::Type::NODE,
      .init_value = Literal{.value = GetNode()},
  };

  MP_RETURN_IF_ERROR(scope_->DeclareVariable(node_self_variable_declaration));

  for (VariableDeclaration& variable_declaration :
       state_.graph.member_declarations) {
    MP_RETURN_IF_ERROR(scope_->DeclareVariable(variable_declaration));
  }

  // Clears the event queue.
  runtime_event_queue_.clear();

  // Starts listening to custom events in recipe graph.
  runtime_graph_->SetRuntimeEventListener([this](RecipeRuntimeEvent event) {
    runtime_event_queue_.push_back(std::move(event));
  });

  // Resets the elapsed time.
  elapsed_time_ = absl::ZeroDuration();

  // Starts listening to tap events.
  tap_event_connection_ =
      Connect([this](const TapGesture::TapEvent& tap_event) {
        if (tap_event.type != PointerEventType::kUp) {
          return;
        }

        // TODO: Add support for sending tap events to all valid
        // tap targets.
        NodeHandle tap_target = GetTapTarget(tap_event, tap_targets_);

        if (!tap_target) {
          output::Recipe(
              "Failed to generate OnTapEvent. No valid tap target found.");
          return;
        }

        RecipeRuntimeEvent on_tap_event{
            .name = std::string(recipe::kOnTapEventName)};
        on_tap_event.arguments[std::string(recipe::kTapTargetSocketName)] =
            tap_target;
        on_tap_event.arguments[std::string(recipe::kTapPositionSocketName)] =
            tap_event.position;

        runtime_event_queue_.push_back(std::move(on_tap_event));
      });

  // Triggers the OnStartEvent.
  RecipeRuntimeEvent on_start_event{.name =
                                        std::string(recipe::kOnStartEventName)};
  TriggerEventAndHandleExecutionResult(on_start_event);

  runtime_state_ = RuntimeState::kRunning;

  return absl::OkStatus();
}

void RecipeRunner::Stop() {
  if (runtime_state_ == RuntimeState::kStopped) {
    return;
  }

  // Disconnects event listeners.
  tap_event_connection_.Disconnect();
  runtime_graph_->SetRuntimeEventListener([](RecipeRuntimeEvent event) {});

  runtime_state_ = RuntimeState::kStopped;
}

void RecipeRunner::SetTapTargets(absl::Span<NodeHandle> tap_targets) {
  if (!tap_targets_) {
    tap_targets_ = RobinSet<NodeHandle>();
  } else {
    tap_targets_->clear();
  }

  for (NodeHandle node : tap_targets) {
    tap_targets_->insert(node);
  }
}

std::optional<const std::vector<NodeHandle>> RecipeRunner::GetTapTargets()
    const {
  if (tap_targets_) {
    return std::vector<NodeHandle>(tap_targets_->begin(), tap_targets_->end());
  }
  return std::nullopt;
}

}  // namespace imp
