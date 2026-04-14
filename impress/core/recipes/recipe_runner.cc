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
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "core/common/registry.h"
#include "core/common/robin_set.h"
#include "core/config.h"
#include "core/input/pointer_event.h"
#include "core/math/vec.h"
#include "core/ncsb/component.h"
#include "core/ncsb/node_handle.h"
#include "core/recipes/language/recipe_async_execution_manager.h"
#include "core/recipes/language/recipe_execution_context.h"
#include "core/recipes/language/recipe_graph.proto.imp.h"
#include "core/recipes/language/recipe_runtime_event.h"
#include "core/recipes/language/recipe_runtime_graph.h"
#include "core/recipes/language/recipe_scope.h"
#include "core/recipes/language/recipe_system.h"
#include "core/recipes/language/recipe_types.proto.imp.h"
#include "core/recipes/language/recipe_utils.h"
#include "core/recipes/recipe_event.h"
#include "core/recipes/recipe_runner_state.proto.imp.h"
#include "core/view/framework/collision/ray_hit.h"
#include "core/view/framework/gestures/hover_gesture.h"
#include "core/view/framework/gestures/tap_gesture.h"
#include "core/view/framework/input/pointer_input_handler.h"
#include "core/view/utils/frame_time.h"
#include "split_engine/input/split_engine_input_event.h"
#include "mediapipe/framework/port/status_macros.h"

#if IMP_RUNTIME(DEV)
#include "core/editor/editor_info.h"
#endif

namespace imp {

using AsyncExecution = RecipeAsyncExecutionManager::AsyncExecution;
using RuntimeState = RecipeRunner::RuntimeState;

absl::Status RecipeRunner::Setup() {
  MP_ASSIGN_OR_RETURN(runtime_graph_, RecipeRuntimeGraph::CreateRuntimeGraph(
                                       GetView(), state_.graph));

  // Defaults to true if `start_on_load` is not set.
  if (state_.start_on_load.value_or(true)) {
    // Schedules the Start() call to the next frame.
    runtime_state_ = RuntimeState::kReady;
  }

  RecipeSystem& recipe_system =
      GetView().GetRegistry().GetOrCreate<RecipeSystem>(GetView());

  // Creates the local RecipeScope.
  scope_ = std::make_unique<RecipeScope>(&recipe_system.GetRootScope());

  return absl::OkStatus();
}

std::optional<absl::Time> RecipeRunner::CalculateCutoffTime() {
  if (!max_execution_time_.has_value()) {
    return std::optional<absl::Time>();
  }

  return absl::Now() + max_execution_time_.value();
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

  if (runtime_state_ != RuntimeState::kRunning) {
    return;
  }

  float time_delta;
  if (elapsed_time_.has_value()) {
    *elapsed_time_ += frame_time.GetDeltaTime();
    time_delta = frame_time.GetDeltaSeconds();
  } else {
    time_delta = 0.f;
    elapsed_time_ = absl::ZeroDuration();
  }

  std::optional<absl::Time> execution_cutoff_time = CalculateCutoffTime();

  // Set the value for the RecipeRunner's `time_since_start` global recipe
  // variable for this frame.
  scope_->GetVariable(std::string(recipe::kTimeSinceStart))->get() =
      recipe::Variable((float)absl::ToDoubleSeconds(*elapsed_time_));

  // Pending events from the previous round of execution are now executed.
  std::vector<RecipeRuntimeEvent> pending_queue;
  std::swap(pending_queue, runtime_event_queue_);
  for (const RecipeRuntimeEvent& event : pending_queue) {
    TriggerEventAndHandleExecutionResult(event, execution_cutoff_time);
    // Events are also sent to the Dispatcher.
    GetView().GetDispatcher().Send(RecipeEvent(event.name, event.arguments));
  }

  // Send a custom update event to this node
  RecipeRunner::RecipeUpdateEvent recipe_update_event(
      frame_time.GetDeltaTime());
  Send(recipe_update_event);

  // Triggers the OnUpdateEvent if a `OnUpdateEvent` Recipe Event node exists in
  // the RecipeRuntimeGraph.
  if (runtime_graph_->HasEvent(recipe::kOnUpdateEventName)) {
    RecipeRuntimeEvent on_update_event{
        .name = std::string(recipe::kOnUpdateEventName)};
    on_update_event.arguments[std::string(recipe::kDeltaSecondsSocketName)] =
        time_delta;
    on_update_event.arguments[std::string(recipe::kElapsedSecondsSocketName)] =
        absl::ToDoubleSeconds(*elapsed_time_);

    TriggerEventAndHandleExecutionResult(on_update_event,
                                         execution_cutoff_time);
  }

  // Consume all previously scheduled async executions that are ready
  async_execution_manager_.ForEachAsyncExecution(
      [this, execution_cutoff_time](
          const RecipeAsyncExecutionManager::AsyncExecution& execution) {
        if (!execution.handle.Ready()) {
          return;
        }
        RecipeRuntimeGraph::ExecutionResult result =
            runtime_graph_->ResumeExecution(execution, GetView(),
                                            execution_cutoff_time);

        if (!result.ok() && result.code() != absl::StatusCode::kCancelled) {
          IMP_LOG(imp::ERROR) << "Async execution failed: " << result;

          if (result.code() == absl::StatusCode::kResourceExhausted) {
            // Stops the recipe runner if the async execution fails due to
            // resource exhaustion.
            Stop();
          }
        }
      });

  async_execution_manager_.DeleteFinishedScopes();
}

void RecipeRunner::TriggerEventAndHandleExecutionResult(
    const RecipeRuntimeEvent& event,
    std::optional<absl::Time> execution_cutoff_time) {
  // Create a new scope based on the base member scope, and the async execution
  // manager is its sole owner
  // TODO Improve the management of RecipeScope, currently the
  // scope passed to TriggerEvent has to match the active scope in
  // RecipeAsyncExecutionManager
  RecipeScope* scope = async_execution_manager_.AddScope(scope_.get());

  RecipeExecutionContext context =
      RecipeExecutionContext{.scope = *scope,
                             .view = GetView(),
                             .async_manager = async_execution_manager_,
                             .execution_cutoff_time = execution_cutoff_time};

  RecipeRuntimeGraph::ExecutionResult result =
      runtime_graph_->TriggerEvent(event, context);

  if (!result.ok()) {
    IMP_LOG(imp::ERROR) << "RecipeEvent " << event.name
               << " execution failed: " << result;
    if (result.code() == absl::StatusCode::kResourceExhausted) {
      // Stops the recipe runner if the event execution fails due to resource
      // exhaustion.
      Stop();
    }

    return;
  }

  output::Recipe("RecipeEvent %s execution succeeded.", event.name);
}

absl::Status RecipeRunner::Start() {
  if (runtime_state_ == RuntimeState::kRunning) {
    return absl::OkStatus();
  }

  // Clears all variables created in a previous invocation.
  scope_->ClearLocalVariables();

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
  elapsed_time_ = std::nullopt;

  // Initializes the `time_since_start` variable to zero
  VariableDeclaration time_since_start_declaration{
      .name = std::string(recipe::kTimeSinceStart),
      .type = VariableDeclaration::Type::FLOAT,
      .init_value = Literal{.value = 0.f},
  };

  MP_RETURN_IF_ERROR(scope_->DeclareVariable(time_since_start_declaration));

  // Starts listening to tap events.
  tap_event_connection_ =
      Connect([this](const TapGesture::TapEvent& tap_event) {
        if (!tap_targets_.has_value() ||
            tap_event.type != PointerEventType::kUp ||
            !tap_event.ray_hits.has_value()) {
          return;
        }

        RayHit tap_ray_hit;
        for (const RayHit& ray_hit : *tap_event.ray_hits) {
          if (tap_targets_->contains(ray_hit.node)) {
            tap_ray_hit = ray_hit;
            break;
          }
        }

        if (!tap_ray_hit.node.IsValid()) {
          output::Recipe(
              "Failed to generate OnTapEvent. No valid tap target found.");
          return;
        }

        RecipeRayHit recipe_ray_hit{
            .distance = tap_ray_hit.distance,
            .node = tap_ray_hit.node,
            .world_point = tap_ray_hit.world_point,
            .world_orientation = tap_ray_hit.world_orientation,
            .world_normal = tap_ray_hit.world_normal};

        HandleTap(recipe_ray_hit, tap_event.position);
      });

  hover_event_connection_ =
      Connect([this](const HoverGesture::HoverEvent& hover_event) {
        if (!hover_targets_.has_value()) {
          return;
        }

        NodeHandle hover_target;
        for (const NodeHandle& hit_node : hover_event.all_intersecting_nodes) {
          if (hover_targets_->contains(hit_node)) {
            hover_target = hit_node;
            break;
          }
        }

        HandleHover(hover_target);
      });

  split_engine_input_event_connection_ =
      Connect([this](const android_xr::SplitEngineInputEvent& event) {
        switch (event.action) {
          case android_xr::SplitEngineInputEvent::Action::ACTION_DOWN: {
            RecipeRayHit recipe_ray_hit{
                .node = event.hit_node->target,
                .world_point = event.hit_node->hit_position,
            };
            HandleTap(recipe_ray_hit, float2{});
            break;
          }
          case android_xr::SplitEngineInputEvent::Action::ACTION_HOVER_ENTER:
            HandleHover(event.hit_node->target);
            break;
          case android_xr::SplitEngineInputEvent::Action::ACTION_HOVER_EXIT:
            HandleHover(NodeHandle());
            break;
          default:
            break;
        }
      });

  // Triggers the OnStartEvent if a `OnStartEvent` Recipe Event node exists in
  // the RecipeRuntimeGraph.
  if (runtime_graph_->HasEvent(recipe::kOnStartEventName)) {
    RecipeRuntimeEvent on_start_event{
        .name = std::string(recipe::kOnStartEventName)};
    TriggerEventAndHandleExecutionResult(on_start_event, CalculateCutoffTime());
  }

  runtime_state_ = RuntimeState::kRunning;

  return absl::OkStatus();
}

void RecipeRunner::Stop() {
  if (runtime_state_ == RuntimeState::kStopped) {
    return;
  }

  // Disconnects event listeners.
  tap_event_connection_.Disconnect();
  hover_event_connection_.Disconnect();
  runtime_graph_->SetRuntimeEventListener([](RecipeRuntimeEvent event) {});

  // Resets the async execution manager and cancels all async executions in
  // flight.
  async_execution_manager_.Reset();

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

void RecipeRunner::SetHoverTargets(absl::Span<NodeHandle> hover_targets) {
  if (!hover_targets_) {
    hover_targets_ = RobinSet<NodeHandle>();
  } else {
    hover_targets_->clear();
  }

  for (NodeHandle node : hover_targets) {
    hover_targets_->insert(node);
  }
}

std::optional<const std::vector<NodeHandle>> RecipeRunner::GetTapTargets()
    const {
  if (tap_targets_) {
    return std::vector<NodeHandle>(tap_targets_->begin(), tap_targets_->end());
  }
  return std::nullopt;
}

std::optional<const std::vector<NodeHandle>> RecipeRunner::GetHoverTargets()
    const {
  if (hover_targets_) {
    return std::vector<NodeHandle>(hover_targets_->begin(),
                                   hover_targets_->end());
  }
  return std::nullopt;
}

void RecipeRunner::HandleTap(RecipeRayHit tap_ray_hit, float2 tap_position) {
  if (!runtime_graph_->HasEvent(recipe::kOnTapEventName)) {
    return;
  }

  RecipeRuntimeEvent on_tap_event{.name = std::string(recipe::kOnTapEventName)};
  on_tap_event.arguments[std::string(recipe::kTapTargetSocketName)] =
      tap_ray_hit.node;
  on_tap_event.arguments[std::string(recipe::kControllerIndexSocketName)] = 0;
  on_tap_event.arguments[std::string(recipe::kTapPositionSocketName)] =
      tap_position;
  on_tap_event.arguments[std::string(recipe::kTapRayHitSocketName)] =
      tap_ray_hit;

  runtime_event_queue_.push_back(std::move(on_tap_event));
}

void RecipeRunner::HandleHover(NodeHandle hover_target) {
  if (hover_target == hovered_node_) {
    return;
  }

  if (hovered_node_.IsValid() &&
      runtime_graph_->HasEvent(recipe::kOnHoverEndEventName)) {
    RecipeRuntimeEvent on_hover_end_event{
        .name = std::string(recipe::kOnHoverEndEventName)};
    on_hover_end_event.arguments[std::string(recipe::kHoverTargetSocketName)] =
        hovered_node_;
    on_hover_end_event
        .arguments[std::string(recipe::kControllerIndexSocketName)] = 0;

    runtime_event_queue_.push_back(std::move(on_hover_end_event));
  }

  if (hover_target.IsValid() &&
      runtime_graph_->HasEvent(recipe::kOnHoverBeginEventName)) {
    RecipeRuntimeEvent on_hover_begin_event{
        .name = std::string(recipe::kOnHoverBeginEventName)};
    on_hover_begin_event
        .arguments[std::string(recipe::kHoverTargetSocketName)] = hover_target;
    on_hover_begin_event
        .arguments[std::string(recipe::kControllerIndexSocketName)] = 0;

    runtime_event_queue_.push_back(std::move(on_hover_begin_event));
  }

  hovered_node_ = hover_target;
}

}  // namespace imp
