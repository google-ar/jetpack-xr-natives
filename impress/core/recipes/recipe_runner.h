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

#ifndef THIRD_PARTY_IMPRESS_CORE_RECIPES_RECIPE_RUNNER_H_
#define THIRD_PARTY_IMPRESS_CORE_RECIPES_RECIPE_RUNNER_H_

#include <memory>
#include <optional>
#include <vector>

#include "absl/status/status.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "core/common/robin_set.h"
#include "core/math/vec.h"
#include "core/ncsb/component.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node_handle.h"
#include "core/recipes/language/recipe_async_execution_manager.h"
#include "core/recipes/language/recipe_runtime_event.h"
#include "core/recipes/language/recipe_runtime_graph.h"
#include "core/recipes/language/recipe_scope.h"
#include "core/recipes/language/recipe_types.proto.imp.h"
#include "core/recipes/recipe_runner_state.proto.imp.h"
#include "core/view/framework/collision/ray_hit.h"
#include "core/view/utils/frame_time.h"

namespace imp {

// Runs "Recipe" scripts based on the lifecycle methods of a node.
//
// Recipes are an experimental protobuf based scripting language for Impress.
class RecipeRunner : public Component {
 public:
  enum class RuntimeState {
    // The RecipeRunner is stopped.
    kStopped,
    // The RecipeRunner is running.
    kRunning,
    // The RecipeRunner is ready to start in the next Update().
    kReady,
  };

  // Event sent to the impress scene node when Update() is called.
  struct RecipeUpdateEvent : public Event {
   public:
    RecipeUpdateEvent(absl::Duration frame_time) : frame_time(frame_time) {}
    absl::Duration frame_time;
  };

  absl::Status Setup();

  void Update(const FrameTime& frame_time);

  // Starts the execution of RecipeGraph.
  // This will trigger OnStartEvent and starts listening to events.
  // All local variables will be reset after this call.
  absl::Status Start();

  // Stops the execution of RecipeGraph.
  // This will disconnect event listeners and stop listening to events.
  void Stop();

  // Returns the RecipeScope.
  RecipeScope& GetScope() { return *scope_; }

  const RecipeRunnerState& GetState() const { return state_; }

  RuntimeState GetRuntimeState() const { return runtime_state_; }

  // Specifies a list of tap targets.
  //
  // OnTapEvents will only be sent to the specified tap targets.
  // Please note that by default, all nodes are tap targets until the tap
  // targets are specified.
  // TODO: Add support for resetting tap targets.
  void SetTapTargets(absl::Span<NodeHandle> tap_targets);

  void SetHoverTargets(absl::Span<NodeHandle> hover_targets);

  // Returns the tap targets.
  //
  // If the tap targets are not set, std::nullopt will be returned.
  std::optional<const std::vector<NodeHandle>> GetTapTargets() const;

  // Returns the hover targets.
  //
  // If the hover targets are not set, std::nullopt will be returned.
  std::optional<const std::vector<NodeHandle>> GetHoverTargets() const;

  RecipeRuntimeGraph& GetRuntimeGraph() { return *runtime_graph_; }

  std::optional<absl::Duration> GetMaxExecutionTime() const {
    return max_execution_time_;
  }

  // Note: no value for execution_time means infinite execution time
  void SetMaxExecutionTime(std::optional<absl::Duration> execution_time) {
    max_execution_time_ = execution_time;
  }

 private:
  void TriggerEventAndHandleExecutionResult(
      const RecipeRuntimeEvent& event,
      std::optional<absl::Time> execution_cutoff_time);

  // Calculate the absolute cutoff time based on max_execution_time_
  std::optional<absl::Time> CalculateCutoffTime();

  // Handles a tap event by pushing an OnTapEvent to the runtime event queue.
  void HandleTap(RecipeRayHit tap_ray_hit, float2 tap_position);
  // Handles a hover event by pushing an OnHoverBeginEvent or OnHoverEndEvent to
  // the runtime event queue.
  void HandleHover(NodeHandle hover_target);

  RecipeRunnerState state_;
  RuntimeState runtime_state_ = RecipeRunner::RuntimeState::kStopped;

  std::optional<RobinSet<NodeHandle>> tap_targets_;
  std::optional<RobinSet<NodeHandle>> hover_targets_;

  std::unique_ptr<RecipeRuntimeGraph> runtime_graph_;
  std::unique_ptr<RecipeScope> scope_;
  std::vector<RecipeRuntimeEvent> runtime_event_queue_;
  std::optional<absl::Duration> elapsed_time_;

  Dispatcher::ScopedConnection tap_event_connection_;

  Dispatcher::ScopedConnection hover_event_connection_;

  Dispatcher::ScopedConnection split_engine_input_event_connection_;

  NodeHandle hovered_node_;

  RecipeAsyncExecutionManager async_execution_manager_;

  // Used to throttle the amount of execution time per update and start events
  std::optional<absl::Duration> max_execution_time_;

 public:
  using IsfInfo = IsfInfo<&RecipeRunner::state_, IsfDependencies<GltfRenderer>>;
  // This makes sure that RecipeRunner is set up, so that it can be edited in
  // the editor's "Node Editor" panel.
  static constexpr bool kRunInEditMode = true;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RECIPES_RECIPE_RUNNER_H_
