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

#ifndef THIRD_PARTY_IMPRESS_CORE_NCSB_BEHAVIOR_SYSTEM_H_
#define THIRD_PARTY_IMPRESS_CORE_NCSB_BEHAVIOR_SYSTEM_H_

#include <stdbool.h>
#include <sys/stat.h>

#include "core/common/enum_indexed_array.h"
#include "core/config.h"
#include "core/graph/dependency_graph.h"
#include "core/ncsb/dispatcher/event.h"
#include "core/ncsb/system.h"
#include "core/ncsb/update_id.h"
#include "core/ncsb/update_phase.h"
#include "core/ncsb/updater_traits.h"
#include "core/view/base_view.h"
#include "core/view/utils/frame_time.h"

namespace imp {

class BaseView;

class UpdateSystem : public System {
 public:
  // Base class for all Updaters. Not subclassed directly, the templated
  // Updater<T> is subclassed instead.
  struct BaseUpdater {
    virtual ~BaseUpdater() = default;

    virtual void Update(const FrameTime& frame_time) = 0;

#if IMP_RUNTIME(DEV)
    // Returns true if the updater should be stopped, when the editor is
    // in "Draft/Edit" or "Paused" mode.
    virtual bool ShouldRunInEditMode() const = 0;
#endif
  };

  // Used to receive an Update callback during an update phase when time
  // advances.
  //
  // Allows for an update phase and update dependencies to be specified to
  // control the order of updates.
  //
  // Component's can also define an Update method with phases & dependencies to
  // hook into this system, they do not need to write an Updater subclass to do
  // so.
  //
  // Defining an Updater looks like this:
  //
  // class Foo : public UpdateSystem::Updater<Foo> {
  //   public:
  //     // Optionally specify a phase. Defaults to UpdatePhase::kDefault.
  //     static constexpr UpdatePhase kUpdatePhase = UpdatePhase::kPreDefault;
  //
  //     // Optionally specify Updaters (could be Components or Updater
  //     // subclasses) that are updated before this one.
  //     using UpdateDependencies = UpdateIds<Barr>;
  //
  //     // Optionally specify Updaters (could be Components or Updater
  //     // subclasses) that are updated after this one.
  //     using UpdateDependents = UpdateIds<Bazz>;
  //
  //     explicit Foo(View& view) : Updater(view) {}
  //
  //     void Update(const FrameTime& frame_time) override;
  // };
  //
  // Updaters are automatically updated while they exist, they do not need to be
  // explicitly added/removed.
  //
  // The template parameter to Updater provides the phases & dependencies
  // information, which is typically the Updater subclass itself.
  //
  // NOTE: This uses the C++ pattern CRTP (A class using itself as it's
  // superclasses template parameter), which makes it possible for the phases &
  // dependencies to be specified in the same way as they are for Components
  // while allowing compile-time validation & automatic registration with the
  // UpdateSystem.
  //
  // However, T doesn't actually have to be the Updater subclass itself. In
  // fact, this is how Component's provide the phases & dependency information
  // without actually being Updater subclasses.
  template <typename T>
  class Updater : public BaseUpdater {
   public:
    explicit Updater(BaseView& view);
    ~Updater() override;
#if IMP_RUNTIME(DEV)
    bool ShouldRunInEditMode() const override { return false; }
#endif

   private:
    UpdateSystem& update_system_;
  };

  // Event that is sent right before all the component update calls are made.
  // It is recommended to use this event if a system or a non component class
  // requires to listen to update events over other options like
  // ViewPreFrameUpdateEvent or ViewPostRenderEvent which are bound to Filament.
  struct PreComponentsUpdateEvent : Event {
    explicit PreComponentsUpdateEvent(const FrameTime& frame_time)
        : frame_time(frame_time) {}
    FrameTime frame_time;
  };

  // Event that is sent right after all the component update calls are made.
  // It is recommended to use this event if a system or a non component class
  // requires to listen to update events over other options like
  // ViewPreFrameUpdateEvent or ViewPostRenderEvent which are bound to Filament.
  struct PostComponentsUpdateEvent : Event {
    explicit PostComponentsUpdateEvent(const FrameTime& frame_time)
        : frame_time(frame_time) {}
    FrameTime frame_time;
  };

  explicit UpdateSystem(BaseView* view);

  void Update(UpdatePhase phase, const FrameTime& frame_time);

 private:
  using UpdateGraph = DependencyGraph<UpdateId, BaseUpdater*>;

  template <typename T>
  void AddUpdater(Updater<T>* updater);

  void RemoveUpdater(UpdatePhase phase, UpdateId update_id);

  EnumIndexedArray<UpdatePhase, UpdateGraph, kNumUpdatePhases> phases_;
};

template <typename T>
void UpdateSystem::AddUpdater(Updater<T>* updater) {
  constexpr bool kHasDependencies =
      updater_traits::kAreUpdateDependenciesDefined<T>;
  constexpr bool kHasDependents =
      updater_traits::kAreUpdateDependentsDefined<T>;
  constexpr UpdateId update_id = kUpdateId<T>;
  constexpr UpdatePhase phase = updater_traits::kUpdatePhaseOrDefault<T>;

  UpdateGraph& update_graph = phases_[phase];

  if constexpr (kHasDependencies) {
    static_assert(type_traits::IsTemplateType<typename T::UpdateDependencies,
                                              UpdateIds>::value,
                  "UpdateDependencies must be of type UpdateIds.");
    // UpdateDependencies::kPhases is an aggregated list of the update phases of
    // all the dependencies. This enforces that T and T's dependencies are
    // all within the same phase.
    static_assert(DoAllPhasesMatch(phase, T::UpdateDependencies::kPhases),
                  "UpdateDependencies must all have the same UpdatePhase as "
                  "this updater.");

    if constexpr (T::UpdateDependencies::kIds.size() > 0) {
      for (UpdateId id : T::UpdateDependencies::kIds) {
        update_graph.AddDependency(update_id, id);
      }
    } else {
      // If there are no dependencies, add the updater to the graph.
      update_graph.AddNode(update_id);
    }
  }

  if constexpr (kHasDependents) {
    static_assert(type_traits::IsTemplateType<typename T::UpdateDependents,
                                              UpdateIds>::value,
                  "UpdateDependents must be of type UpdateIds.");
    // UpdateDependents::kPhases is an aggregated list of the update phases of
    // all the dependents. This enforces that T and T's dependents are
    // all within the same phase.
    static_assert(DoAllPhasesMatch(phase, T::UpdateDependents::kPhases),
                  "UpdateDependents must all have the same UpdatePhase as "
                  "this updater.");

    if constexpr (T::UpdateDependents::kIds.size() > 0) {
      for (UpdateId id : T::UpdateDependents::kIds) {
        update_graph.AddDependency(id, update_id);
      }
    } else {
      // If there are no dependents, add the updater to the graph.
      update_graph.AddNode(update_id);
    }
  }

  if constexpr (!kHasDependencies && !kHasDependents) {
    update_graph.AddNode(update_id);
  }

  update_graph.SetExtra(update_id, updater);
}

template <typename T>
UpdateSystem::Updater<T>::Updater(BaseView& view)
    : update_system_(view.GetUpdateSystem()) {
  update_system_.AddUpdater(this);
}

template <typename T>
UpdateSystem::Updater<T>::~Updater() {
  update_system_.RemoveUpdater(updater_traits::kUpdatePhaseOrDefault<T>,
                               kUpdateId<T>);
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_BEHAVIOR_SYSTEM_H_
