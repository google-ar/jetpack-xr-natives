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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_VIEW_HOST_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_VIEW_HOST_H_

#include <memory>

#include "absl/status/status.h"
#include "core/config.h"
#include "core/view/base_view.h"
#include "core/window/filament_host.h"
#if IMP_RUNTIME(DEV)
#include "core/editor/editor_plugin.h"
#endif

namespace imp {

// Entry point for platform code to integrate with the impress framework.
//
// ViewHost owns the memory for the imp::View, and is used to plumb through
// lifecycle events from the platform to the imp::View.
//
// It also manages the creation of the filament::Engine and sharing the engine
// between instances.
//
// The imp::View is destroyed when ViewHost::Cleanup is called.
//
// ViewHost is a subclass of FilamentHost. FilamentHost handles plumbing through
// lifecycle events more generally to a client of filament. This class
// is a specialization of that functionality for handling imp::View.
class ViewHost : public window::FilamentHost {
 public:
  explicit ViewHost(std::unique_ptr<imp::BaseView> view);

  imp::BaseView* GetView();
  const imp::BaseView* GetView() const;

#if IMP_RUNTIME(DEV)
  // By default, this returns GetView()->CreateEditorPlugin().
  virtual std::unique_ptr<imp::editor::EditorPlugin> CreateEditorPlugin();
#endif

  // TODO: Refactor this and ExecutorTestHelper to share the
  // implementation of this method.
  // Wait for all pending work/futures to complete before returning.
  void DrainAllExecutorsForTest();

  // Flush the render pipeline, ensuring all render operations are complete.
  absl::Status StaticRenderForTest();
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_VIEW_HOST_H_
