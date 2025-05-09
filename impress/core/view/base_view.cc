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

#include "core/view/base_view.h"

#include "filament/filament/include/filament/TransformManager.h"

namespace imp {

// Only one filament engine can exist per-thread. This is used to share the same
// filament engine across a thread using thread_local storage duration.
thread_local filament::Engine* s_shared_engine = nullptr;

filament::Engine* BaseView::GetSharedEngine() { return s_shared_engine; }

void BaseView::SetSharedEngine(filament::Engine* engine) {
  assert((s_shared_engine == engine) ||
         (engine ? (s_shared_engine == nullptr || s_shared_engine == engine)
                 : (s_shared_engine != nullptr)));
  s_shared_engine = engine;
}

void BaseView::SetPreciseTranslationEnabled(bool enabled) {
  GetSharedEngine()->getTransformManager().setAccurateTranslationsEnabled(
      enabled);
}

bool BaseView::IsPreciseTranslationEnabled() const {
  return GetSharedEngine()
      ->getTransformManager()
      .isAccurateTranslationsEnabled();
}

}  // namespace imp
