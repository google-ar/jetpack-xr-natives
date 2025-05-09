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

#include "core/editor/editor_info.h"

#include <functional>

#include "absl/status/statusor.h"
#include "core/common/registry.h"

namespace imp::editor {

bool IsInEditMode(Registry& registry) {
  absl::StatusOr<std::reference_wrapper<EditorInfo>> editor_info =
      registry.Get<EditorInfo>();

  if (editor_info.ok() && editor_info->get().IsEnabled() &&
      (editor_info->get().GetRunMode() == EditorInfo::RunMode::kEditMode ||
       editor_info->get().GetRunMode() ==
           EditorInfo::RunMode::kSwitchingToEditMode)) {
    return true;
  }

  return false;
}

bool IsPaused(imp::Registry& registry) {
  absl::StatusOr<std::reference_wrapper<EditorInfo>> editor_info =
      registry.Get<EditorInfo>();

  if (editor_info.ok() && editor_info->get().IsPaused() &&
      editor_info->get().IsEnabled()) {
    return true;
  }

  return false;
}

bool IsInEditModeOrPaused(imp::Registry& registry) {
  return IsInEditMode(registry) || IsPaused(registry);
}

bool HasFramesToStep(imp::Registry& registry) {
  absl::StatusOr<std::reference_wrapper<EditorInfo>> editor_info =
      registry.Get<EditorInfo>();

  if (editor_info.ok() && editor_info->get().IsEnabled() &&
      editor_info->get().HasFramesToStep()) {
    return true;
  }

  return false;
}

bool ShouldNotUpdate(imp::Registry& registry) {
  return IsInEditModeOrPaused(registry) && !HasFramesToStep(registry);
}

}  // namespace imp::editor
