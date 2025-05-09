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

#ifndef THIRD_PARTY_IMPRESS_CORE_WINDOW_FILAMENT_VIEW_H_
#define THIRD_PARTY_IMPRESS_CORE_WINDOW_FILAMENT_VIEW_H_

#include <string>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Camera.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/View.h"

namespace imp::window::detail {

// Detail type for FilamentHost; wraps a filament::View
class FilamentView {
 public:
  // If skip_color_grading is true, Setup will not set a color grading. Instead
  // the default color grading for the engine will be used.
  absl::Status Setup(filament::Engine* engine, std::string name,
                     bool skip_color_grading = false);
  absl::Status SetupShared(filament::Engine* engine, filament::View* view,
                           std::string name);
  void Cleanup(filament::Engine* engine);

  filament::View* Get() { return view_; }
  filament::Camera* GetViewCamera() { return camera_; }

 private:
  bool owns_view_ = true;
  filament::Camera* camera_ = nullptr;
  filament::View* view_ = nullptr;
};

}  // namespace imp::window::detail

#endif  // THIRD_PARTY_IMPRESS_CORE_WINDOW_FILAMENT_VIEW_H_
