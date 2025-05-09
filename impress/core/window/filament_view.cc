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

#include "core/window/filament_view.h"

#include "filament/filament/include/filament/ColorGrading.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "filament/libs/utils/include/utils/EntityManager.h"

namespace imp::window::detail {

namespace {
using filament::Engine;
}  // namespace

absl::Status FilamentView::Setup(Engine* engine, std::string name,
                                 bool skip_color_grading) {
  view_ = engine->createView();
  camera_ = engine->createCamera(engine->getEntityManager().create());
  if (!view_ || !camera_)
    return absl::InternalError("Failed to setup FilamentView");
  view_->setCamera(camera_);
  view_->setName(name.c_str());
  if (engine->getActiveFeatureLevel() >=
          filament::backend::FeatureLevel::FEATURE_LEVEL_1 &&
      !skip_color_grading) {
    auto color_grading =
        filament::ColorGrading::Builder()
            .toneMapping(filament::ColorGrading::ToneMapping::FILMIC)
            .build(*engine);
    view_->setColorGrading(color_grading);
  }
  return absl::OkStatus();
}

absl::Status FilamentView::SetupShared(Engine* engine, filament::View* view,
                                       std::string name) {
  view_ = view;
  if (!view) return absl::InternalError("SetupShared did not get a valid view");
  camera_ = &view_->getCamera();
  view_->setName(name.c_str());
  owns_view_ = false;
  return absl::OkStatus();
}

void FilamentView::Cleanup(Engine* engine) {
  if (owns_view_) {
    if (camera_) {
      auto entity = camera_->getEntity();
      engine->destroyCameraComponent(entity);
      engine->getEntityManager().destroy(entity);
    }
    if (view_) {
      engine->destroy(view_);
    }
  }
  camera_ = nullptr;
  view_ = nullptr;
}

}  // namespace imp::window::detail
