// Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_SPLIT_ENGINE_HEADLESS_OPENXR_CAMERA_UPDATER_H_
#define THIRD_PARTY_SPLIT_ENGINE_HEADLESS_OPENXR_CAMERA_UPDATER_H_

#include <memory>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "core/config.h"
#include "core/ncsb/update_phase.h"
#include "core/ncsb/update_system.h"
#include "core/view/base_view.h"
#include "core/view/utils/frame_time.h"

namespace imp {
namespace split_engine {

// Lightweight & native alternative to SplitEngineViewUpdateProvider for
// synchronizing camera pose with the device pose in a Split Engine app.
//
// You need to create one of these in your Split Engine app if you want to do
// camera-relative business logic, since otherwise the camera will remain in the
// default position and orientation and not reflect the actual device pose.
//
// Example usage:
//   absl::StatusOr<std::unique_ptr<HeadlessOpenXrCameraUpdater>> updater =
//       HeadlessOpenXrCameraUpdater::Create(view);
//   if (!updater.ok()) {
//     // Handle error.
//   }
//   view.GetRegistry().Register(std::move(*updater));
class HeadlessOpenXrCameraUpdater
    : public imp::UpdateSystem::Updater<HeadlessOpenXrCameraUpdater> {
 public:
  // Hook the update phase very early so any component logic running in kDefault
  // evaluates math relative to the newest camera pose.
  static constexpr UpdatePhase kUpdatePhase = UpdatePhase::kStart;

  static absl::StatusOr<std::unique_ptr<HeadlessOpenXrCameraUpdater>> Create(
      imp::BaseView& view);

  ~HeadlessOpenXrCameraUpdater() override;

  void Update(const imp::FrameTime& frame_time) override;

#if IMP_RUNTIME(DEV)
  bool ShouldRunInEditMode() const override { return true; }
#endif

 private:
  explicit HeadlessOpenXrCameraUpdater(imp::BaseView& view);
  absl::Status Setup();

  struct OpenXrContext;
  std::unique_ptr<OpenXrContext> ctx_;
  imp::BaseView& view_;
};

}  // namespace split_engine
}  // namespace imp

#endif  // THIRD_PARTY_SPLIT_ENGINE_HEADLESS_OPENXR_CAMERA_UPDATER_H_
