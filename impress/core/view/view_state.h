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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_VIEW_STATE_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_VIEW_STATE_H_

#include <memory>
#include <string>

#include "absl/status/status.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Engine.h"
#include "core/common/optional_error.h"
#include "core/view/base_view.h"
#include "core/window/filament_host.h"

namespace imp {

// FilamentHost::State is the client hook which must be provided when creating a
// FilamentHost. ViewState is the common state type used by Impress views.
class ViewState : public window::FilamentHost::State {
 public:
  explicit ViewState(std::unique_ptr<imp::BaseView> view);
  ~ViewState() override;

  imp::BaseView* GetView() { return view_.get(); }
  const imp::BaseView* GetView() const { return view_.get(); }

  void OnHostCreated(window::FilamentHost* filament_host);

  filament::Engine::Config GetEngineConfig() const override;
  filament::backend::FeatureLevel GetMaximumEngineFeatureLevel() const override;
  bool ShouldStartPaused() const override;

  bool ShouldUseSharedGlContext() const override;

  bool ShouldUseSystemFrameScheduledHandler() const override;

 protected:
  using RenderResult = window::FilamentHost::RenderResult;
  using RenderResultFlags = window::FilamentHost::RenderResultFlags;
  using UpdateStageFlags = window::FilamentHost::UpdateStageFlags;

  // FilamentHost::State methods
  OptionalError Setup(window::FilamentHost* filament_host) override;

  absl::Status PreUpdate(
      window::FilamentHost* filament_host, absl::Duration last_vsync,
      absl::Duration next_vsync, UpdateStageFlags* out_flags,
      absl::optional<absl::Duration>* out_time_until_retry) override;
  absl::Status Update(window::FilamentHost* filament_host,
                      absl::Duration last_vsync, absl::Duration next_vsync,
                      const UpdateStageFlags& update_flags) override;
  absl::Status PostUpdate(window::FilamentHost* filament_host) override;

  bool IsStillRendering(window::FilamentHost* filament_host) override;
  bool IsUiDesired(window::FilamentHost* filament_host) override;
  absl::Status PreRender(window::FilamentHost* filament_host) override;
  OptionalError OffscreenRender(window::FilamentHost* filament_host,
                                filament::Renderer* filament_renderer) override;
  OptionalError MultiPassRender() override;
  OptionalError PostRender(window::FilamentHost* filament_host) override;
  OptionalError SecondaryViewRender(
      window::FilamentHost* filament_host) override;
  OptionalError PostFrame(window::FilamentHost* filament_host) override;
  OptionalError UiRender(window::FilamentHost* filament_host) override;
  OptionalError Cleanup(window::FilamentHost* filament_host) override;
  std::string Title(window::FilamentHost* filament_host) override;

  OptionalError SetDisplayRotation(window::FilamentHost* filament_host,
                                   window::WindowRotation orientation) override;

  void OnResize(window::FilamentHost* filament_host, uint2 dimensions,
                uint4 margins, float2 subpixel_ratio) override;

  OptionalError OnFileDrop(window::FilamentHost* filament_host,
                           absl::string_view path) override;

  void NotifyLast(window::FilamentHost* filament_host) override;
  OptionalError Pause() override;
  OptionalError Resume() override;

 private:
  std::unique_ptr<BaseView> view_;
  ViewHooks& view_hooks_;
  std::string title_;
  bool is_multisampled_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_FRAMEWORK_VIEW_VIEW_STATE_H_
