/*
 * Copyright 2025 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_UI_EVENT_LISTENER_H_
#define THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_UI_EVENT_LISTENER_H_

#include <string>

#include "core/math/quat.h"
#include "core/math/vec.h"

namespace svxr {

class UiEventListener {
 public:
  virtual ~UiEventListener() = default;

  virtual void OnStartup() = 0;
  virtual void OnShutdown() = 0;

  virtual void OnLoadIndicatorAnchorUpdated(imp::float3 translation,
                                            imp::quatf rotation, float scale,
                                            float alpha,
                                            float scale_percentage) = 0;
  virtual void OnCloseLoadIndicator() = 0;

  virtual void OnPanelAnchorUpdated(imp::float3 translation,
                                    imp::quatf rotation, float scale,
                                    float alpha) = 0;
  virtual void OnScaleIndicatorAnchorUpdated(imp::float3 translation,
                                             imp::quatf rotation, float scale,
                                             float alpha,
                                             float scale_percentage) = 0;
  virtual void OnA11yRotateLeftControlAnchorUpdated(imp::float3 translation,
                                                    imp::quatf rotation,
                                                    float scale,
                                                    float alpha) = 0;
  virtual void OnA11yRotateRightControlAnchorUpdated(imp::float3 translation,
                                                     imp::quatf rotation,
                                                     float scale,
                                                     float alpha) = 0;
  virtual void OnA11yScaleControlAnchorUpdated(imp::float3 translation,
                                               imp::quatf rotation, float scale,
                                               float alpha) = 0;

  virtual void OnMessageShowed(const std::string& message) = 0;

  virtual void SetResetSizeButtonToOneToOne() = 0;
  virtual void SetResetSizeButtonToReset() = 0;

  virtual bool IsTalkbackEnabled() = 0;

  virtual void OnPlaneRefreshRequested() = 0;

  virtual void OnModelRotated() = 0;
  virtual void OnModelScaled() = 0;
  virtual void OnModelSelected(bool selected) = 0;
  virtual void OnModelScaleChanged(float scale) = 0;
  virtual void OnModelTranslated() = 0;
};

}  // namespace svxr

#endif  // THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_UI_EVENT_LISTENER_H_
