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

#include "core/monitor/default_monitor_summary.h"

#include <cstdint>
#include <optional>

#include "core/common/log.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "filament/filament/include/filament/Engine.h"
#include "core/monitor/monitor_summary.h"
#include "core/monitor/value_measurement.h"
#include "core/view/base_view.h"
#include "core/view/utils/frame_time.h"

namespace imp {
namespace {
constexpr absl::string_view kBufferObjectCount = "buffer objects: ";
constexpr absl::string_view kViewCount = "views: ";
constexpr absl::string_view kSceneCount = "scenes: ";
constexpr absl::string_view kSwapChainCount = "swap chains: ";
constexpr absl::string_view kStreamCount = "streams: ";
constexpr absl::string_view kIndexBufferCount = "index buffers: ";
constexpr absl::string_view kSkinningBufferCount = "skinning buffers: ";
constexpr absl::string_view kMorphTargetBufferCount = "morph target buffers: ";
constexpr absl::string_view kInstanceBufferCount = "instance buffers: ";
constexpr absl::string_view kVertexBufferCount = "vertex buffers: ";
constexpr absl::string_view kIndirectLightCount = "indirect lights: ";
constexpr absl::string_view kMaterialCount = "materials: ";
constexpr absl::string_view kTextureCount = "textures: ";
constexpr absl::string_view kSkyboxCount = "skyboxes: ";
constexpr absl::string_view kColorGradingCount = "color gradings: ";
constexpr absl::string_view kRenderTargetCount = "render targets: ";
}  // namespace

DefaultMonitorSummary::DefaultMonitorSummary(BaseView& view)
    : Updater(view),
      view_(view),
      monitor_(*view.GetMonitor()),
      summary_(monitor_),
      display_period_(absl::ZeroDuration()),
      bufferObjectCount_(ValueMeasurement(monitor_, kBufferObjectCount)),
      viewCount_(ValueMeasurement(monitor_, kViewCount)),
      sceneCount_(ValueMeasurement(monitor_, kSceneCount)),
      swapChainCount_(ValueMeasurement(monitor_, kSwapChainCount)),
      streamCount_(ValueMeasurement(monitor_, kStreamCount)),
      indexBufferCount_(ValueMeasurement(monitor_, kIndexBufferCount)),
      skinningBufferCount_(ValueMeasurement(monitor_, kSkinningBufferCount)),
      morphTargetBufferCount_(
          ValueMeasurement(monitor_, kMorphTargetBufferCount)),
      instanceBufferCount_(ValueMeasurement(monitor_, kInstanceBufferCount)),
      vertexBufferCount_(ValueMeasurement(monitor_, kVertexBufferCount)),
      indirectLightCount_(ValueMeasurement(monitor_, kIndirectLightCount)),
      materialCount_(ValueMeasurement(monitor_, kMaterialCount)),
      textureCount_(ValueMeasurement(monitor_, kTextureCount)),
      skyboxCount_(ValueMeasurement(monitor_, kSkyboxCount)),
      colorGradingCount_(ValueMeasurement(monitor_, kColorGradingCount)),
      renderTargetCount_(ValueMeasurement(monitor_, kRenderTargetCount)) {
  summary_.AddMetric<SampleAgeMetric>("avg sample age: ");

  summary_.AddMetric<ValueMetric>(kBufferObjectCount, kBufferObjectCount);
  summary_.AddMetric<ValueMetric>(kViewCount, kViewCount);
  summary_.AddMetric<ValueMetric>(kSceneCount, kSceneCount);
  summary_.AddMetric<ValueMetric>(kSwapChainCount, kSwapChainCount);
  summary_.AddMetric<ValueMetric>(kStreamCount, kStreamCount);
  summary_.AddMetric<ValueMetric>(kIndexBufferCount, kIndexBufferCount);
  summary_.AddMetric<ValueMetric>(kSkinningBufferCount, kSkinningBufferCount);
  summary_.AddMetric<ValueMetric>(kMorphTargetBufferCount,
                                  kMorphTargetBufferCount);
  summary_.AddMetric<ValueMetric>(kInstanceBufferCount, kInstanceBufferCount);
  summary_.AddMetric<ValueMetric>(kVertexBufferCount, kVertexBufferCount);
  summary_.AddMetric<ValueMetric>(kIndirectLightCount, kIndirectLightCount);
  summary_.AddMetric<ValueMetric>(kMaterialCount, kMaterialCount);
  summary_.AddMetric<ValueMetric>(kTextureCount, kTextureCount);
  summary_.AddMetric<ValueMetric>(kSkyboxCount, kSkyboxCount);
  summary_.AddMetric<ValueMetric>(kColorGradingCount, kColorGradingCount);
  summary_.AddMetric<ValueMetric>(kRenderTargetCount, kRenderTargetCount);
}

void DefaultMonitorSummary::Update(const FrameTime& frame_time) {
  static int64_t frame_count = 0;
  ++frame_count;

  if (frames_between_sampling_ &&
      (frame_count % frames_between_sampling_ == 0)) {
    filament::Engine* engine = view_.GetSharedEngine();
    if (!engine) {
      return;
    }
    bufferObjectCount_.SetValue(
        static_cast<int64_t>(engine->getBufferObjectCount()));
    viewCount_.SetValue(static_cast<int64_t>(engine->getViewCount()));
    sceneCount_.SetValue(static_cast<int64_t>(engine->getSceneCount()));
    swapChainCount_.SetValue(static_cast<int64_t>(engine->getSwapChainCount()));
    streamCount_.SetValue(static_cast<int64_t>(engine->getStreamCount()));
    indexBufferCount_.SetValue(
        static_cast<int64_t>(engine->getIndexBufferCount()));
    skinningBufferCount_.SetValue(
        static_cast<int64_t>(engine->getSkinningBufferCount()));
    morphTargetBufferCount_.SetValue(
        static_cast<int64_t>(engine->getMorphTargetBufferCount()));
    instanceBufferCount_.SetValue(
        static_cast<int64_t>(engine->getInstanceBufferCount()));
    vertexBufferCount_.SetValue(
        static_cast<int64_t>(engine->getVertexBufferCount()));
    indirectLightCount_.SetValue(
        static_cast<int64_t>(engine->getIndirectLightCount()));
    materialCount_.SetValue(static_cast<int64_t>(engine->getMaterialCount()));
    textureCount_.SetValue(static_cast<int64_t>(engine->getTextureCount()));
    skyboxCount_.SetValue(static_cast<int64_t>(engine->getSkyboxeCount()));
    colorGradingCount_.SetValue(
        static_cast<int64_t>(engine->getColorGradingCount()));
    renderTargetCount_.SetValue(
        static_cast<int64_t>(engine->getRenderTargetCount()));
  }

  summary_.Update();

  // log everything every frames_between_output_ frames
  if (frames_between_output_ && (frame_count % frames_between_output_ == 0)) {
    // output to log from string, for portability.
    IMP_LOG(imp::INFO) << absl::StrCat(summary_);
  }
}

void DefaultMonitorSummary::SetDisplayPeriod(
    absl::Duration maximum_display_period, absl::string_view histogram_name) {
  if (display_period_ == maximum_display_period) {
    return;
  }
  if (display_period_metric_) {
    absl::Status status =
        summary_.RemoveCustomMetric(display_period_metric_.value());
    if (!status.ok()) {
      IMP_LOG(imp::ERROR) << "Failed to remove display period metric: " << status;
    }
    display_period_metric_ = std::nullopt;
  }
  display_period_ = maximum_display_period;
  if (maximum_display_period == absl::ZeroDuration()) {
    return;
  }
  // frames which take past the maximum display period.
  int64_t lower_bound = absl::ToInt64Milliseconds(maximum_display_period);
  display_period_metric_ = summary_.AddMetric<PercentageOfHistogramMetric>(
      "  percent slow frames : ", histogram_name, lower_bound, std::nullopt);
}
}  // namespace imp
