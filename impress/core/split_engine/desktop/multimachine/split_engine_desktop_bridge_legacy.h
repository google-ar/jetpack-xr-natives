// Copyright 2025 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_MULTIMACHINE_SPLIT_ENGINE_DESKTOP_BRIDGE_LEGACY_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_MULTIMACHINE_SPLIT_ENGINE_DESKTOP_BRIDGE_LEGACY_H_

#include <cstdint>
#include <memory>
#include <utility>

#include "absl/types/span.h"
#include "core/common/invocable.h"
#include "core/split_engine/android/split_engine_android_bridge.h"
#include "core/split_engine/desktop/multimachine/split_engine_desktop_bridge_client.h"

namespace imp::split_engine {

// TODO: (broken link) - Remove this once Transport refactoring is merged.
class SplitEngineMMDesktopBridgeLegacy
    : public imp::split_engine::SplitEngineAndroidBridge {
 public:
  SplitEngineMMDesktopBridgeLegacy(
      std::unique_ptr<SplitEngineMMDesktopBridgeClient> client)
      : client_(std::move(client)) {};
  ~SplitEngineMMDesktopBridgeLegacy() override = default;

  bool SendRequest(
      absl::Span<const uint8_t> data,
      imp::Invocable<void(absl::Span<const uint8_t>)> callback) override;

 private:
  std::unique_ptr<SplitEngineMMDesktopBridgeClient> client_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_MULTIMACHINE_SPLIT_ENGINE_DESKTOP_BRIDGE_LEGACY_H_
