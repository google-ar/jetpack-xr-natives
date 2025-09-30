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

#include "core/split_engine/desktop/multimachine/split_engine_desktop_bridge_legacy.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "absl/types/span.h"
#include "core/split_engine/desktop/multimachine/split_engine_desktop_bridge_client.h"

namespace imp::split_engine {

bool SplitEngineMMDesktopBridgeLegacy::SendRequest(
    const std::vector<uint8_t>& data,
    std::function<void(const std::vector<uint8_t>&)> callback) {
  return client_
      ->SendRequest(
          data,
          [callback = std::move(callback)](absl::Span<const uint8_t> data) {
            callback(std::vector<uint8_t>(data.begin(), data.end()));
          })
      .ok();
}

}  // namespace imp::split_engine
