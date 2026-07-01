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

#include <memory>
#include <string>
#include <vector>

#include "absl/strings/string_view.h"
#include "core/common/platform_storage.h"

namespace imp {

class NoopPlatformStorage : public PlatformStorage {
 public:
  void SetString(absl::string_view key, absl::string_view value) override {}
  std::string GetString(absl::string_view key) override { return ""; }
  std::vector<std::string> GetKeys() const override { return {}; }
  void Remove(absl::string_view key) override {}
  void Clear() override {}
};

std::unique_ptr<PlatformStorage> CreatePlatformStorage() {
  return std::make_unique<NoopPlatformStorage>();
}

}  // namespace imp
