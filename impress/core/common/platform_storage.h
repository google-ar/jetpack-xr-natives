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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_PLATFORM_STORAGE_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_PLATFORM_STORAGE_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/strings/string_view.h"

namespace imp {

// Abstract interface for platform-specific persistent key-value storage.
class PlatformStorage {
 public:
  virtual ~PlatformStorage() = default;
  virtual void SetString(absl::string_view key, absl::string_view value) = 0;
  virtual std::string GetString(absl::string_view key) = 0;

  virtual std::vector<std::string> GetKeys() const = 0;
  virtual void Remove(absl::string_view key) = 0;
  virtual void Clear() = 0;

  // Convenience method for storing a boolean value.
  void SetBool(absl::string_view key, bool value) {
    SetString(key, value ? "true" : "false");
  }

  // Convenience method for retrieving a boolean value.
  bool GetBool(absl::string_view key, bool default_value) {
    std::string val = GetString(key);
    if (val.empty()) {
      return default_value;
    }
    return val == "true";
  }
};

std::unique_ptr<PlatformStorage> CreatePlatformStorage();

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_PLATFORM_STORAGE_H_
