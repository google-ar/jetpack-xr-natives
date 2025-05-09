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

#ifndef THIRD_PARTY_IMPRESS_CORE_AR_AR_TRACKABLE_ID_H_
#define THIRD_PARTY_IMPRESS_CORE_AR_AR_TRACKABLE_ID_H_

#include <cstdlib>
#include <utility>

namespace imp {
namespace ar {
// A unique identifier for AR trackable objects (planes, faces, images, etc.).
//
// This is a 128 bit identifier to accomodate iOS, where trackables are
// identified by a NSUUID object, which is 128 bits of information.
// Since there is no consistent 128 bit primitive type in c++, we are using two
// int64_t variables as storage - one for "low" bytes and one for "high" bytes.
class ArTrackableId {
  static constexpr int64_t kInvalid = int64_t(-1);

 public:
  constexpr ArTrackableId() : ArTrackableId(kInvalid) {}
  constexpr explicit ArTrackableId(int64_t low) : ArTrackableId(low, 0) {}
  constexpr ArTrackableId(int64_t low, int64_t high) : low_(low), high_(high) {}
  static constexpr ArTrackableId InvalidId() { return ArTrackableId(kInvalid); }
  bool operator<(const ArTrackableId& other) const {
    if (GetHigh() == other.GetHigh()) {
      return GetLow() < other.GetLow();
    }
    return GetHigh() < other.GetHigh();
  }
  bool operator==(const ArTrackableId& other) const {
    return GetLow() == other.GetLow() && GetHigh() == other.GetHigh();
  }
  bool operator!=(const ArTrackableId& other) const {
    return !(*this == other);
  }
  bool IsValid() const { return GetLow() != kInvalid; }
  int64_t GetLow() const { return low_; }
  int64_t GetHigh() const { return high_; }

  template <typename H>
  friend H AbslHashValue(H h, const ArTrackableId& id) {
    return H::combine(std::move(h), id.low_, id.high_);
  }

 private:
  int64_t low_;
  int64_t high_;
};
}  // namespace ar
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_AR_AR_TRACKABLE_ID_H_
