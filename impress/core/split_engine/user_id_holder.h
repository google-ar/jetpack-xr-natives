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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_USER_ID_HOLDER_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_USER_ID_HOLDER_H_

#include <cstdint>

#include "core/ncsb/component.h"

namespace imp::split_engine {

// Component for storing the user id assigned to a node so that it can be
// tracked.
class UserIdHolder : public Component {
 public:
  void Setup(uint32_t user_id);

  uint32_t GetUserId() const;

 private:
  uint32_t user_id_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_USER_ID_HOLDER_H_
