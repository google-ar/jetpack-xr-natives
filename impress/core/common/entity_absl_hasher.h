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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_ENTITY_ABSL_HASHER_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_ENTITY_ABSL_HASHER_H_

#include <cstddef>

#include "absl/hash/hash.h"
#include "filament/libs/utils/include/utils/Entity.h"

namespace imp {

// Custom hasher for utils::Entity that plays well with absl.
//
// This is preferred over utils::Entity::Hasher because it uses absl::HashOf
// instead of just returning the entity id which works much better with
// absl containers.
struct EntityHasher {
  std::size_t operator()(const utils::Entity& entity) const {
    return absl::HashOf(entity.getId());
  }
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_ENTITY_ABSL_HASHER_H_
