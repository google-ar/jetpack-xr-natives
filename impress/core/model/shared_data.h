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

#ifndef THIRD_PARTY_IMPRESS_CORE_MODEL_SHARED_DATA_H_
#define THIRD_PARTY_IMPRESS_CORE_MODEL_SHARED_DATA_H_

#include <cstdint>

#include "core/common/typed_id.h"

namespace imp::model {

struct EntityData;
using EntityId = TypedId<EntityData, uint16_t>;
using WeakEntityId = TypedId<EntityData, int32_t>;

}  // namespace imp::model

#endif  // THIRD_PARTY_IMPRESS_CORE_MODEL_SHARED_DATA_H_
