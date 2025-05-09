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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_MESSAGE_ID_MAPPER_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_MESSAGE_ID_MAPPER_H_

#include "core/split_engine/flatbuffer_arena_allocator.h"
#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {

// An interface for mapping between flatbuffer arena handles and a range of
// message group IDs.
class MessageGroupIdMapper {
 public:
  using ArenaHandle = FlatbufferArenaAllocator::ArenaHandle;

  virtual ~MessageGroupIdMapper() = default;

  virtual MessageGroupId GetMessageGroupId(ArenaHandle arena_handle) = 0;
  virtual ArenaHandle GetArenaHandle(MessageGroupId message_group_id) = 0;

  /*
   * Returns true if the given message group id is within the range of valid
   * message IDs for this mapper.
   */
  virtual bool IsMessageGroupIdValid(MessageGroupId message_group_id) = 0;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_MESSAGE_ID_MAPPER_H_
