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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_MESSAGE_ID_MAPPER_IMPL_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_MESSAGE_ID_MAPPER_IMPL_H_

#include "core/split_engine/android/message_group_id_mapper.h"
#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {

// An implementation of MessageIdMapper that maps between flatbuffer arena
// handles and a range of message group IDs. This implementation specifies
// between two partitions based on the map_to_high_partition parameter.
//
// If map_to_high_partition is false, the mapper will map the handle to a
// message group ID in the lower 16 bits of the message group ID. This allows
// for up to 2^16 concurrent message groups to be open at any given time.
//
// If map_to_high_partition is true, the mapper will map the handle to the upper
// 48 bits of the message group ID. This allows for up to 2^48 concurrent
// message groups to be open at any given time.
class MessageIdMapperImpl : public MessageGroupIdMapper {
 public:
  MessageIdMapperImpl(bool map_to_high_partition);
  ~MessageIdMapperImpl() override = default;

  MessageGroupId GetMessageGroupId(ArenaHandle arena_handle) override;
  ArenaHandle GetArenaHandle(MessageGroupId message_group_id) override;

  bool IsMessageGroupIdValid(MessageGroupId message_group_id) override;

 private:
  const bool map_to_high_partition_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_MESSAGE_ID_MAPPER_IMPL_H_
