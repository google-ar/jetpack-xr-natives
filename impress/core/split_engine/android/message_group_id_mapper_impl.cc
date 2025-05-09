// Copyright 2024 Google LLC
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

#include "core/split_engine/android/message_group_id_mapper_impl.h"

#include "core/split_engine/android/message_group_id_mapper.h"
#include "core/split_engine/flatbuffer_arena_allocator.h"
#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {

namespace {

// Partition message group IDs into two ranges. The lower 16 bits are used if
// map_to_high_partition is false, and the upper 48 bits are used if
// map_to_high_partition is true.
constexpr int kHighOrderGroupIdShift = 8;
constexpr int kHighOrderMask = ~0XFF;

FlatbufferArenaAllocator::ArenaHandle ArenaHandleFromMessageGroupId(
    MessageGroupId message_group_id) {
  if (message_group_id & kHighOrderMask) {
    return (message_group_id >> kHighOrderGroupIdShift) - 1;
  }
  return message_group_id;
}
}  // namespace

MessageIdMapperImpl::MessageIdMapperImpl(bool map_to_high_partition)
    : map_to_high_partition_(map_to_high_partition) {}

MessageGroupId MessageIdMapperImpl::GetMessageGroupId(
    ArenaHandle arena_handle) {
  if (map_to_high_partition_) {
    // ArenaHandle is 0-based, so need to add 1 before shifting.
    return (1 + arena_handle) << kHighOrderGroupIdShift;
  }
  return arena_handle;
}

MessageGroupIdMapper::ArenaHandle MessageIdMapperImpl::GetArenaHandle(
    MessageGroupId message_group_id) {
  return ArenaHandleFromMessageGroupId(message_group_id);
}

bool MessageIdMapperImpl::IsMessageGroupIdValid(
    MessageGroupId message_group_id) {
  if (map_to_high_partition_) {
    return message_group_id & kHighOrderMask;
  }
  return (message_group_id & kHighOrderMask) == 0;
}

}  // namespace imp::split_engine
