/*
 * Copyright 2025 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_MESSAGE_GROUP_SENDER_DISCRETE_LEGACY_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_MESSAGE_GROUP_SENDER_DISCRETE_LEGACY_H_

#include <cstdint>

#include "absl/log/check.h"
#include "absl/status/statusor.h"
#include "core/common/owned_or_borrowed_ptr.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/transport/message_group_id_generator.h"
#include "core/split_engine/transport/message_group_sender_discrete.h"
#include "core/split_engine/transport/message_sender.h"

namespace imp::split_engine {

// Implementation of DiscreteMessageGroupSender that relies on
// MessageGroupMonitor to count active message groups.
class DiscreteMessageGroupSenderLegacy : public DiscreteMessageGroupSender {
 public:
  DiscreteMessageGroupSenderLegacy(
      OwnedOrBorrowedPtr<MessageSender> sender,
      MessageGroupIdGenerator& message_group_id_generator, ClientId client_id);

  absl::StatusOr<int32_t> GetMessageGroupCount(
      MessageGroupType message_group_type) override;

 private:
  const ClientId client_id_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_MESSAGE_GROUP_SENDER_DISCRETE_LEGACY_H_
