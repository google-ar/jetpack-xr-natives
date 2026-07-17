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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_MESSAGE_GROUP_ID_GENERATOR_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_MESSAGE_GROUP_ID_GENERATOR_H_

#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/transport/transport.h"

namespace imp::split_engine {

class MessageGroupIdGenerator {
 public:
  MessageGroupIdGenerator() = default;
  MessageGroupIdGenerator(const MessageGroupIdGenerator&) = delete;
  MessageGroupIdGenerator& operator=(const MessageGroupIdGenerator&) = delete;
  MessageGroupIdGenerator(MessageGroupIdGenerator&&) = default;
  MessageGroupIdGenerator& operator=(MessageGroupIdGenerator&&) = default;

  virtual ~MessageGroupIdGenerator() = default;

  virtual MessageGroupId Generate(Transport::SessionID session_id) = 0;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_MESSAGE_GROUP_ID_GENERATOR_H_
