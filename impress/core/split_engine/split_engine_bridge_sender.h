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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_BRIDGE_SENDER_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_BRIDGE_SENDER_H_

#include <cstddef>
#include <memory>

#include "flatbuffers/flatbuffer_builder.h"
#include "core/split_engine/flatbuffer_arena_allocator.h"

namespace imp::split_engine {

// An interface for SplitEngine to use to send serialized data to a renderer.
class SplitEngineBridgeSender {
 public:
  virtual ~SplitEngineBridgeSender() = default;

  // Sends a message using the default active message group.
  virtual void SendMessage(const flatbuffers::FlatBufferBuilder& message) = 0;

  // Begin a new message group which will be contained within buffer of
  // specified size. The SplitEngineBridgeSender will handle adding overhead
  // for begin and end message group messages.
  virtual void BeginMessageGroup(size_t size_bytes) = 0;

  // Ends the current message group and mark it eligible for release once
  // all messages in the group have been processed. A new message group will
  // need to be started before any more messages can be sent.
  virtual void EndMessageGroup() = 0;

  // True if we're between a BeginMessageGroup/EndMessageGroup pair.
  virtual bool IsMessageGroupActive() const = 0;

  // Creates a new flatbuffer builder that is backed by a buffer of the
  // specified size. It is the responsibility of the caller to ensure that the
  // the current message group has sufficient space to contain the builder.
  //
  // The memory backing the FlatBufferBuilder is owned by the
  // SplitEngineBridgeSender, and so the returned FlatBufferBuilder is only
  // valid for the lifetime of the SplitEngineBridgeSender.
  virtual std::unique_ptr<flatbuffers::FlatBufferBuilder>
  CreateFlatBufferBuilder(size_t size_bytes) = 0;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_BRIDGE_SENDER_H_
