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

#include "core/split_engine/transport/message_group_sender.h"

#include <utility>
#include <variant>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "core/common/owned_ptr.h"
#include "core/common/pass_key.h"
#include "core/split_engine/transport/flatbuffer_builder_holder.h"
#include "core/split_engine/transport/message_sender.h"

namespace imp::split_engine {
MessageGroupSender::OwnedOrBorrowedFlatbufferBuilderHolder::
    OwnedOrBorrowedFlatbufferBuilderHolder(
        imp::OwnedPtr<FlatbufferBuilderHolder> fbb)
    : fbb_(std::move(fbb)) {}

MessageGroupSender::OwnedOrBorrowedFlatbufferBuilderHolder::
    OwnedOrBorrowedFlatbufferBuilderHolder(
        imp::BorrowedPtr<FlatbufferBuilderHolder> fbb)
    : fbb_(fbb) {}

absl::StatusOr<imp::OwnedPtr<MessageGroupSender::FlatbufferBuilderHolder>>
MessageGroupSender::OwnedOrBorrowedFlatbufferBuilderHolder::ExtractOwned(
    PassKey<MessageGroupSender> passkey) {
  if (std::holds_alternative<imp::OwnedPtr<FlatbufferBuilderHolder>>(fbb_)) {
    imp::OwnedPtr<FlatbufferBuilderHolder> result =
        std::move(std::get<imp::OwnedPtr<FlatbufferBuilderHolder>>(fbb_));
    fbb_ = std::monostate();

    return result;
  }

  return absl::FailedPreconditionError("Holder does not contain OwnedPtr");
}

absl::StatusOr<imp::BorrowedPtr<MessageGroupSender::FlatbufferBuilderHolder>>
MessageGroupSender::OwnedOrBorrowedFlatbufferBuilderHolder::ExtractBorrowed(
    PassKey<MessageGroupSender> passkey) {
  if (std::holds_alternative<imp::BorrowedPtr<FlatbufferBuilderHolder>>(fbb_)) {
    imp::BorrowedPtr<FlatbufferBuilderHolder> result =
        std::move(std::get<imp::BorrowedPtr<FlatbufferBuilderHolder>>(fbb_));
    fbb_ = std::monostate();

    return result;
  }

  return absl::FailedPreconditionError("Holder does not contain BorrowedPtr");
}

FlatbufferBuilderHolder<MessageSender>&
MessageGroupSender::OwnedOrBorrowedFlatbufferBuilderHolder::Get() {
  if (std::holds_alternative<imp::OwnedPtr<FlatbufferBuilderHolder>>(fbb_)) {
    return *std::get<imp::OwnedPtr<FlatbufferBuilderHolder>>(fbb_);
  } else if (std::holds_alternative<imp::BorrowedPtr<FlatbufferBuilderHolder>>(
                 fbb_)) {
    return *std::get<imp::BorrowedPtr<FlatbufferBuilderHolder>>(fbb_);
  } else {
    IMP_LOG(imp::FATAL) << "Holder is empty";
  }
}

PassKey<MessageGroupSender> MessageGroupSender::GetKey() {
  return PassKey<MessageGroupSender>();
}

}  // namespace imp::split_engine
