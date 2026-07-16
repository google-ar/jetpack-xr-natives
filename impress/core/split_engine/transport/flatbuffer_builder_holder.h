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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_FLATBUFFER_BUILDER_HOLDER_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_FLATBUFFER_BUILDER_HOLDER_H_

#include <memory>
#include <utility>

#include "flatbuffers/flatbuffer_builder.h"
#include "core/common/invocable.h"
#include "core/common/pass_key.h"

namespace imp::split_engine {

// Ensures that senders are using correct FlatbufferBuilder
template <typename Owner>
class FlatbufferBuilderHolder {
 public:
  using CleanupCallback = imp::Invocable<void()>;
  FlatbufferBuilderHolder(
      PassKey<Owner> passkey,
      std::unique_ptr<flatbuffers::FlatBufferBuilder> builder,
      CleanupCallback cleanup_callback = {})
      : FlatbufferBuilderHolder(std::move(builder),
                                std::move(cleanup_callback)) {}

  ~FlatbufferBuilderHolder() {
    builder_.reset();

    if (cleanup_callback_) {
      cleanup_callback_();
    }
  }

  FlatbufferBuilderHolder(const FlatbufferBuilderHolder&) = delete;
  FlatbufferBuilderHolder& operator=(const FlatbufferBuilderHolder&) = delete;
  FlatbufferBuilderHolder(FlatbufferBuilderHolder&&) = default;
  FlatbufferBuilderHolder& operator=(FlatbufferBuilderHolder&&) = default;

  flatbuffers::FlatBufferBuilder& operator*() const { return *builder_; }
  flatbuffers::FlatBufferBuilder* operator->() const { return builder_.get(); }

  void CancelCleanup(PassKey<Owner> passkey) { cleanup_callback_ = {}; }

 private:
  FlatbufferBuilderHolder(
      std::unique_ptr<flatbuffers::FlatBufferBuilder> builder,
      CleanupCallback cleanup_callback = {})
      : builder_(std::move(builder)),
        cleanup_callback_(std::move(cleanup_callback)) {}

  std::unique_ptr<flatbuffers::FlatBufferBuilder> builder_;
  CleanupCallback cleanup_callback_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_FLATBUFFER_BUILDER_HOLDER_H_
