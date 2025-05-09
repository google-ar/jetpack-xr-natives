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

#include <memory>

#include "absl/status/statusor.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/loader/provider/extensions/behavior/behavior.proto.imp.h"
#include "core/loader/provider/extensions/behavior/loader_extension.h"
#include "core/loader/provider/extensions/behavior/model_creator_extension.h"
#include "core/loader/provider/extensions/gltf_extension_behavior.h"
#include "core/proto/json_message_visitor.h"
#include "core/proto/json_reader.h"

namespace imp::loader::extensions {

class BehaviorNoop : public Behavior {
 public:
  BehaviorNoop() = default;
  void AddHooks(proto::JsonMessageVisitor& json_message_visitor) override;

 private:
};

void BehaviorNoop::AddHooks(proto::JsonMessageVisitor& json_message_visitor) {
  // Registers a functor to handle the Behavior Variable message.
  json_message_visitor.OnVisit([](imp::gltf::Behavior& behavior, int field_id,
                                  proto::JsonReader& visitor, const char* ptr,
                                  int token_type) -> absl::StatusOr<bool> {
    // Handles and skips parsing of the behavior proto.
    visitor.Unknown(ptr, field_id, token_type);
    return true;
  });
}

std::unique_ptr<Behavior> CreateBehaviorGltfExtension() {
  return std::make_unique<BehaviorNoop>();
}

std::unique_ptr<details::BehaviorLoaderExtension> CreateBehaviorLoaderExtension(
    flatbuffers::FlatBufferBuilder& fbb) {
  return nullptr;
}

std::unique_ptr<details::BehaviorModelCreatorExtension>
CreateBehaviorModelCreatorExtension() {
  return nullptr;
}

}  // namespace imp::loader::extensions
