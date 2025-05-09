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
#include "core/loader/provider/extensions/gltf_extension_interactivity.h"
#include "core/loader/provider/extensions/interactivity/interactivity.proto.imp.h"
#include "core/loader/provider/extensions/interactivity/loader_extension.h"
#include "core/loader/provider/extensions/interactivity/model_creator_extension.h"
#include "core/proto/json_message_visitor.h"
#include "core/proto/json_reader.h"

namespace imp::loader::extensions {

class InteractivityNoop : public Interactivity {
 public:
  InteractivityNoop() = default;
  void AddHooks(proto::JsonMessageVisitor& json_message_visitor) override;

 private:
};

void InteractivityNoop::AddHooks(
    proto::JsonMessageVisitor& json_message_visitor) {
  // Registers a functor to handle the Interactivity Variable message.
  json_message_visitor.OnVisit([](imp::gltf::Interactivity& interactivity,
                                  int field_id, proto::JsonReader& visitor,
                                  const char* ptr,
                                  int token_type) -> absl::StatusOr<bool> {
    // Handles and skips parsing of the interactivity proto.
    visitor.Unknown(ptr, field_id, token_type);
    return true;
  });
}

std::unique_ptr<Interactivity> CreateInteractivityGltfExtension() {
  return std::make_unique<InteractivityNoop>();
}

std::unique_ptr<details::InteractivityLoaderExtension>
CreateInteractivityLoaderExtension(flatbuffers::FlatBufferBuilder& fbb) {
  return nullptr;
}

std::unique_ptr<details::InteractivityModelCreatorExtension>
CreateInteractivityModelCreatorExtension() {
  return nullptr;
}

}  // namespace imp::loader::extensions
