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

#include "core/scripting/message_handlers/add_or_update_component_handler.h"

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/config.h"
#include "core/ncsb/node.h"
#include "core/scripting/proto/api.proto.imp.h"
#include "core/view/framework/scene/scene_system.h"

namespace imp::scripting {

#if IMP_RUNTIME(DEV)
constexpr absl::string_view kComponentDataTextproto = "any_data: %s";
#endif

absl::StatusOr<ComponentData> ParseComponentTextprotoAnyExpansion(
    absl::string_view any_expansion_textproto) {
#if IMP_RUNTIME(DEV)
  ComponentData component_data;
  std::string component_data_textproto =
      absl::StrFormat(kComponentDataTextproto, any_expansion_textproto);
  MP_RETURN_IF_ERROR(
      proto::ParseTextproto(component_data_textproto, &component_data));
  return component_data;
#else
  return absl::UnimplementedError(
      "Textproto data is not supported on non-DEV builds");
#endif
}

Future<absl::Status> AddOrUpdateComponentHandler::HandleMessage(
    const AddComponentRequest& message) {
  if (!message.target) {
    return Future<absl::Status>(absl::InvalidArgumentError("Invalid Node"));
  }
  if (message.component_data.any_data()) {
    return view_.GetSceneSystem().AddComponent(
        message.target, *message.component_data.any_data());
  }

  if (!message.component_data.textproto_data()) {
    return Future<absl::Status>(absl::InvalidArgumentError(
        "Request must contain either any or textproto component data."));
  }

  absl::StatusOr<ComponentData> component_data =
      ParseComponentTextprotoAnyExpansion(
          *message.component_data.textproto_data());
  if (!component_data.ok()) {
    return Future<absl::Status>(component_data.status());
  }

  return view_.GetSceneSystem().AddComponent(message.target,
                                             *component_data->any_data());
}

Future<absl::Status> AddOrUpdateComponentHandler::HandleMessage(
    const UpdateComponentRequest& message) {
  if (!message.target) {
    return Future<absl::Status>(absl::InvalidArgumentError("Invalid Node"));
  }
  if (message.component_data.any_data()) {
    return view_.GetSceneSystem().UpdateComponent(
        message.target, *message.component_data.any_data());
  }

  if (!message.component_data.textproto_data()) {
    return Future<absl::Status>(absl::InvalidArgumentError(
        "Request must contain either any or textproto component data."));
  }

  absl::StatusOr<ComponentData> component_data =
      ParseComponentTextprotoAnyExpansion(
          *message.component_data.textproto_data());
  if (!component_data.ok()) {
    return Future<absl::Status>(component_data.status());
  }

  return view_.GetSceneSystem().UpdateComponent(message.target,
                                                *component_data->any_data());
}

}  // namespace imp::scripting
