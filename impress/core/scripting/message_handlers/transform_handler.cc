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

#include "core/scripting/message_handlers/transform_handler.h"

#include <utility>

#include "core/ncsb/node.h"

namespace imp::scripting {

Future<GetTransformResponse> TransformHandler::HandleMessage(
    const GetTransformRequest& message) {
  if (auto err = CheckInvalidNodeTarget(message)) {
    return Future<GetTransformResponse>(err.value());
  }

  if (!message.translation && !message.rotation && !message.scale) {
    return Future<GetTransformResponse>(
        absl::InvalidArgumentError("GetTransformRequest with none of "
                                   "translation, rotation, or scale set."));
  }

  GetTransformResponse response;
  if (message.translation) {
    response.translation =
        message.transform_type == TransformType::TRANSFORM_TYPE_LOCAL
            ? message.target->GetLocalPosition()
            : message.target->GetWorldPosition();
  }
  if (message.rotation) {
    response.rotation =
        message.transform_type == TransformType::TRANSFORM_TYPE_LOCAL
            ? message.target->GetLocalRotation()
            : message.target->GetWorldRotation();
  }
  if (message.scale) {
    response.scale =
        message.transform_type == TransformType::TRANSFORM_TYPE_LOCAL
            ? message.target->GetLocalScale()
            : message.target->GetWorldScale();
  }
  return Future<GetTransformResponse>(std::move(response));
}

Future<absl::Status> TransformHandler::HandleMessage(
    const SetTransformRequest& message) {
  if (auto err = CheckInvalidNodeTarget(message)) {
    return Future<absl::Status>(err.value());
  }

  if (message.translation) {
    message.transform_type == TransformType::TRANSFORM_TYPE_LOCAL
        ? message.target->SetLocalPosition(*message.translation)
        : message.target->SetWorldPosition(*message.translation);
  }
  if (message.rotation) {
    message.transform_type == TransformType::TRANSFORM_TYPE_LOCAL
        ? message.target->SetLocalRotation(*message.rotation)
        : message.target->SetWorldRotation(*message.rotation);
  }
  if (message.scale) {
    message.transform_type == TransformType::TRANSFORM_TYPE_LOCAL
        ? message.target->SetLocalScale(*message.scale)
        : message.target->SetWorldScale(*message.scale);
  }
  return Future<absl::Status>(absl::OkStatus());
}

Future<GetTransformMatrixResponse> TransformHandler::HandleMessage(
    const GetTransformMatrixRequest& message) {
  if (auto err = CheckInvalidNodeTarget(message)) {
    return Future<GetTransformMatrixResponse>(err.value());
  }

  GetTransformMatrixResponse response;
  response.transform =
      message.transform_type == TransformType::TRANSFORM_TYPE_LOCAL
          ? message.target->GetLocalTrs()
          : message.target->GetWorldTrs();
  return Future<GetTransformMatrixResponse>(response);
}

Future<absl::Status> TransformHandler::HandleMessage(
    const SetTransformMatrixRequest& message) {
  if (auto err = CheckInvalidNodeTarget(message)) {
    return Future<absl::Status>(err.value());
  }

  message.transform_type == TransformType::TRANSFORM_TYPE_LOCAL
      ? message.target->SetLocalTrs(message.transform)
      : message.target->SetWorldTrs(message.transform);
  return Future<absl::Status>(absl::OkStatus());
}

}  // namespace imp::scripting
