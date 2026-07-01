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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIAL_REQUESTER_IMPL_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIAL_REQUESTER_IMPL_H_

#include <cstddef>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "flatbuffers/buffer.h"
#include "core/async/future.h"
#include "core/common/owned_ptr.h"
#include "core/split_engine/material_requester.h"
#include "core/split_engine/transport/request_sender.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

// Sends requests to the remote side via Split Engine transport.
// Calls are routed via `RequestSender`.
class MaterialRequesterImpl : public MaterialRequester {
 public:
  explicit MaterialRequesterImpl(
      imp::BorrowedPtr<RequestSender> request_sender);

  absl::StatusOr<RequestSender::RequestBuilder> CreateFlatBufferBuilder(
      size_t size) override;

  Future<absl::Status> RequestBuiltInMaterialInstance(
      RequestSender::RequestBuilder fbb,
      flatbuffers::Offset<android_xr::schemas::BuiltInMaterialRequest> request)
      override;

  Future<absl::Status> AddCustomMaterial(
      RequestSender::RequestBuilder fbb,
      flatbuffers::Offset<android_xr::schemas::AddCustomMaterialRequest>
          request) override;

 private:
  imp::BorrowedPtr<RequestSender> request_sender_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIAL_REQUESTER_IMPL_H_
