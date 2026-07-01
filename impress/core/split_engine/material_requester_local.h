/*
 * Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIAL_REQUESTER_LOCAL_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIAL_REQUESTER_LOCAL_H_

#include <cstddef>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "flatbuffers/buffer.h"
#include "core/async/future.h"
#include "core/split_engine/material_requester.h"
#include "core/split_engine/transport/request_sender.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

// The only goal of this implementation is to create flatbuffer builders in
// local mode when Split Engine Serializer is not available.
class MaterialRequesterLocal : public MaterialRequester {
 public:
  MaterialRequesterLocal();
  ~MaterialRequesterLocal() override = default;

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
  RequestSender request_sender_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIAL_REQUESTER_LOCAL_H_
