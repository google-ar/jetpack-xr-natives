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

#include "core/split_engine/material_requester_legacy_impl.h"

#include <cstddef>
#include <memory>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "flatbuffers/buffer.h"
#include "core/async/future.h"
#include "core/common/owned_ptr.h"
#include "core/split_engine/android/split_engine_android_bridge.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/split_engine/transport/basic_transport_request_stub.h"
#include "core/split_engine/transport/request_sender.h"
#include "core/split_engine/transport/transport.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

MaterialRequesterLegacyImpl::MaterialRequesterLegacyImpl(
    SplitEngineAndroidBridge& bridge)
    : request_sender_(std::make_unique<RequestSender>(
          imp::OwnedPtr<Transport>(std::make_unique<RequestTransportStub>()))),
      bridge_(bridge) {}

absl::StatusOr<RequestSender::RequestBuilder>
MaterialRequesterLegacyImpl::CreateFlatBufferBuilder(size_t size) {
  return request_sender_->CreateRequestBuilder(size);
}

Future<absl::Status>
MaterialRequesterLegacyImpl::RequestBuiltInMaterialInstance(
    RequestSender::RequestBuilder fbb,
    flatbuffers::Offset<android_xr::schemas::BuiltInMaterialRequest> request) {
  return SendRequest<android_xr::schemas::BuiltInMaterialRequest, absl::Status>(
      bridge_, *fbb, request);
}

Future<absl::Status> MaterialRequesterLegacyImpl::AddCustomMaterial(
    RequestSender::RequestBuilder fbb,
    flatbuffers::Offset<android_xr::schemas::AddCustomMaterialRequest>
        request) {
  return SendRequest<android_xr::schemas::AddCustomMaterialRequest,
                     absl::Status>(bridge_, *fbb, request);
}

}  // namespace imp::split_engine
