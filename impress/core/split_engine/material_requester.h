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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIAL_REQUESTER_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIAL_REQUESTER_H_

#include <cstddef>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "flatbuffers/buffer.h"
#include "core/async/future.h"
#include "core/split_engine/transport/request_sender.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

// Abstracts away how exactly requests are sent to the remote side.
//
// This interface is supposed to be future-proof. If AIDL will be changed to use
// shared memory for requests, callers of this interface will not need to be
// modified: creation of FlatBufferBuilder is abstracted away from the callers.
//
class MaterialRequester {
 public:
  virtual ~MaterialRequester() = default;

  // Creates a FlatBufferBuilder for material requests with the specified
  // size.
  virtual absl::StatusOr<RequestSender::RequestBuilder> CreateFlatBufferBuilder(
      size_t size) = 0;

  // Requests a built-in material instance (think filament::MaterialInstance).
  virtual Future<absl::Status> RequestBuiltInMaterialInstance(
      RequestSender::RequestBuilder fbb,
      flatbuffers::Offset<android_xr::schemas::BuiltInMaterialRequest>
          request) = 0;

  // Adds a custom material to the backend (think filament::Material).
  virtual Future<absl::Status> AddCustomMaterial(
      RequestSender::RequestBuilder fbb,
      flatbuffers::Offset<android_xr::schemas::AddCustomMaterialRequest>
          request) = 0;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIAL_REQUESTER_H_
