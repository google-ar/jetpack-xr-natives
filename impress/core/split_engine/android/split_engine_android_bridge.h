/*
 * Copyright 2024 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_SPLIT_ENGINE_BRIDGE_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_SPLIT_ENGINE_BRIDGE_H_

#include <jni.h>

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/types/span.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/verifier.h"
#include "core/async/future.h"
#include "core/common/invocable.h"
#include "core/split_engine/flatbuffer_utils.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"

namespace imp::split_engine {

// Interface for an Android specific Split Engine Bridge.
// TODO: (broken link) - Remove this once Transport refactoring is merged.
class SplitEngineAndroidBridge {
 public:
  SplitEngineAndroidBridge() = default;
  virtual ~SplitEngineAndroidBridge() = default;

  // Creates a texture surface bound to the given external texture id(s). Note
  // that additional texture ids are for stereo rendering.
  // Returns a Java android.view.Surface object backed by the external texture.
  virtual jobject CreateExternalTextureSurface(
      const std::vector<TextureId>& texture_ids) {
    // Default implementation to be used in desktop environment.
    // This is for pure compilation purposes only. External Texture Surface is
    // Android specific.
    return nullptr;
  };

  // Sets the resolution of the external texture surface bound to the given
  // texture id.
  virtual bool SetExternalTextureSurfaceSize(TextureId texture_id,
                                             int32_t width, int32_t height) {
    // Default implementation to be used in desktop environment.
    // This is for pure compilation purposes only. External Texture Surface is
    // Android specific.
    return false;
  };

  // Sends a flatbuffer request to the backend with a handler for a flatbuffer
  // response.
  virtual bool SendRequest(
      absl::Span<const uint8_t> data,
      imp::Invocable<void(absl::Span<const uint8_t>)> callback) = 0;
};

// Helper method to send a request to the Split Engine backend and return a
// Future<ResponseT> that will be resolved when the response is received.
template <typename RequestT, typename ResponseT>
Future<ResponseT> SendRequest(SplitEngineAndroidBridge& bridge,
                              flatbuffers::FlatBufferBuilder& fbb,
                              flatbuffers::Offset<RequestT> request_offset) {
  flatbuffers::Offset<android_xr::schemas::Request> request =
      android_xr::schemas::CreateRequest(
          fbb, android_xr::schemas::RequestTypesTraits<RequestT>::enum_value,
          request_offset.Union());
  fbb.Finish(request);
  Future<ResponseT> result;
  if (!bridge.SendRequest(
          absl::MakeConstSpan(fbb.GetBufferPointer(), fbb.GetSize()),
          [result](absl::Span<const uint8_t> response_data) {
            flatbuffers::Verifier verifier(response_data.data(),
                                           response_data.size());
            if (!verifier.VerifyBuffer<android_xr::schemas::Response>()) {
              result.Return(absl::InternalError(
                  "Invalid flatbuffer schema passed to SendRequest()"));
            }
            auto response_data_copy = std::make_unique<std::vector<uint8_t>>(
                response_data.data(),
                response_data.data() + response_data.size());
            std::vector<uint8_t>* response_data_copy_ptr =
                response_data_copy.get();
            const android_xr::schemas::Response* response =
                flatbuffers::GetRoot<android_xr::schemas::Response>(
                    response_data_copy_ptr->data());
            if (!response) {
              result.Return(
                  absl::InternalError("Failed to parse response from data"));
              return;
            }
            // If this is an ErrorResponse, convert and return the error status.
            if (response->response_type() ==
                android_xr::schemas::ResponseTypes::ErrorResponse) {
              const android_xr::schemas::ErrorResponse* error_response =
                  response->response_as<android_xr::schemas::ErrorResponse>();
              absl::Status status =
                  ErrorCodeToStatus(error_response->error_code(),
                                    error_response->error_message()->str());
              result.Return(status);
              return;
            }
            // If the expected response is empty, just return an absl::OkStatus.
            if constexpr (std::is_same_v<ResponseT, absl::Status>) {
              result.Return(absl::OkStatus());
            } else {
              // A concrete Response type is expected. Verify the type matches.
              if (response->response_type() !=
                  android_xr::schemas::ResponseTypesTraits<
                      ResponseT>::enum_value) {
                result.Return(absl::InvalidArgumentError(
                    "Response type does not match expected type"));
                return;
              }
              // Return the concrete response type and take ownership of the
              // data.
              result.DependsOn(std::move(response_data_copy));
              result.Return(response->response_as<ResponseT>());
            }
          })) {
    return Future<ResponseT>(
        absl::InvalidArgumentError("Bridge request failed"));
  }
  return result;
}

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_SPLIT_ENGINE_BRIDGE_H_
