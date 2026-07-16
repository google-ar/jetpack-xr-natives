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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_REQUEST_SENDER_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_REQUEST_SENDER_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/async/future.h"
#include "core/common/invocable.h"
#include "core/common/owned_or_borrowed_ptr.h"
#include "core/common/owned_ptr.h"
#include "core/split_engine/transport/flatbuffer_builder_holder.h"
#include "core/split_engine/transport/transport.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"

namespace imp::split_engine {

// RequestSender is the next layer on top of Transport: if Transport operates on
// spans of raw data, RequestSender operates on flatbuffers.
// Architecture / Layering:
//
//
//   [ Foreground Executor Thread ]
//                 | (DCHECK enforced)
//                 V
// ┌───────────────┴───────────────────────┐
// |             RequestSender             |
// ├───────────────────────────────────────┤
// | * Operates on Flatbuffers             |
// | * Content-agnostic (requests)         |
// | * Knows Response is a                 |
// |   android_xr::schemas::Response       |
// └───────────────┬───┬───────────────────┘
//       Request   |   ^  Response (via callback)
//                 V   |
// ┌───────────────┴───┴───────────────────┐
// |               Transport               |
// ├───────────────────────────────────────┤
// | * Operates on raw data spans          |
// └───────────────────────────────────────┘
//
// RequestSender knows that requests are represented by flatbuffers, but does
// not know about the content of the request.
//
// RequestSender knows that Response to any such request is a
// android_xr::schemas::Response flatbuffer.
//
// RequestSender shall be accessed from the foreground executor thread only.
//
class RequestSender {
 public:
  using FlatbufferBuilderHolder = FlatbufferBuilderHolder<RequestSender>;

  explicit RequestSender(imp::OwnedOrBorrowedPtr<Transport> transport);
  RequestSender(const RequestSender&) = delete;
  RequestSender& operator=(const RequestSender&) = delete;
  RequestSender(RequestSender&&) = default;
  RequestSender& operator=(RequestSender&&) = default;

  absl::StatusOr<imp::OwnedPtr<FlatbufferBuilderHolder>> CreateRequestBuilder(
      size_t size);

  // Sends a request to the Split Engine backend and return a
  // Future<ResponseT> that will be resolved when the response is received.
  template <typename RequestT, typename ResponseT>
  Future<ResponseT> SendRequest(imp::OwnedPtr<FlatbufferBuilderHolder> fbb,
                                flatbuffers::Offset<RequestT> request_offset);

 private:
  // [requirement] Requests are "two-way"
  struct Response {
   public:
    Response(std::vector<uint8_t> data)
        : data_(std::move(data)),
          response(flatbuffers::GetRoot<android_xr::schemas::Response>(
              data_.data())) {}

   private:
    const std::vector<uint8_t> data_;

   public:
    const android_xr::schemas::Response* const response;
  };

  using RequestCallback =
      imp::Invocable<void(absl::StatusOr<std::unique_ptr<Response>>)>;

  absl::Status SendRequest(imp::OwnedPtr<FlatbufferBuilderHolder> fbb,
                           RequestCallback callback);

  imp::OwnedOrBorrowedPtr<Transport> transport_;

  // Map is accessed from the foreground executor thread only, no need for
  // additional synchronization.
  absl::flat_hash_map<uint64_t, Transport::SessionID>
      flatbuffer_builder_to_session_id_;

  void StoreSessionID(uint64_t key, Transport::SessionID session_id);
  Transport::SessionID ExtractSessionID(uint64_t key);
};

template <typename RequestT, typename ResponseT>
Future<ResponseT> RequestSender::SendRequest(
    imp::OwnedPtr<FlatbufferBuilderHolder> fbb,
    flatbuffers::Offset<RequestT> request_offset) {
  (*fbb)->Finish(android_xr::schemas::CreateRequest(
      **fbb, android_xr::schemas::RequestTypesTraits<RequestT>::enum_value,
      request_offset.Union()));

  Future<ResponseT> result;
  const auto send_request_status = this->SendRequest(
      std::move(fbb),
      [result](
          absl::StatusOr<std::unique_ptr<RequestSender::Response>> status) {
        if (!status.ok()) {
          result.Return(status.status());
          return;
        }

        auto response = (*status)->response;
        if constexpr (std::is_same_v<ResponseT, absl::Status>) {
          // If the expected response is just absl::Status, just return an
          // absl::OkStatus (error status is handled above)
          result.Return(absl::OkStatus());
        } else {
          // A concrete Response type is expected. Verify the type
          // matches.
          if (response->response_type() !=
              android_xr::schemas::ResponseTypesTraits<ResponseT>::enum_value) {
            result.Return(absl::InvalidArgumentError(
                "Response type does not match expected type"));
            return;
          }
          // Return the concrete response type and take ownership of the
          // data.
          result.DependsOn(std::move(*status));
          result.Return(response->response_as<ResponseT>());
        }
      });

  if (!send_request_status.ok()) {
    return Future<ResponseT>(absl::InvalidArgumentError(absl::StrCat(
        "Bridge request failed: ", send_request_status.message())));
  }
  return result;
}

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_REQUEST_SENDER_H_
