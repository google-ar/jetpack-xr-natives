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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_IPC_LOADER_CLIENT_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_IPC_LOADER_CLIENT_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "filament/libs/utils/include/utils/Condition.h"
#include "filament/libs/utils/include/utils/Mutex.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/flatbuffers.h"
#include "core/common/buffer_access.h"
#include "core/common/flatbuffer_helpers.h"
#include "core/common/optional_error.h"
#include "core/ipc/message_pipe.h"
#include "core/loader/ipc/loader_client_base.h"
#include "core/loader/ipc/schemas/loader_ipc_generated.h"
#include "core/loader/loader_options.h"
#include "core/loader/provider/schemas/loaded_model_generated.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::loader::ipc {

using MessagePipe = ::imp::ipc::MessagePipe;

class LoaderClient : public LoaderClientBase {
 public:
  explicit LoaderClient(int fd);
  ~LoaderClient() override;

  OptionalError Start(absl::string_view uri, BufferAccess&& access,
                      LoaderOptions options) override;

  OptionalError TryLoad(std::vector<std::string>* out_missing_resource_paths,
                        bool* out_loaded) override;

  OptionalError AddResource(absl::string_view path, BufferAccess&& access,
                            ResourceType type) override;

  // Gets the results of the load.
  OptionalError GetLoadedModel(
      FlatBufferAccess<imp::schemas::LoadedModel>* out_model) override;

  void Close(bool wait_for_done_response) override;
  void EnsureClosed() override;

 private:
  // Callback when MessagePipe receives a message.
  void OnMessage(std::unique_ptr<uint8_t[]>&& message, size_t size);

  // Callback when the pipe is closed.
  void OnClosed();
  bool IsClosed();

  void SendRequest(const flatbuffers::FlatBufferBuilder& builder);
  OptionalError GetResponse(
      FlatBufferAccess<const schemas::Response>* out_response);

  template <typename T>
  OptionalError GetResponseAs(FlatBufferAccess<T>* out_response) {
    *out_response = FlatBufferAccess<T>{};

    FlatBufferAccess<const schemas::Response> response;
    MP_RETURN_IF_ERROR(GetResponse(&response));

    if (response->response_type() !=
        schemas::ResponseTypesTraits<T>::enum_value) {
      return Error("Unexpected response");
    }

    *out_response = FlatBufferAccess<T>{response->response_as<T>(),
                                        response.ReleaseBuffer()};
    if (!out_response->Root()) {
      return Error("Invalid response");
    }

    return NoError();
  }

  absl::Mutex lock_;
  absl::CondVar message_available_condition_;
  absl::CondVar has_storage_condition_;

  bool closed_ = false;
  std::unique_ptr<uint8_t[]> last_message_;
  size_t last_message_size_ = 0;
  MessagePipe pipe_;
};

}  // namespace imp::loader::ipc

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_IPC_LOADER_CLIENT_H_
