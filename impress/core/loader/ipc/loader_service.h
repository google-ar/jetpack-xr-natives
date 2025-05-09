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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_IPC_LOADER_SERVICE_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_IPC_LOADER_SERVICE_H_

#include <cstddef>
#include <cstdint>
#include <memory>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/common/optional_error.h"
#include "core/ipc/message_pipe.h"
#include "core/loader/ipc/schemas/loader_ipc_generated.h"
#include "core/loader/provider/provider.h"

namespace imp::loader::ipc {

using MessagePipe = ::imp::ipc::MessagePipe;

class LoaderService {
 public:
  explicit LoaderService(int fd);
  ~LoaderService();

 private:
  absl::StatusOr<MessagePipe::OnMessageResult> OnRequest(
      const schemas::Request* request, std::unique_ptr<uint8_t[]> storage);

  OptionalError HandleStart(const schemas::Start* request,
                            std::unique_ptr<uint8_t[]> storage);
  OptionalError HandleTryLoad(const schemas::TryLoad* request,
                              std::unique_ptr<uint8_t[]> storage);
  OptionalError HandleAddResource(const schemas::AddResource* request,
                                  std::unique_ptr<uint8_t[]> storage);
  OptionalError HandleGetLoadedModel(const schemas::GetLoadedModel* request,
                                     std::unique_ptr<uint8_t[]> storage);
  OptionalError HandleDone(const schemas::Done* request,
                           std::unique_ptr<uint8_t[]> storage);

  // Called by MessagePipe when a message is received.
  MessagePipe::OnMessageResult OnMessage(std::unique_ptr<uint8_t[]> message,
                                         size_t size);

  void SendErrorResponse(absl::string_view message);
  void SendMessage(const flatbuffers::FlatBufferBuilder& builder);
  void Close();
  MessagePipe pipe_;
  std::unique_ptr<Provider> provider_;
};

}  // namespace imp::loader::ipc

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_IPC_LOADER_SERVICE_H_
