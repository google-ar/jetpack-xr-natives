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

#include "core/loader/ipc/loader_client.h"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>  // NOLINT(build/c++11)
#include <string>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/verifier.h"
#include "core/common/buffer_access.h"
#include "core/common/flatbuffer_helpers.h"
#include "core/common/optional_error.h"
#include "core/loader/ipc/loader_client_base.h"
#include "core/loader/ipc/schemas/loader_ipc_generated.h"
#include "core/loader/loader_options.h"
#include "core/loader/provider/schemas/loaded_model_generated.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::loader::ipc {

namespace {
using OnMessageResult = MessagePipe::OnMessageResult;
}  // namespace

LoaderClient::LoaderClient(int fd)
    : pipe_(
          fd,
          [](std::unique_ptr<uint8_t[]> data, size_t size,
             void* user) -> OnMessageResult {
            LoaderClient* client = reinterpret_cast<LoaderClient*>(user);
            client->OnMessage(std::move(data), size);
            return OnMessageResult::kKeepAlive;
          },
          [](void* user) {
            LoaderClient* client = reinterpret_cast<LoaderClient*>(user);
            client->OnClosed();
          },
          this, "ClientPipe") {}

LoaderClient::~LoaderClient() { Close(false); }

OptionalError LoaderClient::Start(absl::string_view uri, BufferAccess&& access,
                                  LoaderOptions options) {
  flatbuffers::FlatBufferBuilder builder;
  auto command_offset = schemas::CreateStart(
      builder, builder.CreateString(uri.data(), uri.size()),
      builder.CreateVector(reinterpret_cast<const uint8_t*>(access.Data()),
                           access.Size()),
      options.use_lite_materials, options.exclude_excess_nodes,
      options.remove_shadow_planes,
      static_cast<int32_t>(options.compression_type));
  auto request_offset = schemas::CreateRequest(
      builder, schemas::RequestTypes::Start, command_offset.Union());
  builder.Finish(request_offset);

  SendRequest(builder);

  FlatBufferAccess<schemas::StartResponse> response;
  MP_RETURN_IF_ERROR(GetResponseAs(&response));

  return NoError();
}

OptionalError LoaderClient::TryLoad(
    std::vector<std::string>* out_missing_resource_paths, bool* out_loaded) {
  out_missing_resource_paths->clear();
  *out_loaded = false;

  flatbuffers::FlatBufferBuilder builder;
  auto command_offset = schemas::CreateTryLoad(builder);
  auto request_offset = schemas::CreateRequest(
      builder, schemas::RequestTypes::TryLoad, command_offset.Union());
  builder.Finish(request_offset);

  SendRequest(builder);

  FlatBufferAccess<schemas::TryLoadResponse> response;
  MP_RETURN_IF_ERROR(GetResponseAs(&response));

  bool has_missing_resources = false;
  if (response->missing_resource_paths()) {
    auto missing = response->missing_resource_paths();
    has_missing_resources = missing->size() > 0;
    for (size_t i = 0; i < missing->size(); ++i) {
      out_missing_resource_paths->push_back(missing->Get(i)->str());
    }
  }

  // It shouldn't be possible to be loaded if we have missing resources, and
  // vice versa.
  if ((response->loaded() && has_missing_resources) ||
      (!response->loaded() && !has_missing_resources)) {
    return Error("Load state mismatch, internal error");
  }

  *out_loaded = response->loaded();
  return NoError();
}

OptionalError LoaderClient::AddResource(absl::string_view path,
                                        BufferAccess&& access,
                                        ResourceType type) {
  flatbuffers::FlatBufferBuilder builder;
  auto command_offset = schemas::CreateAddResource(
      builder,
      /*was_missing=*/type == ResourceType::Missing,
      builder.CreateString(path.data(), path.size()),
      builder.CreateVector(reinterpret_cast<const uint8_t*>(access.Data()),
                           access.Size()));
  auto request_offset = schemas::CreateRequest(
      builder, schemas::RequestTypes::AddResource, command_offset.Union());
  builder.Finish(request_offset);

  SendRequest(builder);

  FlatBufferAccess<schemas::AddResourceResponse> response;
  MP_RETURN_IF_ERROR(GetResponseAs(&response));

  return NoError();
}

OptionalError LoaderClient::GetLoadedModel(
    FlatBufferAccess<imp::schemas::LoadedModel>* out_model) {
  flatbuffers::FlatBufferBuilder builder;
  auto command_offset = schemas::CreateGetLoadedModel(builder);
  auto request_offset = schemas::CreateRequest(
      builder, schemas::RequestTypes::GetLoadedModel, command_offset.Union());
  builder.Finish(request_offset);

  SendRequest(builder);

  FlatBufferAccess<schemas::GetLoadedModelResponse> response;
  MP_RETURN_IF_ERROR(GetResponseAs(&response));

  const imp::schemas::LoadedModel* model_root = response->buffer_nested_root();
  if (!model_root) {
    return Error("No model loaded");
  }

  *out_model = FlatBufferAccess<imp::schemas::LoadedModel>{
      model_root, response.ReleaseBuffer()};

  return NoError();
}

void LoaderClient::Close(bool wait_for_done_response) {
  if (IsClosed()) return;
  if (pipe_.IsClosed()) {
    IMP_LOG(imp::FATAL)
        << "fatal error in Loader Client: closed_ is true but pipe not closed";
  }

  flatbuffers::FlatBufferBuilder builder;
  auto command_offset = schemas::CreateDone(builder);
  auto request_offset = schemas::CreateRequest(
      builder, schemas::RequestTypes::Done, command_offset.Union());
  builder.Finish(request_offset);
  SendRequest(builder);

  if (wait_for_done_response) {
    FlatBufferAccess<schemas::DoneResponse> response;
    if (auto error = GetResponseAs(&response); !error.ok()) {
      IMP_LOG(imp::ERROR) << "Error getting Done response: " << error;
    }
  }
  pipe_.Close();
}

void LoaderClient::EnsureClosed() {
  if (IsClosed()) return;
  IMP_LOG(imp::ERROR)
      << "Client pipe not closed; sending Done without blocking on response";
  flatbuffers::FlatBufferBuilder builder;
  auto command_offset = schemas::CreateDone(builder);
  auto request_offset = schemas::CreateRequest(
      builder, schemas::RequestTypes::Done, command_offset.Union());
  builder.Finish(request_offset);
  SendRequest(builder);
  pipe_.Close();
}

void LoaderClient::OnMessage(std::unique_ptr<uint8_t[]>&& message,
                             size_t size) {
  absl::MutexLock lock(lock_);
  // Wait for the last message to be processed first.
  while (last_message_ != nullptr) {
    has_storage_condition_.Wait(&lock_);
  }
  last_message_ = std::move(message);
  last_message_size_ = size;
  message_available_condition_.Signal();
}

bool LoaderClient::IsClosed() {
  absl::MutexLock lock(lock_);
  return closed_;
}

void LoaderClient::OnClosed() {
  absl::MutexLock lock(lock_);
  closed_ = true;
  message_available_condition_.SignalAll();
}

void LoaderClient::SendRequest(const flatbuffers::FlatBufferBuilder& builder) {
  if (builder.GetSize() > std::numeric_limits<uint32_t>::max()) {
    IMP_LOG(imp::FATAL) << "Request too large";
  }
  pipe_.Send(builder.GetBufferPointer(),
             static_cast<uint32_t>(builder.GetSize()));
}

OptionalError LoaderClient::GetResponse(
    FlatBufferAccess<const schemas::Response>* out_response) {
  // If this fails, the caller did not supply valid output parameters.
  *out_response = FlatBufferAccess<const schemas::Response>{};
  {
    absl::MutexLock lock(lock_);
    while (last_message_ == nullptr && !closed_) {
      message_available_condition_.Wait(&lock_);
    }

    if (closed_) {
      IMP_LOG(imp::INFO) << "Pipe closed";
      return Error("Pipe closed");
    }

    auto message_buf = reinterpret_cast<const uint8_t*>(last_message_.get());
    auto verifier = flatbuffers::Verifier(message_buf, last_message_size_);
    if (!verifier.VerifyBuffer<schemas::Response>()) {
      IMP_LOG(imp::INFO) << "Failed to parse " << last_message_size_ << " byte response";
      return Error("Failed to parse %lu byte response", last_message_size_);
    }

    const schemas::Response* response =
        flatbuffers::GetRoot<schemas::Response>(message_buf);

    // If there is an error, return it as an OptionalError to continue error
    // propagation.
    if (response->response_type() == schemas::ResponseTypes::ErrorResponse) {
      auto error = response->response_as<schemas::ErrorResponse>();
      if (error && error->message()) {
        IMP_LOG(imp::ERROR) << "Received error from service: "
                   << error->message()->c_str();
        return Error(error->message()->c_str());
      } else {
        IMP_LOG(imp::ERROR) << "Unknown error from service";
        return Error("Internal error");
      }
    }

    *out_response = FlatBufferAccess<const schemas::Response>{
        response, BufferAccess{std::move(last_message_), last_message_size_}};
    last_message_size_ = 0;
    has_storage_condition_.Signal();
  }
  return NoError();
}

}  // namespace imp::loader::ipc
