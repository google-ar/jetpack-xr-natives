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
#ifndef THIRD_PARTY_IMPRESS_CORE_MATERIALCOMPILER_TEST_MATERIAL_COMPILER_SERVICE_H_
#define THIRD_PARTY_IMPRESS_CORE_MATERIALCOMPILER_TEST_MATERIAL_COMPILER_SERVICE_H_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <utility>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "mediapipe/framework/port/status_matchers.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/verifier.h"
#include "core/ipc/message_pipe.h"
#include "core/materialcompiler/schemas/material_compiler_ipc_generated.h"

namespace imp_material_compiler {

class TestMaterialCompilerService {
 public:
  explicit TestMaterialCompilerService(int fd)
      : pipe_(
            fd,
            [](std::unique_ptr<uint8_t[]> data, size_t size,
               void* user) -> imp::ipc::MessagePipe::OnMessageResult {
              TestMaterialCompilerService* service =
                  static_cast<TestMaterialCompilerService*>(user);
              service->OnMessage(std::move(data), size);
              return imp::ipc::MessagePipe::OnMessageResult::kKeepAlive;
            },
            [](void* user) {
              TestMaterialCompilerService* service =
                  static_cast<TestMaterialCompilerService*>(user);
              service->OnPipeClosed();
            },
            this, "TestMaterialCompilerServicePipe") {}

  MOCK_METHOD(void, OnPipeClosed, ());
  MOCK_METHOD(void, OnRequest, (const imp::schemas::Request*));

  void SendResponse(const flatbuffers::FlatBufferBuilder& builder) {
    EXPECT_GT(builder.GetSize(), 0);
    EXPECT_LE(builder.GetSize(), std::numeric_limits<uint32_t>::max());
    EXPECT_TRUE(pipe_.Send(builder.GetBufferPointer(),
                           static_cast<uint32_t>(builder.GetSize())));
  }

  void Close() { pipe_.Close(); }

 private:
  void OnMessage(std::unique_ptr<uint8_t[]> message, size_t size) {
    auto verifier = flatbuffers::Verifier(message.get(), size);
    ASSERT_TRUE(verifier.VerifyBuffer<imp::schemas::Request>());
    const imp::schemas::Request* request =
        flatbuffers::GetRoot<imp::schemas::Request>(message.get());
    ASSERT_NE(request, nullptr);
    OnRequest(request);
  }

  imp::ipc::MessagePipe pipe_;
};
}  // namespace imp_material_compiler

#endif  // THIRD_PARTY_IMPRESS_CORE_MATERIALCOMPILER_TEST_MATERIAL_COMPILER_SERVICE_H_
