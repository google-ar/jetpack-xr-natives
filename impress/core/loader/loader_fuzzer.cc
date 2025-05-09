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

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "security/fuzzing/blaze/proto_message_mutator.h"
#include "core/common/log.h"
#include "absl/random/distributions.h"
#include "absl/random/random.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "filament/filament/include/filament/Engine.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/common/buffer_access.h"
#include "core/common/context.h"
#include "core/loader/loader.h"
#include "core/loader/loader_in_process.h"
#include "core/loader/loader_options.h"
#include "core/loader/provider/gltf/gltf.pb.h"
#include "testing/test_view.h"
#include "google/protobuf/util/json_util.h"
#include "util/random/shared_bit_gen.h"

namespace imp::loader {

static void TestOneInput(const gltf::Gltf& gltf) {
  imp::testing::TestView test_view(std::make_unique<imp::Context>(),
                                   filament::Engine::Backend::VULKAN, nullptr);
  google::protobuf::util::JsonPrintOptions options;
  std::string gltf_data;
  const auto status =
      google::protobuf::util::MessageToJsonString(gltf, &gltf_data, options);
  if (status.ok()) {
    absl::StatusOr<std::unique_ptr<Loader>> loader = CreateLoaderInProcess(
        *test_view.GetView(),
        absl::StrCat("path",
                     absl::Uniform<uint64_t>(util_random::SharedBitGen())),
        BufferAccess::Wrap(reinterpret_cast<const uint8_t*>(gltf_data.data()),
                           gltf_data.size()),
        {},  // materials_zip_bytes
        LoaderOptions{});
    if (!loader.ok()) {
      IMP_LOG(imp::INFO) << "CreateLoaderInProc: " << loader.status();
      return;
    }

    std::vector<std::string> missing;
    std::shared_ptr<int> underlying_data = std::make_shared<int>(4);
    Future<absl::Status> load_future =
        (*loader)->TryLoad(&missing, [underlying_data]() {});
    while (!load_future.Ready()) {
      Executor::ForegroundExecutor()->Pump(false);
      absl::SleepFor(absl::Milliseconds(1));
    }
    if (!load_future.Get().ok()) {
      if (missing.empty()) {
        IMP_LOG(imp::INFO) << "TryLoad: " << load_future.Get();
      }
    }

    for (const auto& file : missing) {
      // Since we are fuzzing the client, not the service, provide dummy data
      // as a response.
      BufferAccess file_data =
          BufferAccess::Wrap(reinterpret_cast<const uint8_t*>("test"), 4);

      if (auto error =
              (*loader)->AddMissingResource(file, std::move(file_data));
          !error.ok()) {
        IMP_LOG(imp::INFO) << "AddMissingResource: " << error;
      }
    }

    load_future = (*loader)->TryLoad(&missing, [underlying_data]() {});
    while (!load_future.Ready()) {
      Executor::ForegroundExecutor()->Pump(false);
      absl::SleepFor(absl::Milliseconds(1));
    }
    if (!load_future.Get().ok()) {
      IMP_LOG(imp::INFO) << "TryLoad: " << load_future.Get();
    }
  }
}

}  // namespace imp::loader

DEFINE_BINARY_PROTO_FUZZER(const imp::gltf::Gltf& gltf) {
  imp::loader::TestOneInput(gltf);
}
