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

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <thread>  // NOLINT
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/common/buffer_access.h"
#include "core/common/context.h"
#include "core/loader/loader.h"
#include "core/loader/loader_in_process_embedded.h"
#include "core/loader/loader_options.h"
#include "core/loader/provider/gltf/gltf.pb.h"
#include "testing/test_view.h"

namespace imp::loader {

class LoaderFuzzer {
 public:
  LoaderFuzzer() {}
  void TestOneInput(const uint8_t* data, size_t size) {
    imp::testing::TestView test_view(std::make_unique<imp::Context>());
    for (auto extension : {".gltf", ".glb", ".zip", "sfb"}) {
      auto path = absl::StrFormat("fake_fuzz_file%s", extension);
      absl::StatusOr<std::unique_ptr<Loader>> loader = CreateLoaderInProcess(
          *test_view.GetView(), path, BufferAccess::Wrap(data, size),
          LoaderOptions{});
      if (!loader.ok()) {
        IMP_LOG(imp::INFO) << "Error loading " << loader.status();
        continue;
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
          continue;
        }
      } else {
        // Success, no reason to TryLoad again.
        continue;
      }

      for (const auto& file : missing) {
        // Since we are fuzzing the client, not the service, provide dummy data
        // as a response.
        constexpr auto kFakeFileSize = 1024 * 1024;
        BufferAccess file_data;
        if (auto ptr = BufferAccess::Create(kFakeFileSize, &file_data)) {
          std::memset(ptr, 0xD0, kFakeFileSize);
        } else {
          // Fall back to dummy storage
          file_data =
              BufferAccess::Wrap(reinterpret_cast<const uint8_t*>("test"), 4);
        }

        if (auto error =
                (*loader)->AddMissingResource(file, std::move(file_data));
            !error.ok()) {
          IMP_LOG(imp::INFO) << "AddMissingResource: " << error;
          continue;
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

 private:
};

}  // namespace imp::loader

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* Data, size_t Size) {
  static imp::loader::LoaderFuzzer loader_fuzzer;
  loader_fuzzer.TestOneInput(Data, Size);
  return 0;
}
