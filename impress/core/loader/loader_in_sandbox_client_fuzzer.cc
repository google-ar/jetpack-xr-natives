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
#include <deque>
#include <memory>
#include <string>
#include <thread>  // NOLINT
#include <utility>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/common/buffer_access.h"
#include "core/common/context.h"
#include "core/ipc/test_socket.h"
#include "core/loader/loader.h"
#include "core/loader/loader_in_sandbox.h"
#include "core/loader/loader_options.h"
#include "testing/test_view.h"
#include "third_party/llvm/llvm-project/compiler-rt/include/fuzzer/FuzzedDataProvider.h"

namespace imp::loader {

using ::imp::ipc::TestSocket;

class FuzzerState {
 public:
  FuzzerState(const uint8_t* data, size_t size) : stream_(data, size) {
    response_thread_ = std::thread(&FuzzerState::ResponseThreadWorker, this);
  }

  ~FuzzerState() {
    {
      absl::MutexLock lock(&lock_);
      quit_ = true;
    }
    response_thread_.join();
  }

  FuzzedDataProvider& Stream() { return stream_; }

  int RemoteFd() { return socket_.RemoteFd(); }

  void PushResponse() {
    absl::MutexLock lock(&lock_);
    const std::vector<uint8_t> response =
        stream_.ConsumeBytes<unsigned char>(stream_.remaining_bytes());
    responses_.push_back(response);
  }

 private:
  void ResponseThreadWorker() {
    while (true) {
      absl::MutexLock lock(&lock_);
      auto cond = [this]() ABSL_EXCLUSIVE_LOCKS_REQUIRED(&lock_) {
        return quit_ || !responses_.empty();
      };
      lock_.Await(absl::Condition(&cond));

      if (quit_) {
        return;
      }

      (void)socket_.ReadPacket();  // Ignore result.
      socket_.WritePacket(responses_.front());
      responses_.pop_front();
    }
  }

  FuzzedDataProvider stream_;
  TestSocket socket_;

  absl::Mutex lock_;
  std::thread response_thread_;
  bool quit_ ABSL_GUARDED_BY(lock_) = false;
  std::deque<std::vector<uint8_t>> ABSL_GUARDED_BY(lock_) responses_;
};

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
  FuzzerState state(data, size);
  const std::string path = state.Stream().ConsumeRandomLengthString(size);
  const std::vector<uint8_t> gltf_data =
      state.Stream().ConsumeBytes<unsigned char>(size);

  state.PushResponse();

  imp::testing::TestView test_view(std::make_unique<imp::Context>());
  absl::StatusOr<std::unique_ptr<Loader>> loader =
      LoaderInSandboxCreator().Create(
          *test_view.GetView(), state.RemoteFd(), path,
          BufferAccess::Wrap(gltf_data.data(), gltf_data.size()),
          {},  // materials_zip_bytes
          LoaderOptions{});
  if (!loader.status().ok()) {
    IMP_LOG(imp::INFO) << "LoaderInSandbox::Create(): " << loader.status();
    return 0;
  }

  std::vector<std::string> missing;
  std::shared_ptr<int> underlying_data = std::make_shared<int>(4);
  state.PushResponse();
  Future<absl::Status> load_future =
      (*loader)->TryLoad(&missing, [underlying_data]() {});
  while (!load_future.Ready()) {
    Executor::ForegroundExecutor()->Pump(false);
    absl::SleepFor(absl::Milliseconds(1));
  }
  if (!load_future.Get().ok()) {
    if (missing.empty()) {
      IMP_LOG(imp::INFO) << "TryLoad: " << load_future.Get();
      return 0;
    }
  }

  for (const std::string& file : missing) {
    // Since we are fuzzing the client, not the service, provide dummy data as
    // a response.
    BufferAccess file_data =
        BufferAccess::Wrap(reinterpret_cast<const uint8_t*>("test"), 4);

    state.PushResponse();
    if (absl::Status error =
            (*loader)->AddMissingResource(file, std::move(file_data));
        !error.ok()) {
      IMP_LOG(imp::INFO) << "AddMissingResource: " << error;
    }
  }

  state.PushResponse();
  load_future = (*loader)->TryLoad(&missing, [underlying_data]() {});
  while (!load_future.Ready()) {
    Executor::ForegroundExecutor()->Pump(false);
    absl::SleepFor(absl::Milliseconds(1));
  }
  if (!load_future.Get().ok()) {
    IMP_LOG(imp::INFO) << "TryLoad: " << load_future.Get();
  }

  return 0;
}

}  // namespace imp::loader
