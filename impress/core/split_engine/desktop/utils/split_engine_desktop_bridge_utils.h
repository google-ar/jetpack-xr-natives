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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_SPLIT_ENGINE_DESKTOP_BRIDGE_UTILS_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_SPLIT_ENGINE_DESKTOP_BRIDGE_UTILS_H_

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string_view>
#include <thread>  // NOLINT

#include "absl/functional/any_invocable.h"
#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "core/split_engine/desktop/utils/split_engine_desktop_bridge_epoll.h"
#include "core/split_engine/desktop/utils/split_engine_desktop_bridge_uds.h"
#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {

// `FileDescriptorSender` and `FileDescriptorReceiver` implement special
// actions needed to transfer file descriptors between two processes correctly.
//
// They perform synchronous transfer of file descriptors over a Unix Domain
// Socket (UDS).
//
// The `FileDescriptorSender` sends a file descriptor and its metadata over a
// UDS to the `FileDescriptorReceiver`. The `FileDescriptorReceiver` receives
// the file descriptor and metadata, and then sends an acknowledgement back to
// the `FileDescriptorSender`.
//
// The `FileDescriptorSender` then receives the acknowledgement and
// assumes that the `FileDescriptorReceiver` has received the file descriptor
// and can use it.
//
// NOTE: The `FileDescriptorReceiver` is using just one thread to accept and
// process connections. Since the solution is intended for local single-machine
// development and testing, it is not expected to cause any issues (e.g.
// malicious app that will connect, but will not send any data). If more complex
// scenarios with lots of concurrent connections are encountered, this solution
// should be revisited.
//
// Metadata includes the size of the shared memory buffer file descriptor is
// associated with and the file descriptor value from the sending process
// (`fd`). The receiving side will map `fd` to whatever value is received via
// socket (which may be different from `fd`) and will use it later during
// `RegisterBuffer` RPC call to properly `mmap` the shared memory buffer.
//

struct FileDescriptorMetadata {
  BridgeId bridge_id;

  // File descriptor value from the sending process.
  int fd;
  // Size of the shared memory buffer file descriptor is associated with.
  size_t size;
};

class FileDescriptorSender : public UnixDomainSocket {
 public:
  // `uds_path`: The path to the Unix Domain Socket (UDS) to connect to.
  explicit FileDescriptorSender(std::string_view uds_path);

  // Sends a file descriptor and its metadata over a UDS.
  absl::Status Send(int fd, FileDescriptorMetadata& metadata);

 private:
  absl::Mutex mutex_;
  bool connected_ = false;

  // Connect to the receiver.
  absl::Status Connect();
  // Disconnect from the receiver.
  absl::Status Disconnect();
  // Send the file descriptor and its metadata over a UDS.
  absl::Status SendImpl(int fd, FileDescriptorMetadata& metadata);
  // Receive an acknowledgement from the receiver.
  absl::Status ReceiveAck();
};

class FileDescriptorReceiver : public UnixDomainSocket {
 public:
  using Callback = absl::AnyInvocable<void(int, const FileDescriptorMetadata&)>;

  // `uds_path`: The path to the Unix Domain Socket (UDS) to listen on.
  // `callback`: The callback to call when a new file descriptor is received.
  FileDescriptorReceiver(std::string_view uds_path, Callback&& callback);

  ~FileDescriptorReceiver();

  // Starts the server thread.
  absl::Status Start(uint32_t max_pending_connections = 5);

  // Stops the server thread.
  void Stop();

 private:
  // Starts the server thread.
  absl::Status StartServer(uint32_t max_pending_connections);
  // Runs the server thread.
  void Server();
  // Accepts all pending connections.
  absl::Status AcceptAll();
  // Sends an acknowledgement to the client.
  absl::Status SendAck(int fd);
  // Receives a file descriptor and its metadata from the client.
  absl::Status Receive(int client_fd);

  Callback callback_;

  absl::Mutex mutex_;
  std::thread server_thread_;
  int shutdown_pipe_[2] = {-1, -1};
  std::unique_ptr<EpollBase> epoll_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_SPLIT_ENGINE_DESKTOP_BRIDGE_UTILS_H_
