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

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string_view>
#include <thread>  // NOLINT
#include <utility>

#include "absl/cleanup/cleanup.h"
#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "core/split_engine/desktop/utils/split_engine_desktop_bridge_epoll.h"
#include "core/split_engine/desktop/utils/split_engine_desktop_bridge_uds.h"
#include "core/split_engine/desktop/utils/split_engine_desktop_bridge_utils.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::split_engine {

FileDescriptorReceiver::FileDescriptorReceiver(const std::string_view uds_path,
                                               Callback&& callback)
    : UnixDomainSocket(uds_path, true), callback_(std::move(callback)) {
  epoll_ = std::make_unique<EpollImpl<10>>();
}

FileDescriptorReceiver::~FileDescriptorReceiver() {
  Stop();

  if (shutdown_pipe_[0] != -1) {
    close(shutdown_pipe_[0]);
  }
  if (shutdown_pipe_[1] != -1) {
    close(shutdown_pipe_[1]);
  }
}

absl::Status FileDescriptorReceiver::Start(uint32_t max_pending_connections) {
  absl::MutexLock lock(&mutex_);
  if (server_thread_.joinable()) {
    return absl::AlreadyExistsError("Server already started");
  }
  MP_RETURN_IF_ERROR(StartServer(max_pending_connections));
  return absl::OkStatus();
}

void FileDescriptorReceiver::Stop() {
  absl::MutexLock lock(&mutex_);
  if (!server_thread_.joinable()) {
    return;
  }
  write(shutdown_pipe_[1], "\x42", 1);
  server_thread_.join();
}

absl::Status FileDescriptorReceiver::StartServer(
    uint32_t max_pending_connections) {
  SetNonBlocking(GetSocketFd());

  struct sockaddr_un addr;
  memset(&addr, 0, sizeof(addr));
  addr.sun_family = AF_UNIX;
  strncpy(addr.sun_path, GetUdsPath().data(), sizeof(addr.sun_path) - 1);

  if (bind(GetSocketFd(), (struct sockaddr*)&addr, sizeof(addr)) == -1) {
    return absl::ErrnoToStatus(errno, "bind failed");
  }

  if (listen(GetSocketFd(), max_pending_connections) == -1) {
    return absl::ErrnoToStatus(errno, "listen failed");
  }

  if (pipe(shutdown_pipe_) == -1) {
    return absl::ErrnoToStatus(errno, "pipe failed");
  }

  absl::Cleanup shutdown_pipe_close = [this] {
    close(shutdown_pipe_[0]);
    shutdown_pipe_[0] = -1;
    close(shutdown_pipe_[1]);
    shutdown_pipe_[1] = -1;
  };

  SetNonBlocking(shutdown_pipe_[0]);

  // Server is using `epoll` (Linux) or `kevent` (MacOS) to wait for some action
  // on provided file descriptors.
  //
  // Two file descriptors are used:
  //
  //  - `shutdown_pipe_[0]` is "reading end" of the pipe. If we want to stop the
  //  server we'll write to the "writing end" of the pipe (`shutdown_pipe_[1]`)
  //  and epoll will notify us about that.
  //
  //  - `GetSocketFd()` is used to accept new connections. epoll will notify us
  //  if there are some new connections waiting.
  //
  MP_RETURN_IF_ERROR(epoll_->Setup(GetSocketFd(), shutdown_pipe_[0]));

  server_thread_ = std::thread(&FileDescriptorReceiver::Server, this);
  std::move(shutdown_pipe_close).Cancel();

  return absl::OkStatus();
}

void FileDescriptorReceiver::Server() {
  while (true) {
    const absl::StatusOr<int> epoll_result = epoll_->Poll();
    
    const int number_of_events = *epoll_result;

    for (int i = 0; i < number_of_events; ++i) {
      const absl::StatusOr<int> event_fd_result = epoll_->GetFd(i);
      
      const int event_fd = *event_fd_result;

      if (event_fd == shutdown_pipe_[0]) {
        return;
      } else if (event_fd == GetSocketFd()) {
        
      }
    }
  }
}

absl::Status FileDescriptorReceiver::AcceptAll() {
  IMP_LOG(imp::INFO) << "AcceptAll";
  while (true) {
    struct sockaddr_un client_addr;
    socklen_t client_addr_len = sizeof(client_addr);
    const int client_sock_fd =
        accept(GetSocketFd(), (struct sockaddr*)&client_addr, &client_addr_len);
    if (client_sock_fd == -1) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      }
      return absl::ErrnoToStatus(errno, "accept failed");
    }

    SetBlocking(client_sock_fd);

    MP_RETURN_IF_ERROR(Receive(client_sock_fd));
    MP_RETURN_IF_ERROR(SendAck(client_sock_fd));

    close(client_sock_fd);
  }

  return absl::OkStatus();
}

absl::Status FileDescriptorReceiver::SendAck(int fd) {
  char ack = 0;
  const ssize_t nbytes = send(fd, &ack, sizeof(ack), 0);
  if (nbytes < sizeof(ack)) {
    return absl::InternalError("Did not send enough bytes");
  }
  return absl::OkStatus();
}

absl::Status FileDescriptorReceiver::Receive(int client_fd) {
  IMP_LOG(imp::INFO) << "Receive";
  FileDescriptorMetadata metadata;
  struct msghdr msg = {0};
  struct cmsghdr* cmsg;

  struct iovec iov[1];
  iov[0].iov_base = &metadata;
  iov[0].iov_len = sizeof(FileDescriptorMetadata);
  msg.msg_iov = iov;
  msg.msg_iovlen = 1;

  union {
    char buf[CMSG_SPACE(sizeof(int))];
    struct cmsghdr align;
  } u;
  msg.msg_control = u.buf;
  msg.msg_controllen = sizeof(u.buf);

  // Increase the receive buffer size to 64KB to avoid recvmsg failing on
  // MacOS with EMSGSIZE under heavy load
  const int buffer_size = 64 * 1024;
  setsockopt(client_fd, SOL_SOCKET, SO_RCVBUF, &buffer_size,
             sizeof(buffer_size));

  int received_fd = -1;
  ssize_t nbytes = recvmsg(client_fd, &msg, 0);
  if (nbytes <= 0) {
    return absl::ErrnoToStatus(errno, "recvmsg failed");
  }

  if (nbytes < (ssize_t)sizeof(metadata)) {
    return absl::InternalError("Did not receive enough bytes");
  }

  cmsg = CMSG_FIRSTHDR(&msg);
  if (cmsg != nullptr && cmsg->cmsg_level == SOL_SOCKET &&
      cmsg->cmsg_type == SCM_RIGHTS &&
      cmsg->cmsg_len == CMSG_LEN(sizeof(int))) {
    memcpy(&received_fd, CMSG_DATA(cmsg), sizeof(int));
  } else {
    return absl::InternalError("Did not receive a file descriptor");
  }

  callback_(received_fd, metadata);
  return absl::OkStatus();
}

}  // namespace imp::split_engine
