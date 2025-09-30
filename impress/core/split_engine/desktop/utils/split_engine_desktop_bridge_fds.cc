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
#include <cstring>
#include <string_view>

#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "core/split_engine/desktop/utils/split_engine_desktop_bridge_uds.h"
#include "core/split_engine/desktop/utils/split_engine_desktop_bridge_utils.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::split_engine {

FileDescriptorSender::FileDescriptorSender(const std::string_view uds_path)
    : UnixDomainSocket(uds_path, false) {}

absl::Status FileDescriptorSender::Send(int fd,
                                        FileDescriptorMetadata& metadata) {
  absl::MutexLock lock(mutex_);
  MP_RETURN_IF_ERROR(Connect());
  MP_RETURN_IF_ERROR(SendImpl(fd, metadata));
  MP_RETURN_IF_ERROR(ReceiveAck());
  MP_RETURN_IF_ERROR(Disconnect());
  return absl::OkStatus();
}

absl::Status FileDescriptorSender::Connect() {
  if (connected_) {
    return absl::OkStatus();
  }

  struct sockaddr_un addr;
  memset(&addr, 0, sizeof(addr));
  addr.sun_family = AF_UNIX;
  strncpy(addr.sun_path, GetUdsPath().data(), sizeof(addr.sun_path) - 1);
  if (connect(GetSocketFd(), (struct sockaddr*)&addr, sizeof(addr)) == -1) {
    return absl::ErrnoToStatus(errno, "connect failed");
  }

  connected_ = true;
  return absl::OkStatus();
}

absl::Status FileDescriptorSender::Disconnect() {
  if (!connected_) {
    return absl::OkStatus();
  }

  close(GetSocketFd());

  connected_ = false;
  return absl::OkStatus();
}

absl::Status FileDescriptorSender::SendImpl(int fd,
                                            FileDescriptorMetadata& metadata) {
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

  cmsg = CMSG_FIRSTHDR(&msg);
  if (cmsg == nullptr) {
    return absl::InternalError("CMSG_FIRSTHDR returned nullptr");
  }
  cmsg->cmsg_level = SOL_SOCKET;
  cmsg->cmsg_type = SCM_RIGHTS;
  cmsg->cmsg_len = CMSG_LEN(sizeof(int));
  memcpy(CMSG_DATA(cmsg), &fd, sizeof(int));

  if (sendmsg(GetSocketFd(), &msg, 0) < 0) {
    return absl::ErrnoToStatus(errno, "sendmsg failed");
  }

  return absl::OkStatus();
}

absl::Status FileDescriptorSender::ReceiveAck() {
  char ack = 0;
  const ssize_t nbytes = recv(GetSocketFd(), &ack, sizeof(ack), 0);
  if (nbytes < sizeof(ack)) {
    return absl::ErrnoToStatus(errno, "read failed");
  }
  return absl::OkStatus();
}

}  // namespace imp::split_engine
