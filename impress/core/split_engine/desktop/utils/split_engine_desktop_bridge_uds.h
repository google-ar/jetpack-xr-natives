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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_UTILS_SPLIT_ENGINE_DESKTOP_BRIDGE_UDS_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_UTILS_SPLIT_ENGINE_DESKTOP_BRIDGE_UDS_H_

#include <fcntl.h>
#include <sys/socket.h>

#include <cerrno>
#include <cstring>
#include <string>
#include <string_view>

#include "absl/log/check.h"

namespace imp::split_engine {

// Wrapper around Unix domain socket.
class UnixDomainSocket {
 public:
  // `uds_path`: The path to the Unix Domain Socket (UDS) to connect to.
  // `uds_path_owned`: if true, this class will take over the ownership of the
  // UDS path and unlink it when the socket is destroyed.
  UnixDomainSocket(const std::string_view uds_path, bool uds_path_owned = true)
      : uds_path_(uds_path),
        uds_path_owned_(uds_path_owned),
        socket_fd_(socket(AF_UNIX, SOCK_STREAM, 0)) {
    
  }

  UnixDomainSocket(const UnixDomainSocket&) = delete;
  UnixDomainSocket& operator=(const UnixDomainSocket&) = delete;

  ~UnixDomainSocket() {
    close(socket_fd_);
    if (uds_path_owned_) {
      unlink(uds_path_.c_str());
    }
  }

 protected:
  int GetSocketFd() const { return socket_fd_; }
  std::string_view GetUdsPath() const { return uds_path_; }
  void SetBlocking(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    
    
  }
  void SetNonBlocking(int fd) {
    const int flags = fcntl(fd, F_GETFL, 0);
    
    
  }

 private:
  const std::string uds_path_;
  const bool uds_path_owned_;
  const int socket_fd_;
};
}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_UTILS_SPLIT_ENGINE_DESKTOP_BRIDGE_UDS_H_
