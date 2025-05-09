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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_IPC_TEST_SOCKET_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_IPC_TEST_SOCKET_H_

#include <errno.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstdint>
#include <vector>

#include "core/common/log.h"
#include "core/common/platform_helpers.h"

namespace imp::ipc {

// A utility to control a socket-based service from a test.
// Divides functionality into test-side and remote-side, with remote-side being
// the service being tested.
class TestSocket {
 public:
  TestSocket() { CreateSocketPair(); }

  ~TestSocket() {
    if (fds_[0]) {
      close(fds_[0]);
    }

    if (fds_[1]) {
      close(fds_[1]);
    }
  }

  int RemoteFd() const { return fds_[0]; }

  // The fd for the test side of the socket.
  // Most tests should use the Read/Write functions below instead of addressing
  // the fd directly.
  int TestFd() const { return fds_[1]; }

  // Reads a packet from the test-side of the stream.
  // Returns an empty vector on failure.
  std::vector<uint8_t> ReadPacket() {
    constexpr size_t kHeaderSize = sizeof(uint32_t);
    union {
      uint32_t size;
      uint8_t data[kHeaderSize];
    } packet_size = {};

    // Read the header first to determine the packet size.
    if (!ReadExact(packet_size.data, kHeaderSize)) {
      return {};
    }

    std::vector<uint8_t> result(packet_size.size);
    if (!ReadExact(result.data(), result.size())) {
      return {};
    }

    return result;
  }

  // Read one-shot from the stream, returns any data received.

  // Returns false if the read failed.
  bool ReadExact(uint8_t* data, size_t size) {
    for (size_t offset = 0; offset < size;) {
      const ssize_t bytes_read =
          TEMP_FAILURE_RETRY(read(TestFd(), data + offset, size - offset));
      if (bytes_read <= 0) {
        return false;
      }

      offset += bytes_read;
    }

    return true;
  }

  // Writes a packet to the test-side of the stream.
  // Returns false if the write failed.
  bool WritePacket(const std::vector<uint8_t>& data) {
    const uint32_t size = static_cast<uint32_t>(data.size());
    return WriteRaw(reinterpret_cast<const uint8_t*>(&size),
                    sizeof(uint32_t)) &&
           WriteRaw(data.data(), data.size());
  }

  // Returns false if the write failed.
  bool WriteRaw(const uint8_t* data, size_t size) {
    size_t offset = 0;

    while (offset < size) {
      const ssize_t bytes_written =
          TEMP_FAILURE_RETRY(write(TestFd(), data + offset, size - offset));
      if (bytes_written > 0) {
        offset += bytes_written;
      } else {
        return false;
      }
    }

    return true;
  }

  // Close the pipe from the test side.
  void Close() {
    if (fds_[1] != 0) {
      close(fds_[1]);
      fds_[1] = 0;
    }
  }

 private:
  void CreateSocketPair() {
    if (socketpair(AF_UNIX, SOCK_STREAM, 0, fds_) != 0) {
      IMP_LOG(imp::FATAL) << "Failed to create socket";
    }
  }

  int fds_[2] = {};
};

}  // namespace imp::ipc

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_IPC_TEST_SOCKET_H_
