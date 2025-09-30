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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_UTILS_SPLIT_ENGINE_DESKTOP_BRIDGE_EPOLLER_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_UTILS_SPLIT_ENGINE_DESKTOP_BRIDGE_EPOLLER_H_

#include <array>
#include <cerrno>
#include <cstddef>
#include <utility>

#if defined(__linux__)
#include <sys/epoll.h>
#elif defined(__APPLE__)
#include <sys/event.h>
#include <sys/time.h>
#else
static_assert(false, "Unsupported platform");
#endif
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "absl/cleanup/cleanup.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"

namespace imp::split_engine {

// Simple epoll wrapper since boost::asyncio is not available
class EpollBase {
 public:
  EpollBase() = default;
  virtual ~EpollBase() = default;
  virtual absl::Status Setup(int socket_fd, int shutdown_pipe_fd) = 0;
  virtual absl::StatusOr<int> Poll() = 0;
  virtual absl::StatusOr<int> GetFd(int index) = 0;
};

#if defined(__linux__)
template <size_t kMaxEvents>
class EpollImpl : public EpollBase {
 public:
  ~EpollImpl() override {
    if (epoll_fd_ != -1) {
      close(epoll_fd_);
    }
  }

  absl::Status Setup(int socket_fd, int shutdown_pipe_fd) override {
    epoll_fd_ = epoll_create1(0);
    if (epoll_fd_ == -1) {
      return absl::ErrnoToStatus(errno, "epoll_create1 failed");
    }

    absl::Cleanup epoll_close = [this] {
      close(epoll_fd_);
      epoll_fd_ = -1;
    };

    struct epoll_event event;
    event.events = EPOLLIN | EPOLLET;
    event.data.fd = socket_fd;
    if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, socket_fd, &event) == -1) {
      return absl::ErrnoToStatus(errno, "epoll_ctl ADD failed");
    }

    event.events = EPOLLIN | EPOLLET;
    event.data.fd = shutdown_pipe_fd;
    if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, shutdown_pipe_fd, &event) == -1) {
      return absl::ErrnoToStatus(errno,
                                 "epoll_ctl ADD failed for shutdown_pipe");
    }

    std::move(epoll_close).Cancel();
    return absl::OkStatus();
  }
  absl::StatusOr<int> Poll() override {
    if (epoll_fd_ == -1) {
      return absl::InternalError("epoll_fd_ is not initialized");
    }
    events_.fill({});
    const int result =
        epoll_wait(epoll_fd_, events_.data(), events_.size(), 1000);
    if (result == -1) {
      if (errno == EINTR) {
        return 0;
      }
      return absl::ErrnoToStatus(errno, "epoll_wait failed");
    }
    return result;
  }

  absl::StatusOr<int> GetFd(int index) override {
    if (index >= events_.size()) {
      return absl::OutOfRangeError("Index is out of range");
    }
    return events_[index].data.fd;
  }

  int epoll_fd_ = -1;
  std::array<struct epoll_event, kMaxEvents> events_;
};
#endif

#if defined(__APPLE__)
template <size_t kMaxEvents>
class EpollImpl : public EpollBase {
 public:
  ~EpollImpl() override {
    if (kq_fd_ != -1) {
      close(kq_fd_);
    }
  }

  absl::Status Setup(int socket_fd, int shutdown_pipe_fd) override {
    kq_fd_ = kqueue();
    if (kq_fd_ == -1) {
      return absl::ErrnoToStatus(errno, "kqueue failed");
    }
    absl::Cleanup epoll_close = [this] {
      close(kq_fd_);
      kq_fd_ = -1;
    };

    struct kevent change_event;
    EV_SET(&change_event, socket_fd, EVFILT_READ, EV_ADD | EV_ENABLE | EV_CLEAR,
           0, 0, NULL);
    if (kevent(kq_fd_, &change_event, 1, NULL, 0, NULL) == -1) {
      return absl::ErrnoToStatus(errno, "kevent failed");
    }
    EV_SET(&change_event, shutdown_pipe_fd, EVFILT_READ,
           EV_ADD | EV_ENABLE | EV_CLEAR, 0, 0, NULL);
    if (kevent(kq_fd_, &change_event, 1, NULL, 0, NULL) == -1) {
      return absl::ErrnoToStatus(errno, "kevent failed for shutdown_pipe");
    }

    std::move(epoll_close).Cancel();
    return absl::OkStatus();
  }

  absl::StatusOr<int> Poll() override {
    if (kq_fd_ == -1) {
      return absl::InternalError("kq_fd_ is not initialized");
    }
    events_.fill({});
    struct timespec timeout{};
    timeout.tv_sec = 1;  // seconds

    const int result =
        kevent(kq_fd_, nullptr, 0, events_.data(), events_.size(), &timeout);
    if (result == -1) {
      if (errno == EINTR) {
        return 0;
      }
      return absl::ErrnoToStatus(errno, "kevent failed");
    }
    return result;
  }

  absl::StatusOr<int> GetFd(int index) override {
    if (index >= events_.size()) {
      return absl::OutOfRangeError("Index is out of range");
    }
    const int event_flags = events_[index].flags;
    if (event_flags & EV_ERROR) {
      return absl::ErrnoToStatus(events_[index].data, "kevent error");
    }

    return events_[index].ident;
  }

  int kq_fd_ = -1;
  std::array<struct kevent, kMaxEvents> events_;
};
#endif

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_UTILS_SPLIT_ENGINE_DESKTOP_BRIDGE_EPOLLER_H_
