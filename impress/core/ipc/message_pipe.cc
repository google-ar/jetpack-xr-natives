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

#include "core/ipc/message_pipe.h"

#include <errno.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <cstdlib>
#include <memory>
#include <utility>

#include "core/common/log.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "core/common/platform_helpers.h"
#include "thread/thread.h"
#include "thread/thread_options.h"

namespace imp::ipc {

namespace {
constexpr char kThreadPrefix[] = "imp_mpipe";
}  // namespace

MessagePipe::MessagePipe(int fd, OnMessageCallback message_callback,
                         OnCloseCallback close_callback, void* user,
                         absl::string_view name)
    : fd_(fd),
      worker_thread_id_(0),
      message_callback_(message_callback),
      close_callback_(close_callback),
      user_(user),
      name_(!name.empty() ? name : absl::string_view("UnnamedPipe")) {
  fcntl(fd, F_SETFL, O_NONBLOCK);
  // Create the worker thread last, after initializing member variables.
  CreateWorkerThread();
}

MessagePipe::~MessagePipe() {
  if (GetThreadId() == worker_thread_id_) {
    IMP_LOG(imp::FATAL) << name_ << " destructed from worker thread";
  }
  Close();
}

void MessagePipe::CreateWorkerThread() {
  absl::MutexLock lock(&lock_);
  worker_thread_.emplace(this);
  worker_thread_->RegisterExitHandler([this]() {
    IMP_LOG(imp::ERROR) << "MessagePipe '" << name_
               << "' worker thread exited abnormally";
  });
  worker_thread_->Start();
  // Block until the worker thread has started.
  auto cond = [this] { return worker_thread_id_ != 0; };
  lock_.Await(absl::Condition(&cond));
}

void MessagePipe::Close() {
  // Also marks the pipe as closed.
  const bool is_closed = closed_.exchange(true);
  if (is_closed && worker_thread_) {
    // Re-entrant close.
    IMP_LOG(imp::FATAL) << name_ << " was re-entrantly closed";
    return;
  } else if (is_closed) {
    // Double-close.
    return;
  }

  absl::MutexLock lock(&lock_);
  close_notifier_.Notify();

  if (worker_thread_) {
    worker_thread_->Join();
    worker_thread_.reset();
  }

  shutdown(fd_, SHUT_RDWR);
  close(fd_);
}

bool MessagePipe::Send(const uint8_t* data, uint32_t data_size) {
  if (!WriteInternal(reinterpret_cast<const uint8_t*>(&data_size),
                     sizeof(data_size))) {
    return false;
  }

  if (!WriteInternal(data, data_size)) {
    return false;
  }

  return true;
}

bool MessagePipe::IsClosed() { return closed_.load(); }

void MessagePipe::WorkerThreadMain() {
  {
    absl::MutexLock lock(&lock_);
    worker_thread_id_ = GetThreadId();
  }
  OnMessageResult last_result = OnMessageResult::kKeepAlive;
  while (!IsClosed() && (last_result == OnMessageResult::kKeepAlive)) {
    std::unique_ptr<uint8_t[]> packet;
    size_t size;
    if (!ReadPacketInternal(&packet, &size)) {
      break;
    }

    last_result = message_callback_(std::move(packet), size, user_);
  }
  if (close_callback_) {
    close_callback_(user_);
  }
}

bool MessagePipe::ReadPacketInternal(std::unique_ptr<uint8_t[]>* out_result,
                                     size_t* out_size) {
  out_result->reset();
  *out_size = 0;

  uint32_t data_size = 0;
  if (!ReadInternal(sizeof(data_size),
                    reinterpret_cast<uint8_t*>(&data_size))) {
    return false;
  }

  auto result = std::make_unique<uint8_t[]>(static_cast<size_t>(data_size));
  if (data_size) {
    if (!ReadInternal(data_size, result.get())) {
      return false;
    }
  }

  *out_result = std::move(result);
  *out_size = data_size;
  return true;
}

bool MessagePipe::ReadInternal(const size_t bytes, uint8_t* data) {
  uint8_t* data_current = data;
  uint8_t* data_end = data + bytes;

  fd_set read_fds;

  while (data_current < data_end) {
    FD_ZERO(&read_fds);
    FD_SET(fd_, &read_fds);
    FD_SET(close_notifier_.GetReceiveFd(), &read_fds);

    const int result = TEMP_FAILURE_RETRY(
        select(FD_SETSIZE, &read_fds, nullptr, nullptr, nullptr));
    if (result <= 0) {
      IMP_LOG(imp::ERROR) << name_ << " select failed";
      return false;
    }

    if (FD_ISSET(close_notifier_.GetReceiveFd(), &read_fds)) {
      IMP_LOG(imp::INFO) << name_ << " read canceled";
      return false;
    }

    const size_t bytes_remaining = data_end - data_current;
    const ssize_t bytes_read =
        TEMP_FAILURE_RETRY(read(fd_, data_current, bytes_remaining));
    if (bytes_read == 0) {
      IMP_LOG(imp::ERROR) << name_ << " failed to read " << bytes_remaining << " of "
                 << bytes << " bytes, read " << bytes_read << ", pipe closed";
      return false;
    } else if (bytes_read < 0) {
      if (errno == EAGAIN) {
        continue;
      }

      IMP_LOG(imp::ERROR) << name_ << " failed to read " << bytes_remaining << " of "
                 << bytes << " bytes, errno " << errno << " , pipe closed";
      return false;
    }

    data_current += bytes_read;
  }

  return true;
}

bool MessagePipe::WriteInternal(const uint8_t* data, size_t data_size) {
  const uint8_t* data_current = data;
  const uint8_t* data_end = data + data_size;

  fd_set write_fds;

  while (data_current < data_end) {
    FD_ZERO(&write_fds);
    FD_SET(fd_, &write_fds);

    const int result = TEMP_FAILURE_RETRY(
        select(FD_SETSIZE, nullptr, &write_fds, nullptr, nullptr));
    if (result <= 0) {
      IMP_LOG(imp::ERROR) << "Select failed";
      return false;
    }

    const size_t bytes_remaining = data_end - data_current;
    const ssize_t bytes_written =
        TEMP_FAILURE_RETRY(write(fd_, data_current, bytes_remaining));
    if (bytes_written == -1) {
      if (errno == EAGAIN) {
        continue;
      }

      IMP_LOG(imp::ERROR) << name_ << " failed to write " << bytes_remaining << " of "
                 << data_size << " bytes, wrote " << bytes_written
                 << ", pipe closed";

      return false;
    }

    data_current += bytes_written;
  }

  return true;
}

// Helper for the self-pipe trick, to wake the worker thread when we want to
// destruct, since read() may not wake if we close its fd.
// (broken link)
MessagePipe::NotifyPipe::NotifyPipe() {
  const int result = pipe(fds_);
  if (result != 0) {
    IMP_LOG(imp::FATAL) << "Failed to create pipe";
  }
  fcntl(fds_[kFdReceive], F_SETFL, O_NONBLOCK);
}

MessagePipe::NotifyPipe::~NotifyPipe() {
  if (fds_[0]) {
    close(fds_[0]);
  }

  if (fds_[1]) {
    close(fds_[1]);
  }
}

void MessagePipe::NotifyPipe::Notify() {
  const uint8_t kNotifyByte = 1;
  TEMP_FAILURE_RETRY(write(fds_[kFdSend], &kNotifyByte, sizeof(kNotifyByte)));
}

MessagePipe::WorkerThread::WorkerThread(MessagePipe* pipe)
    : Thread(thread::Options().set_joinable(true).set_nice_priority_level(5),
             kThreadPrefix),
      pipe_(pipe) {}

void MessagePipe::WorkerThread::Run() { pipe_->WorkerThreadMain(); }

}  // namespace imp::ipc
