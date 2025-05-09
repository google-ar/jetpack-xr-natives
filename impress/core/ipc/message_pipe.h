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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_IPC_MESSAGE_PIPE_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_IPC_MESSAGE_PIPE_H_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "zetasql/base/types.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/types/optional.h"
#include "core/common/platform_helpers.h"
#include "thread/thread.h"

namespace imp::ipc {

class WorkerThread;

/**
 * A MessagePipe is used on both the 'client' (i.e. main app) and 'service'
 * (i.e. isolated process) in the Android loader to facilitate inter-process
 * communication. Each pipe holds onto one of a pair of file handles created via
 * e.g. createSocketPair() on the java side.
 *
 * Clients of a message pipe define two methods, both of which are called on the
 * worker thread internal to the pipe.  The OnMessageCallback is called when the
 * message pipe encounters new incoming data.  After reacting to it (by either
 * performing local work, or by e.g. responding via .Send(), the caller can tell
 * the MessagePipe to either keep the connection alive, or initiate closing. The
 * OnCloseCallback is called before the worker thread exits and the main thread
 * cleans up.
 */
class MessagePipe {
 public:
  enum class OnMessageResult {
    kKeepAlive,
    kInitiateClose,
  };
  using OnMessageCallback = OnMessageResult (*)(std::unique_ptr<uint8_t[]> data,
                                                size_t size, void* user);

  using OnCloseCallback = void (*)(void* user);

  // Create a message pipe connected to a specific fd.
  MessagePipe(int fd, OnMessageCallback message_callback,
              OnCloseCallback close_callback, void* user,
              absl::string_view name);

  // No copy.
  MessagePipe(const MessagePipe&) = delete;
  MessagePipe& operator=(const MessagePipe&) = delete;

  ~MessagePipe();

  // Close the pipe and cancel any pending operations.
  void Close();

  // Send a message to the host.
  bool Send(const uint8_t* data, uint32_t data_size);

  // Returns true if the pipe is closed.
  bool IsClosed();

 private:
  // Helper for the self-pipe trick, to wake the worker thread when we want to
  // destruct, since read() may not wake if we close its fd.
  // (broken link)
  class NotifyPipe {
   public:
    NotifyPipe();
    ~NotifyPipe();

    void Notify();
    int GetReceiveFd() { return fds_[kFdReceive]; }

   private:
    static constexpr size_t kFdReceive = 0;
    static constexpr size_t kFdSend = 1;
    int fds_[2];
  };

  /**
   * Manages the private worker thread used by MessagePipe.  The worker thread
   * blocks by requesting the next message from its pipe.  The NotifyPipe is
   * used as a secondary pipe (WorkerThread will respond to bytes being present
   * in either file) which allows the main message pipe thread to initiate
   * termination of the worker thread when necessary.
   */
  class WorkerThread : public Thread {
   public:
    explicit WorkerThread(MessagePipe* pipe);

   protected:
    void Run() override;

   private:
    MessagePipe* pipe_;
  };

  void WorkerThreadMain();

  bool ReadPacketInternal(std::unique_ptr<uint8_t[]>* out_result,
                          size_t* out_size);

  bool ReadInternal(const size_t bytes, uint8_t* out_data);
  bool WriteInternal(const uint8_t* data, size_t data_size);

  void CreateWorkerThread();

  const int fd_;

  absl::Mutex lock_;
  std::atomic<bool> closed_ = ATOMIC_FLAG_INIT;
  NotifyPipe close_notifier_;
  uint32 worker_thread_id_;

  absl::optional<WorkerThread> worker_thread_;

  const OnMessageCallback message_callback_;
  const OnCloseCallback close_callback_;
  void* user_;
  std::string name_;
};

}  // namespace imp::ipc

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_IPC_MESSAGE_PIPE_H_
