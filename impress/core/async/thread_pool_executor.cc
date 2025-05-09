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

#include "core/async/thread_pool_executor.h"

#include <algorithm>
#include <cassert>
#include <optional>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "core/async/executor.h"
#include "core/async/task.h"
#include "core/async/task_scheduler.h"
#include "core/common/invocable.h"
#include "core/common/platform_helpers.h"
#include "core/config.h"


#if IMP_PLATFORM(IOS)
#include <sys/sysctl.h>

#include <memory>
#endif

namespace imp {

namespace {

// When computing the number of available cores, remove two (one for the main
// thread and one for the filament thread) to avoid core contention.
constexpr int kBuiltinThreadCount = 2;
constexpr int kDefaultNumWorkerThreads = 2;
// On two-core systems we still need to make at least one worker thread.
// TODO reduce this back to 1 after GltfAssetLoader::Load no longer
// blocks a worker thread waiting for other worker threads to complete.
constexpr int kMinimumWorkerThreadCount = 2;
// Don't be silly on e.g. linux machines with cores by the score.
constexpr int kMaximumWorkerThreadCount = 6;
// Go with the android default for background processing; reference:
// (broken link)
constexpr int kWorkerThreadNicenessOffset = 10;
constexpr char kThreadPrefix[] = "imp_bg_wk";

int GetWorkerThreadCount() {
  // TODO move this into common/ and make it cross-platform.
#if IMP_PLATFORM(IOS)
  unsigned int ncpu;
  size_t len = sizeof(ncpu);
  sysctlbyname("hw.ncpu", &ncpu, &len, nullptr, 0);
  const int ideal_worker_thread_count =
      static_cast<int>(ncpu) - kBuiltinThreadCount;
  return std::max(ideal_worker_thread_count, kMinimumWorkerThreadCount);
#else
  // We max these together to avoid ClangTidy warnings for the constants when
  // compiled on other platforms.
  return std::min(
      std::max(std::max(kMinimumWorkerThreadCount, kBuiltinThreadCount),
               kDefaultNumWorkerThreads),
      kMaximumWorkerThreadCount);
#endif
}

}  // namespace

struct ThreadPoolGlobals {
  ThreadPoolThreadBegin thread_begin = []() {};
  ThreadPoolThreadEnd thread_end = []() {};
};

ThreadPoolGlobals &GetThreadPoolGlobals() {
  static ThreadPoolGlobals *globals = new ThreadPoolGlobals();
  return *globals;
}

void SetCallbacksForThreadPool(ThreadPoolThreadBegin thread_begin,
                               ThreadPoolThreadEnd thread_end) {
  ThreadPoolGlobals &globals = GetThreadPoolGlobals();
  globals.thread_begin = std::move(thread_begin);
  globals.thread_end = std::move(thread_end);
  IMP_LOG(imp::INFO) << "Assigned ThreadPool callbacks";
}

void ClearCallbacksForThreadPool() {
  ThreadPoolGlobals &globals = GetThreadPoolGlobals();
  globals.thread_begin = []() {};
  globals.thread_end = []() {};
  IMP_LOG(imp::INFO) << "Cleared ThreadPool callbacks";
}

class ThreadBookend {
 public:
  ThreadBookend() {
    ThreadPoolGlobals &globals = GetThreadPoolGlobals();
    globals.thread_begin();
  }
  ~ThreadBookend() {
    ThreadPoolGlobals &globals = GetThreadPoolGlobals();
    globals.thread_end();
  }
};

#if IMP_THREADS(GOOGLE3)
ThreadPoolExecutor::WorkerThread::WorkerThread(
    ThreadPoolExecutor *thread_pool_executor, Executor *foreground_executor)
    : Thread(thread::Options().set_joinable(true).set_nice_priority_level(
                 kWorkerThreadNicenessOffset),
             kThreadPrefix),
      thread_pool_executor_(thread_pool_executor),
      foreground_executor_(foreground_executor) {}

void ThreadPoolExecutor::WorkerThread::Run() {
  // Associates the foreground and background executors with this thread.
  Executor::SetForegroundExecutor(foreground_executor_);
  Executor::SetBackgroundExecutor(thread_pool_executor_);
  ThreadBookend destroyed_on_scope_exit;
  while (thread_pool_executor_->ProcessNextRequest()) {
    thread_pool_executor_->wait_condvar_.SignalAll();
  }
}
#endif

ThreadPoolExecutor::ThreadPoolExecutor(Executor *foreground_executor)
    : finished_(false), callback_counter_(0) {
  task_scheduler_ = std::make_unique<TaskScheduler>();
  const int worker_thread_count = GetWorkerThreadCount();

  worker_threads_.reserve(worker_thread_count);
  for (int i = 0; i < worker_thread_count; ++i) {
#if IMP_THREADS(GOOGLE3)
    worker_threads_.push_back(
        std::make_unique<WorkerThread>(this, foreground_executor));
    worker_threads_.back()->Start();
#elif IMP_THREADS(STDLIB)
    worker_threads_.emplace_back([this, foreground_executor]() {
      SetThreadName(kThreadPrefix);
      SetThreadNiceness(GetThreadId(), kWorkerThreadNicenessOffset);
      // Associates the foreground and background executors with this thread.
      Executor::SetForegroundExecutor(foreground_executor);
      Executor::SetBackgroundExecutor(this);

      ThreadBookend destroyed_on_scope_exit;
      while (ProcessNextRequest()) {
        wait_condvar_.SignalAll();
      }
    });
#else
#error Invalid thread mode.
#endif
  }
}

ThreadPoolExecutor::~ThreadPoolExecutor() { Shutdown(); }

std::optional<Invocable<void()>> ThreadPoolExecutor::WaitForTask() {
  absl::MutexLock lock(&mu_);

  while (task_scheduler_->IsEmpty() && !finished_) {
    condvar_.Wait(&mu_);
  }
  assert(finished_ || !task_scheduler_->IsEmpty());

  if (finished_) return {};

  Invocable<void()> callback = task_scheduler_->PopTask();

  callback_counter_++;
  return callback;
}

bool ThreadPoolExecutor::ProcessNextRequest() {
  std::optional<Invocable<void()>> task_maybe = WaitForTask();
  if (!task_maybe) {
    // No task == stop requested.
    return false;
  }

  auto task = std::move(task_maybe.value());

  Executor::SetCurrentExecutor(this);
  task();
  Executor::SetCurrentExecutor(nullptr);

  {
    absl::MutexLock lock(&mu_);
    callback_counter_--;
  }

  return true;
}

TaskId ThreadPoolExecutor::ScheduleInvocable(Invocable<void()> invocable,
                                             int task_priority) {
  {
    absl::MutexLock lock(&mu_);
    if (finished_) {
      return kInvalidTaskId;
    }
  }

  TaskId task_id;
  {
    absl::MutexLock lock(&mu_);
    task_id = task_scheduler_->PushTask(std::move(invocable), task_priority);
  }
  condvar_.Signal();
  return task_id;
}

TaskId ThreadPoolExecutor::ReserveTaskId() {
  absl::MutexLock lock(&mu_);
  if (finished_) {
    return kInvalidTaskId;
  }
  return task_scheduler_->ReserveTaskId();
}

bool ThreadPoolExecutor::ScheduleWithReservedTaskId(
    TaskId reserved_task_id, imp::Invocable<void()> function,
    int task_priority) {
  {
    absl::MutexLock lock(&mu_);
    if (finished_) {
      return false;
    }
    task_scheduler_->PushWithReservedTaskId(reserved_task_id,
                                            std::move(function), task_priority);
  }
  condvar_.Signal();
  return true;
}

absl::Status ThreadPoolExecutor::UpdateTaskPriority(TaskId task_id,
                                                    int task_priority) {
  absl::MutexLock lock(&mu_);
  if (finished_) {
    return absl::OkStatus();
  }
  return task_scheduler_->RescheduleTask(task_id, task_priority);
}

absl::StatusOr<int> ThreadPoolExecutor::GetTaskPriority(TaskId task_id) {
  absl::MutexLock lock(&mu_);
  if (finished_) {
    return absl::FailedPreconditionError("Executor has shutdown.");
  }
  return task_scheduler_->GetTaskPriority(task_id);
}

void ThreadPoolExecutor::Shutdown() {
  std::vector<imp::Invocable<void()>> tasks;
  {
    absl::MutexLock lock(&mu_);
    if (finished_) {
      return;
    }
    while (!task_scheduler_->IsEmpty()) {
      tasks.push_back(task_scheduler_->PopTask());
    }
    finished_ = true;
    condvar_.SignalAll();
  }

  IMP_LOG(imp::INFO) << "Shutting down thread pool";
  for (auto &worker_thread : worker_threads_) {
#if IMP_THREADS(GOOGLE3)
    worker_thread->Join();
#elif IMP_THREADS(STDLIB)
    worker_thread.join();
#else
#error Invalid thread mode.
#endif
  }
  worker_threads_.clear();
  // Destroy all tasks outside of MutexLock to avoid invoking destructors
  // that could lead to deadlocks.
  // TODO (broken link) Determine how draining tasks during shutdown should be
  // handled
  tasks.clear();
}

bool ThreadPoolExecutor::Pump(bool drain) {
  if (Executor::CurrentExecutor() == this) {
    IMP_LOG(imp::FATAL) << "Cannot Pump the ThreadPoolExecutor from within a "
                  "ThreadPoolExecutor thread.";
    return false;
  }

  if (drain == false) {
    // It would be possible to add support for this, but we don't have a need
    // so not implementing it for the time being.
    IMP_LOG(imp::WARNING) << "Not supported to Pump only a single task using the "
                    "ThreadPoolExecutor.";
    return false;
  }

  {
    // Check if any tasks are scheduled.
    absl::MutexLock lock(&mu_);
    if (task_scheduler_->IsEmpty() && callback_counter_ == 0) {
      return false;
    }
  }

  WaitUntilDrained();
  return true;
}

void ThreadPoolExecutor::WaitUntilDrained() {
  absl::MutexLock lock(&mu_);
  while (!task_scheduler_->IsEmpty() || callback_counter_ != 0) {
    wait_condvar_.Wait(&mu_);
  }
}

bool ThreadPoolExecutor::IsPumpingRequired() { return false; }

}  // namespace imp
