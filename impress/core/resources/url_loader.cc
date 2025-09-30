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

#include "core/resources/url_loader.h"

#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include "absl/status/status.h"
#include "absl/strings/cord.h"
#include "absl/synchronization/mutex.h"
#include "core/async/future.h"
#include "core/async/future_interrupter.h"

namespace imp {
namespace resources {

FutureInterrupter UrlLoader::GetGlobalInterrupter() {
  return global_interrupter_;
}

float UrlLoader::GetDownloadProgress(size_t download_baseline) {
  size_t total_size = 0;
  size_t downloaded_size = 0;
  {
    absl::MutexLock lock(&download_progress_info_->download_progress_map_mutex);
    for (const auto& download_task :
         download_progress_info_->download_progress_map) {
      downloaded_size += download_task.second.downloaded_size;
      size_t task_total_size = download_task.second.total_size;
      if (task_total_size != kUnknownContentLength) {
        total_size += task_total_size;
      } else {
        // When is task total size is unknown, use the downloaded size to give
        // our best estimate.
        total_size += download_task.second.downloaded_size;
      }
    }
  }

  if (total_size <= download_baseline) {
    return 0;
  }
  return ((downloaded_size - download_baseline) * 1.0f) /
         (total_size - download_baseline);
}

float UrlLoader::GetDownloadProgress(std::string_view url) {
  {
    absl::MutexLock lock(&download_progress_info_->download_progress_map_mutex);
    if (auto download_task =
            download_progress_info_->download_progress_map.find(
                std::string(url));
        download_task != download_progress_info_->download_progress_map.end()) {
      size_t downloaded_size = download_task->second.downloaded_size;
      size_t task_total_size = download_task->second.total_size;
      if (task_total_size != kUnknownContentLength && task_total_size != 0) {
        return downloaded_size * 1.0f / task_total_size;
      }
    }
  }
  return 0.0f;
}

size_t UrlLoader::GetDownloadedSize() {
  size_t downloaded_size = 0;
  {
    absl::MutexLock lock(&download_progress_info_->download_progress_map_mutex);
    for (const auto& download_task :
         download_progress_info_->download_progress_map) {
      downloaded_size += download_task.second.downloaded_size;
    }
  }
  return downloaded_size;
}

const UrlLoader::Config& UrlLoader::GetConfig() { return config_; }

void UrlLoader::SetConfig(Config config) { config_ = std::move(config); }

void UrlLoader::Shutdown() { global_interrupter_.Interrupt(); }

}  // namespace resources
}  // namespace imp
