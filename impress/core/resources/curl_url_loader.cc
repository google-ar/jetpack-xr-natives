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

#include "core/resources/curl_url_loader.h"

#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <queue>
#include <string>
#include <utility>

#include "absl/base/attributes.h"
#include "absl/base/const_init.h"
#include "absl/base/thread_annotations.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/cord.h"
#include "absl/strings/match.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/types/optional.h"
#include "curl/curl.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/async/future_common.h"
#include "core/common/buffer_access.h"
#include "core/common/file_helpers.h"
#include "core/common/robin_map.h"
#include "core/config.h"
#include "core/resources/url_loader.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {
namespace resources {

namespace {

using ReceiveChunkCallback = std::function<void(absl::string_view)>;

constexpr int kPollTimeoutMs = 250;

}  // namespace

class CurlUrlLoader : public UrlLoader {
 public:
  CurlUrlLoader();

  Future<absl::Cord> LoadUrl(const std::string& url) override;

 private:
  // Url to load queued from the foreground thread which will then be consumed &
  // loaded by curl on a background thread.
  struct QueuedUrl {
    // The url being loaded.
    std::string url;
    // The future that the data should be returned to when loading is complete.
    WeakFuture<absl::Cord> result;
  };

  // Tracks information related to the in-flight downloads from Curl.
  struct LoadingUrl {
    // The url being loaded.
    std::string url;
    // The data we've loaded so far from curl for this url.
    absl::Cord in_progress_data;
    // The future that the data should be returned to when loading is complete.
    WeakFuture<absl::Cord> result;
    // The callback used when curl receives more data to append the data to the
    // cord.
    std::unique_ptr<ReceiveChunkCallback> receive_chunk_callback;
  };

  struct DownloadInfo {
    absl::Mutex mu;
    std::queue<QueuedUrl> urls_to_load ABSL_GUARDED_BY(mu);
    bool is_running ABSL_GUARDED_BY(mu) = false;
  };

  absl::StatusOr<absl::Cord> LoadPath(const std::string& path);

  void ScheduleUrlConsumer();

  // Called from background executor.
  static void ConsumeUrls(std::shared_ptr<DownloadInfo> download_info);

  // Kept in shared_ptr so that it stays in memory after the CurlUrlLoader is
  // destroyed and the thread is cleaning up.
  std::shared_ptr<DownloadInfo> download_info_;

  // Keeps the future alive for consuming urls.
  Future<absl::Status> consume_urls_future;
};

CurlUrlLoader::CurlUrlLoader() {
  ABSL_CONST_INIT static absl::Mutex init_mutex(absl::kConstInit);
  static bool curl_initialized = false;
  absl::MutexLock lock(&init_mutex);
  if (!curl_initialized) {
    curl_global_init(CURL_GLOBAL_ALL);
    curl_initialized = true;
  }

  download_info_ = std::make_shared<DownloadInfo>();
}

static size_t WriteCurlChunk(void* buffer, size_t size, size_t nmemb,
                             void* curl_callback) {
  absl::string_view chunk_data = {static_cast<const char*>(buffer), nmemb};
  ReceiveChunkCallback* receive_chunk =
      reinterpret_cast<ReceiveChunkCallback*>(curl_callback);
  (*receive_chunk)(chunk_data);
  return nmemb;
}

Future<absl::Cord> CurlUrlLoader::LoadUrl(const std::string& url) {
  constexpr auto kFileUriPrefix = absl::string_view{"file://"};
  if (absl::StartsWith(url, kFileUriPrefix)) {
    return Future<absl::Cord>(LoadPath(url.substr(kFileUriPrefix.size())));
  }

  bool was_running;
  Future<absl::Cord> result;

  // While locked, enqueue the url to load and determine if we need to start a
  // Future task to consume the urls. We must start a task to consume the urls
  // if no url is already loading at the moment.
  {
    absl::MutexLock lock(&download_info_->mu);
    download_info_->urls_to_load.push({std::string(url), result});

    was_running = download_info_->is_running;
    download_info_->is_running = true;
  }

  if (!was_running) {
    ScheduleUrlConsumer();
  }

  return result;
}

void CurlUrlLoader::ScheduleUrlConsumer() {
  // TODO (broken link) Schedule future with correct future group if there are
  // in-flight requests going on.
  consume_urls_future = Future<absl::Status>::Schedule(
      [download_info = download_info_] {
        ConsumeUrls(download_info);
        return absl::OkStatus();
      },
      {.executor = Executor::Type::kBackground});
}

// TODO: Implement a way to add headers to the request.
void CurlUrlLoader::ConsumeUrls(std::shared_ptr<DownloadInfo> download_info) {
  CURLM* curlm = curl_multi_init();

  int num_remaining_urls_loading = 0;
  bool should_continue = false;
  RobinMap<CURL*, LoadingUrl> loading_urls;

  // While any urls are pending in the queue OR any in-flight downloads are
  // still in progress, contine to process urls with curl.
  do {
    // Grab the pending urls from the queue.
    std::queue<QueuedUrl> urls_to_load;
    {
      absl::MutexLock lock(&download_info->mu);
      std::swap(urls_to_load, download_info->urls_to_load);
    }

    // Start downloading all of the pending urls from curl using curl_multi
    // which allows for simultaneous connections.
    while (!urls_to_load.empty()) {
      QueuedUrl queued_url = urls_to_load.front();
      urls_to_load.pop();

      CURL* curl = curl_easy_init();
      if (!curl) {
        absl::optional<Future<absl::Cord>> result = queued_url.result.Lock();
        if (result.has_value()) {
          result->Return(
              absl::UnavailableError("Failed to initialize Url Connection"));
        }
        continue;
      }

      curl_easy_setopt(curl, CURLOPT_URL, queued_url.url.c_str());
      curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCurlChunk);
#if IMP_PLATFORM(MACOS)
      curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0);
#endif
      curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
      curl_easy_setopt(curl, CURLOPT_USERAGENT, "Impress/1.0");

      LoadingUrl& loading_url =
          loading_urls
              .emplace(curl, LoadingUrl{.url = queued_url.url,
                                        .result = queued_url.result})
              .first.value();

      // Received chunks are appended to the cord.  The cord will be
      // flattened after the callback is complete so when this lambda
      // returns the cord may be unflattened.
      loading_url.receive_chunk_callback =
          std::make_unique<ReceiveChunkCallback>(
              [&loading_urls, curl](absl::string_view chunk) {
                loading_urls.at(curl).in_progress_data.Append(chunk);
              });

      // We pass a ReceiveChunkCallback pointer as the user data which
      // will be passed into the WriteCurlChunk function.  There, this
      // will be cast back to the ReceiveChunkCallback type and called.
      // This is necessary because the curl api is a c-style api, and we
      // are limited this wasy in the data that can be passed through to
      // the callback.
      curl_easy_setopt(
          curl, CURLOPT_WRITEDATA,
          reinterpret_cast<void*>(loading_url.receive_chunk_callback.get()));

      curl_multi_add_handle(curlm, curl);
    }

    // Triggers all the receive_chunk_callback methods to be called with data
    // downloaded using curl.
    CURLMcode mc = curl_multi_perform(curlm, &num_remaining_urls_loading);

    // Detect if curl has finished download a url (success of failure).
    // If so, return the result to the future and clean it up.
    CURLMsg* m = nullptr;
    do {
      int msgq = 0;
      m = curl_multi_info_read(curlm, &msgq);
      if (m && (m->msg == CURLMSG_DONE)) {
        CURL* e = m->easy_handle;
        CURLcode res = m->data.result;
        LoadingUrl& loading_url = loading_urls.at(e);

        absl::optional<Future<absl::Cord>> result = loading_url.result.Lock();
        if (result.has_value()) {
          if (res != CURLE_OK) {
            result->Return(absl::InternalError(
                absl::StrFormat("Curl Error loading '%s': %s", loading_url.url,
                                curl_easy_strerror(res))));
          } else {
            result->Return(loading_url.in_progress_data);
          }
        }

        // Ensure the finished curl request is cleaned up.
        // Notably, CURL* pointers can get re-used by curl so it is important to
        // make sure they get erased.
        loading_urls.erase(e);
        curl_multi_remove_handle(curlm, e);
        curl_easy_cleanup(e);
      }
    } while (m);

    if (mc == CURLM_OK && num_remaining_urls_loading > 0) {
      // Wait until Curl notifies us that there is more data to process. We
      // don't want to wait for too long so that any new requests that have been
      // enqueued can be processed.
      mc = curl_multi_poll(curlm, nullptr, 0, kPollTimeoutMs, nullptr);
    }

    if (mc != CURLM_OK) {
      // Polling failed, but we continue instead of breaking so that we can keep
      // processing requests.
      IMP_LOG(imp::ERROR) << "curl_multi_poll() failed, code " << (int)mc << ".\n";
      continue;
    }

    // Continue looping if any downloads are in-flight or if there are new
    // requests in the queue.
    if (num_remaining_urls_loading == 0) {
      absl::MutexLock lock(&download_info->mu);
      should_continue = !download_info->urls_to_load.empty();
      if (!should_continue) {
        download_info->is_running = false;
      }
    } else {
      should_continue = true;
    }

    /* if there are still transfers, loop! */
  } while (should_continue);

  curl_multi_cleanup(curlm);
}

absl::StatusOr<absl::Cord> CurlUrlLoader::LoadPath(const std::string& path) {
  BufferAccess contents;
  MP_RETURN_IF_ERROR(LoadBinary(path, &contents));
  return absl::Cord{contents.StringView()};
}

std::unique_ptr<UrlLoader> CreateCurlLoader() {
  return std::make_unique<CurlUrlLoader>();
}

}  // namespace resources
}  // namespace imp
