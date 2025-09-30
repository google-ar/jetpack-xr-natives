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

#include "core/resources/ios_url_loader.h"

#include "third_party/absl/base/attributes.h"
#include "third_party/absl/base/const_init.h"
#include "third_party/absl/memory/memory.h"
#include "third_party/absl/strings/cord_buffer.h"
#include "third_party/absl/types/span.h"
#include "core/common/buffer_access.h"
#include "core/monitor/profiling_clock.h"
#include "core/resources/url_loader.h"

#import <Foundation/Foundation.h>

#include <memory>
#include <string>
#include <utility>

NS_ASSUME_NONNULL_BEGIN

@class IMPURLSessionManager;

namespace imp::resources {
namespace {
// The cadence of worker threads grabbing a mutex to update progress isn't known ahead of time; this
// limit ensures that it doesn't happen any faster than 60 times a second.
constexpr auto kProgressUpdateIntervalLimit = absl::Milliseconds(16);

class IosUrlLoader : public UrlLoader {
 public:
  IosUrlLoader();
  ~IosUrlLoader() override;

  Future<absl::Cord> LoadUrl(const std::string &url) override;

  void Shutdown() override;

  void SetExpectedSize(const std::string &string_url, size_t expected_size);
  void SetFinalSize(const std::string &string_url, size_t final_size);
  bool TryAddData(const std::string &string_url, size_t downloaded_size);

 private:
  NSURLSession *url_session_;
  IMPURLSessionManager *session_manager_;
  absl::Time last_progress_update_time_;
};
}  // namespace
}  // namespace imp::resources

@interface IMPURLTaskData : NSObject
@property(nonatomic) NSUInteger expectedSize;
@end

@implementation IMPURLTaskData {
  absl::optional<imp::WeakFuture<absl::Cord>> _contentFuture;
  imp::FutureInterrupter _interrupter;
  NSMutableArray<NSData *> *_accumulatedData;
  imp::resources::IosUrlLoader *_nativeLoader;
  std::string _URLCppString;
  NSUInteger _deferredSize;
  NSUInteger _expectedSize;
}

- (void)setExpectedSize:(NSUInteger)expectedSize {
  _expectedSize = expectedSize;
  _nativeLoader->SetExpectedSize(_URLCppString, expectedSize);
}

- (instancetype)initForURLString:(NSString *)URLString
                withNativeLoader:(imp::resources::IosUrlLoader *)nativeLoader {
  self = [super init];
  if (self) {
    _accumulatedData = [[NSMutableArray alloc] init];
    _nativeLoader = nativeLoader;
    _URLCppString = std::string([URLString cString]);
  }
  return self;
}

- (imp::Future<absl::Cord>)createFuture {
  imp::Future<absl::Cord> result;
  _contentFuture.emplace(imp::WeakFuture<absl::Cord>(result));
  return _interrupter.MakeInterruptible(result);
}

- (void)addData:(NSData *)data {
  [_accumulatedData addObject:data];
  if (!_nativeLoader) return;
  if (_nativeLoader->TryAddData(_URLCppString, data.length + _deferredSize)) {
    _deferredSize = 0;
  } else {
    _deferredSize += data.length;
  }
}

- (BOOL)isInterrupted:(imp::FutureInterrupter)globalInterrupter {
  return _interrupter.IsInterrupted() || globalInterrupter.IsInterrupted();
}

- (void)completeFuture:(nullable NSError *)error {
  auto maybe_future = _contentFuture.value().Lock();
  if (!maybe_future || maybe_future->Ready()) {
    return;
  }
  if (error != nil) {
    maybe_future->Return(
        absl::InternalError(std::string([[error localizedDescription] UTF8String])));
  } else {
    NSUInteger totalSize = 0;
    for (NSData *data in _accumulatedData) totalSize += [data length];
    _nativeLoader->SetFinalSize(_URLCppString, totalSize);
    imp::BufferAccess *nativeCopy = new imp::BufferAccess();
    __block uint8_t *cursor = imp::BufferAccess::Create(static_cast<size_t>(totalSize), nativeCopy);
    absl::Cord result;
    for (NSData *data in _accumulatedData) {
      [data enumerateByteRangesUsingBlock:^(const void *bytes, NSRange byteRange, BOOL *stop) {
        memcpy(cursor, bytes, byteRange.length);
        cursor += byteRange.length;
      }];
    }

    result.AppendExternalMemory(nativeCopy->StringView(), nativeCopy, [](void *arg) {
      imp::BufferAccess *instance = reinterpret_cast<imp::BufferAccess *>(arg);
      delete instance;
    });
    maybe_future->Return(result);
  }
}

- (void)cancel {
  _nativeLoader = nil;
  auto maybe_future = _contentFuture.value().Lock();
  if (!maybe_future || maybe_future->Ready()) {
    return;
  }
  maybe_future->Return(absl::CancelledError("Interrupted before completing download."));
}

@end

@interface IMPURLSessionManager : NSObject <NSURLSessionDelegate>
@end

@implementation IMPURLSessionManager {
  NSMutableDictionary<NSURLSessionTask *, IMPURLTaskData *> *_taskData;
  imp::FutureInterrupter _globalInterrupter;
}

- (instancetype)init:(imp::FutureInterrupter)globalInterrupter {
  self = [super init];
  if (self) {
    _taskData = [[NSMutableDictionary alloc] init];
    _globalInterrupter = globalInterrupter;
  }
  return self;
}

- (IMPURLTaskData *)dataForTask:(NSURLSessionTask *)task {
  @synchronized(_taskData) {
    return [_taskData objectForKey:task];
  }
}

- (IMPURLTaskData *)startTrackingTask:(NSURLSessionTask *)task
                         forURLString:(NSString *)URLString
                     withNativeLoader:(imp::resources::IosUrlLoader *)nativeLoader {
  IMPURLTaskData *result = [[IMPURLTaskData alloc] initForURLString:URLString
                                                   withNativeLoader:nativeLoader];
  @synchronized(_taskData) {
    [_taskData setObject:result forKey:task];
  }
  return result;
}

- (void)stopTrackingTask:(NSURLSessionTask *)task {
  @synchronized(_taskData) {
    [_taskData removeObjectForKey:task];
  }
}

- (void)cancelAll {
  NSMutableDictionary<NSURLSessionTask *, IMPURLTaskData *> *taskDataCopy;
  @synchronized(_taskData) {
    taskDataCopy = [_taskData copy];
    [_taskData removeAllObjects];
  }

  for (NSURLSessionTask *task in taskDataCopy) {
    IMPURLTaskData *data = [taskDataCopy objectForKey:task];
    [data cancel];
    [task cancel];
  }
}

- (void)URLSession:(NSURLSession *)session
              dataTask:(NSURLSessionDataTask *)task
    didReceiveResponse:(NSURLResponse *)response
     completionHandler:(void (^)(NSURLSessionResponseDisposition disposition))handler {
  [self dataForTask:task].expectedSize = response.expectedContentLength;
  handler(NSURLSessionResponseAllow);
}

- (void)URLSession:(NSURLSession *)session
          dataTask:(NSURLSessionDataTask *)task
    didReceiveData:(NSData *)data {
  IMPURLTaskData *taskData = [self dataForTask:task];
  if ([taskData isInterrupted:_globalInterrupter]) {
    NSLog(@"Ignoring data for interrupted task");
  }

  [taskData addData:data];
}

- (void)URLSession:(NSURLSession *)session
             dataTask:(NSURLSessionDataTask *)dataTask
    willCacheResponse:(NSCachedURLResponse *)proposedResponse
    completionHandler:(void (^)(NSCachedURLResponse *cachedResponse))completionHandler {
  completionHandler(proposedResponse);
}

- (void)URLSession:(NSURLSession *)session
                    task:(NSURLSessionTask *)task
    didCompleteWithError:(nullable NSError *)error {
  __weak __typeof__(self) weakSelf = self;
  // Complete the future on the main thread.
  dispatch_async(dispatch_get_main_queue(), ^{
    [[weakSelf dataForTask:task] completeFuture:error];
    [weakSelf stopTrackingTask:task];
  });
}

@end

namespace imp::resources {
namespace {

constexpr NSUInteger kCacheMemoryCapacity = 50 * 1024 * 1024;  // 50 MB
constexpr NSUInteger kCacheDiskCapacity = 250 * 1024 * 1024;   // 250 MB
constexpr NSString *kCacheDiskPath = @"ImpressUrlCache";

IosUrlLoader::IosUrlLoader() {
  // Make a url session config that supports caching with our own cache for impress url requests.
  // We make the cache larger than the default so that it is large enough to cache glTF, textures,
  // materials, etc. NSURLSession will only cache files that are <= 5% of the total cache size.
  NSURLSessionConfiguration *config = [NSURLSessionConfiguration defaultSessionConfiguration];
  config.URLCache = [[NSURLCache alloc] initWithMemoryCapacity:kCacheMemoryCapacity
                                                  diskCapacity:kCacheDiskCapacity
                                                      diskPath:kCacheDiskPath];

  // Make our own url session instead of using the shared session singleton.
  // This allows us to do the following:
  // 1. Construct our own NSOperationQueue that supports simultaneous tasks, unlike sharedSession
  //    which is serialized.
  // 2. Cancel all tasks associated with the session when the loader is shutdown.
  // 3. Use our own http cache.
  session_manager_ = [[IMPURLSessionManager alloc] init:GetGlobalInterrupter()];
  url_session_ = [NSURLSession sessionWithConfiguration:config
                                               delegate:session_manager_
                                          delegateQueue:[[NSOperationQueue alloc] init]];
  last_progress_update_time_ = ProfilingClock::GetMonotonicClockTime();
}

IosUrlLoader::~IosUrlLoader() { Shutdown(); }

void IosUrlLoader::Shutdown() {
  if (session_manager_) {
    UrlLoader::Shutdown();

    [session_manager_ cancelAll];
    [url_session_ invalidateAndCancel];
    session_manager_ = nil;
    url_session_ = nil;
  }
}

void IosUrlLoader::SetExpectedSize(const std::string &string_url, size_t expected_size) {
  absl::MutexLock lock(&download_progress_info_->download_progress_map_mutex);
  download_progress_info_->download_progress_map[string_url] =
      EntryProgressInfo{.downloaded_size = 0, .total_size = expected_size};
}

void IosUrlLoader::SetFinalSize(const std::string &string_url, size_t final_size) {
  absl::MutexLock lock(&download_progress_info_->download_progress_map_mutex);
  download_progress_info_->download_progress_map[string_url] =
      EntryProgressInfo{.downloaded_size = final_size, .total_size = final_size};
}

bool IosUrlLoader::TryAddData(const std::string &string_url, size_t downloaded_size) {
  auto current_time = ProfilingClock::GetMonotonicClockTime();
  if ((current_time - last_progress_update_time_) <= kProgressUpdateIntervalLimit) return false;

  absl::MutexLock lock(&download_progress_info_->download_progress_map_mutex);
  download_progress_info_->download_progress_map[string_url].downloaded_size += downloaded_size;
  last_progress_update_time_ = current_time;
  return true;
}

Future<absl::Cord> IosUrlLoader::LoadUrl(const std::string &url) {
  @autoreleasepool {
    // Use the NSUrlSession API to create an async task to download the url.
    // When complete, return the result through the previously declared future.
    NSString *url_string = @(url.c_str());

    // Check if the given URL is a local file
    NSString *file_prefix = @(kLocalFileScheme.data());
    if ([url_string hasPrefix:file_prefix]) {
      return Future<absl::Cord>::Schedule(
          [url, url_string, file_prefix]() -> absl::StatusOr<absl::Cord> {
            NSString *local_file = [url_string substringFromIndex:[file_prefix length]];
            NSFileManager *file_manager = [NSFileManager defaultManager];

            if (![file_manager fileExistsAtPath:local_file]) {
              return absl::NotFoundError(
                  absl::StrFormat("Failed to find %s in local filesystem.", url));
            }

            NSData *file_contents = [file_manager contentsAtPath:local_file];
            __block absl::Cord cord;
            size_t bytes_read = 0;
            while (bytes_read < file_contents.length) {
              size_t remaining_size = file_contents.length - bytes_read;
              __block absl::CordBuffer buffer =
                  absl::CordBuffer::CreateWithDefaultLimit(remaining_size);
              absl::Span<char> data = buffer.available_up_to(remaining_size);
              [file_contents getBytes:data.data() range:NSMakeRange(bytes_read, data.size())];
              bytes_read += data.size();
              buffer.IncreaseLengthBy(data.size());
              cord.Append(std::move(buffer));
            }
            return cord;
          },
          {.executor = Executor::Type::kBackground});
    }

    NSMutableURLRequest *request =
        [NSMutableURLRequest requestWithURL:[NSURL URLWithString:url_string]];

    for (const auto &[key, value] : GetConfig().request_headers) {
      NSString *key_string = @(key.c_str());
      NSString *value_string = @(value.c_str());
      [request setValue:value_string forHTTPHeaderField:key_string];
    }

    NSURLSessionDataTask *task = [url_session_ dataTaskWithRequest:request];

    IMPURLTaskData *task_data = [session_manager_ startTrackingTask:task
                                                       forURLString:url_string
                                                   withNativeLoader:this];
    Future<absl::Cord> future = [task_data createFuture];
    // This actually starts the download.
    [task resume];

    // Add a new future stage that will cancel the task when cancelled.
    // this way, if the code loading the url lets the future go out of scope or cancels it, then
    // the download will actually be cancelled instead of just continuing in the background.
    // return local_interrupter.MakeInterruptible(future).Then(
    return future.Then(
        [task](absl::StatusOr<absl::Cord> cord_or) {
          if (cord_or.status().code() == absl::StatusCode::kCancelled) {
            [task cancel];
          }
          return cord_or;
        },
        Executor::Type::kImmediate);
  }
}

}  // namespace

std::unique_ptr<UrlLoader> CreateIosUrlLoader() { return std::make_unique<IosUrlLoader>(); }

}  // namespace imp::resources

NS_ASSUME_NONNULL_END
