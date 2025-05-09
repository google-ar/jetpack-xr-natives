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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_PLATFORM_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_PLATFORM_HELPERS_H_

#include <string>

#include "absl/base/attributes.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"

namespace imp {

namespace output {
ABSL_DEPRECATED("Use ABSL_MIN_LOG_LEVEL preprocessor macro instead")
void Configure(bool verbose);

ABSL_DEPRECATED("Use LOG(INFO) instead")
void Info(absl::string_view info);

ABSL_DEPRECATED("Use LOG(WARNING) instead")
void Warning(absl::string_view warning);

ABSL_DEPRECATED("Use LOG(ERROR) instead")
void Error(absl::string_view error);

[[noreturn]] ABSL_DEPRECATED("Use LOG(FATAL) instead") void Fatal(
    absl::string_view fatal);

ABSL_DEPRECATED("Use ABSL_RAW_LOG() instead")
void RawText(absl::string_view raw_text);

enum class OutputKind {
  kInfo,
  kWarning,
  kError,
  kFatal,
  kMax = kFatal,
};

// A function type for an external log output function.
typedef void (*PlatformOutputFunction)(void* context, OutputKind kind,
                                       absl::string_view body);

// TODO: Migrate all usages of this API to absl::LogSink after
// Impress has migrated to using absl logging. Then, eliminate this API.
void AddExternalLogHandler(void* context, PlatformOutputFunction f);
void RemoveExternalLogHandler(void* context);

template <class... Args>
void Info(const absl::FormatSpec<Args...>& format, Args&&... args) {
  Info(absl::StrFormat(format, std::forward<Args>(args)...));
}
template <class... Args>
void Warning(const absl::FormatSpec<Args...>& format, Args&&... args) {
  Warning(absl::StrFormat(format, std::forward<Args>(args)...));
}
template <class... Args>
void Error(const absl::FormatSpec<Args...>& format, Args&&... args) {
  Error(absl::StrFormat(format, std::forward<Args>(args)...));
}
template <class... Args>
[[noreturn]] void Fatal(const absl::FormatSpec<Args...>& format,
                        Args&&... args) {
  Fatal(absl::StrFormat(format, std::forward<Args>(args)...));
}
template <class... Args>
void RawText(const absl::FormatSpec<Args...>& format, Args&&... args) {
  RawText(absl::StrFormat(format, std::forward<Args>(args)...));
}

}  // namespace output

// Get a system-colloquial integer identifier for the currently running thread
uint32_t GetThreadId();
// Get the thread priority, phrased as niceness, since the system can get
// confused about which direction a "higher" priority goes.  The higher the
// niceness, the lower the probability of being scheduled.
int32_t GetThreadNiceness(uint32_t tid);
void SetThreadNiceness(uint32_t tid, int32_t niceness);
// Get roughly 16 bytes worth of the currently running thread name
std::string GetThreadName();
void SetThreadName(const char* name);

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_PLATFORM_HELPERS_H_
