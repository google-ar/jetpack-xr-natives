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
#ifndef THIRD_PARTY_JETPACK_XR_NATIVES_LOGGING_LOG_H_
#define THIRD_PARTY_JETPACK_XR_NATIVES_LOGGING_LOG_H_

#include <cstdint>
#include <memory>
#include <sstream>
#include <utility>

#include "logging/log_sink.h"

namespace androidx::xr {

#define LOG_ERROR Log(LogLevel::kError)
#define LOG_WARN Log(LogLevel::kWarn)
#define LOG_INFO Log(LogLevel::kInfo)
#define LOG_DEBUG Log(LogLevel::kDebug)
#define LOG_VERBOSE Log(LogLevel::kVerbose)

enum class LogLevel : uint8_t {
  kVerbose = 0x02,  // ANDROID_LOG_VERBOSE
  kDebug = 0x03,    // ANDROID_LOG_DEBUG
  kInfo = 0x04,     // ANDROID_LOG_INFO
  kWarn = 0x05,     // ANDROID_LOG_WARN
  kError = 0x06,    // ANDROID_LOG_ERROR
};

// Utility for logging messages in Jetpack XR.
class Log {
 public:
  Log(LogLevel level);
  ~Log();

  template <typename T>
  Log& operator<<(T&& value) {
    stream_ << std::forward<T>(value);
    return *this;
  }

 private:
  std::shared_ptr<LogSink> log_sink_;
  LogLevel level_;
  std::stringstream stream_;
};

}  // namespace androidx::xr

#endif  // THIRD_PARTY_JETPACK_XR_NATIVES_LOGGING_LOG_H_
