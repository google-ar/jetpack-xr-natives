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
#ifndef THIRD_PARTY_JETPACK_XR_NATIVES_LOGGING_LOG_SINK_H_
#define THIRD_PARTY_JETPACK_XR_NATIVES_LOGGING_LOG_SINK_H_

#include <string>

namespace androidx::xr {

using std::string;

// Interface for handling log messages.
class LogSink {
 public:
  virtual ~LogSink() = default;
  virtual void error(const string& message) const = 0;
  virtual void warn(const string& message) const = 0;
  virtual void info(const string& message) const = 0;
  virtual void debug(const string& message) const = 0;
  virtual void verbose(const string& message) const = 0;
};

}  // namespace androidx::xr

#endif  // THIRD_PARTY_JETPACK_XR_NATIVES_LOGGING_LOG_SINK_H_
