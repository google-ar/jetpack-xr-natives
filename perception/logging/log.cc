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
#include "logging/log.h"

namespace androidx::xr {
Log::~Log() {
  switch (level_) {
    case LogLevel::kVerbose:
      log_sink_->verbose(stream_.str().c_str());
      break;
    case LogLevel::kDebug:
      log_sink_->debug(stream_.str().c_str());
      break;
    case LogLevel::kInfo:
      log_sink_->info(stream_.str().c_str());
      break;
    case LogLevel::kWarn:
      log_sink_->warn(stream_.str().c_str());
      break;
    case LogLevel::kError:
      log_sink_->error(stream_.str().c_str());
      break;
  }
}
}  // namespace androidx::xr
