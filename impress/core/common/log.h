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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_LOG_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_LOG_H_

#include <sstream>

#include "absl/strings/string_view.h"
#include "core/common/platform_helpers.h"

namespace imp {

/**
 * Helper class to convert absl logging API calls to impress logging API
 * calls.
 *
 * Though absl logging API is preferred, due to issues with Copybara + Bazel +
 * AOSP, we need to use this as a drop-in replacement to fix runtime and build
 * issues.
 *
 * TODO: (broken link) Rename this file to better describe that it's a drop-in
 * replacement
 */
class ImpressStreamLogger {
 public:
  // Globally enable or disable logging.
  static void SetEnabled(bool enabled);
  static bool IsEnabled();

  ImpressStreamLogger(output::OutputKind kind);
  ~ImpressStreamLogger();

  template <typename T>
  ImpressStreamLogger& operator<<(T&& t) {
    if (!IsEnabled()) return *this;

    ss_ << t;
    return *this;
  }

  ImpressStreamLogger& AtLocation(absl::string_view file, int line);

  void Flush();

 private:
  output::OutputKind kind_;
  std::stringstream ss_;
};

constexpr output::OutputKind INFO = imp::output::OutputKind::kInfo;
constexpr output::OutputKind WARNING = imp::output::OutputKind::kWarning;
constexpr output::OutputKind ERROR = imp::output::OutputKind::kError;
constexpr output::OutputKind FATAL = imp::output::OutputKind::kFatal;

#define IMP_LOG(severity) imp::ImpressStreamLogger(severity)

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_LOG_H_
