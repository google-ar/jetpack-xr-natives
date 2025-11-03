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

#include "core/common/log.h"

#include "absl/strings/string_view.h"
#include "core/common/platform_helpers.h"

namespace imp {

bool& GetEnabledInternal() {
  // It's best to store static data structures as a static pointer inside of
  // a function, per (broken link) and (broken link).
  static bool* is_enabled = new bool(false);
  return *is_enabled;
}

void ImpressStreamLogger::SetEnabled(bool enabled) {
  bool& is_enabled = GetEnabledInternal();
  is_enabled = enabled;
}

bool ImpressStreamLogger::IsEnabled() { return GetEnabledInternal(); }

ImpressStreamLogger::ImpressStreamLogger(output::OutputKind kind)
    : kind_(kind) {}
ImpressStreamLogger::~ImpressStreamLogger() { Flush(); }

ImpressStreamLogger& ImpressStreamLogger::AtLocation(absl::string_view file,
                                                     int line) {
  ss_ << file << ":" << line << " ";
  return *this;
}

void ImpressStreamLogger::Flush() {
  if (!IsEnabled()) return;

  switch (kind_) {
    case output::OutputKind::kInfo:
      output::Info(ss_.str());
      break;
    case output::OutputKind::kWarning:
      output::Warning(ss_.str());
      break;
    case output::OutputKind::kError:
      output::Error(ss_.str());
      break;
    case output::OutputKind::kFatal:
      output::Fatal(ss_.str());
      break;
  }
}

}  // namespace imp
