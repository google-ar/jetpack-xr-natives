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
#ifndef THIRD_PARTY_JETPACK_XR_NATIVES_LOGGING_JXR_LOG_SINK_H_
#define THIRD_PARTY_JETPACK_XR_NATIVES_LOGGING_JXR_LOG_SINK_H_

#include <jni.h>

#include <memory>
#include <string>

#include "logging/log_sink.h"

namespace androidx::xr {
using std::shared_ptr;
using std::string;

/*
 * LogSink for Jetpack XR applications on AOSP devices. Uses JNI to call the
 * Kotlin log APIs in androidx.xr.runtime.Log.
 * */
class JxrLogSink : public LogSink {
 public:
  void error(const string& message) const override;
  void warn(const string& message) const override;
  void info(const string& message) const override;
  void debug(const string& message) const override;
  void verbose(const string& message) const override;
  static shared_ptr<JxrLogSink> GetSharedInstance() {
    static shared_ptr<JxrLogSink> instance =
        shared_ptr<JxrLogSink>(new JxrLogSink());
    return instance;
  }

 private:
  void log_jni(const char* method_name, const char* message) const;
};

extern "C" {
extern JavaVM* g_VM;
extern jobject log_singleton;
}

}  // namespace androidx::xr

#endif  // THIRD_PARTY_JETPACK_XR_NATIVES_LOGGING_JXR_LOG_SINK_H_
