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

#include "core/common/platform_helpers.h"

#include <string>
#include <vector>

#include "absl/base/attributes.h"
#include "absl/base/const_init.h"
#include "absl/base/log_severity.h"
#include "absl/base/thread_annotations.h"
#include "absl/log/log_entry.h"
#include "absl/log/log_sink.h"
#include "absl/log/log_sink_registry.h"
#include "absl/synchronization/mutex.h"
#include "core/common/string_helpers.h"
#include "core/config.h"

#if IMP_PLATFORM(ANDROID)
#include <android/log.h>
#endif  // IMP_PLATFORM
#if IMP_PLATFORM(ANDROID) || IMP_PLATFORM(LINUX)
#include <sys/resource.h>
#endif  // IMP_PLATFORM(ANDROID) || IMP_PLATFORM(LINUX)
#if IMP_PLATFORM(LINUX) || IMP_PLATFORM(ANDROID_API19)
#include <sys/prctl.h>
#endif  // IMP_PLATFORM(LINUX) || IMP_PLATFORM(ANDROID_API19)

#if IMP_PLATFORM(MACOS) || IMP_PLATFORM(IOS)
#include <pthread.h>
#elif IMP_PLATFORM(ANDROID)
#include <unistd.h>
#elif IMP_PLATFORM(WASM)
#include <pthread.h>
#else  // IMP_PLATFORM(MACOS) || IMP_PLATFORM(IOS)
#include <sys/syscall.h>
#include <sys/types.h>
#include <unistd.h>
#endif  // IMP_PLATFORM(MACOS) || IMP_PLATFORM(IOS)

#if IMP_PLATFORM(WASM)
#include <emscripten/console.h>
#endif  // IMP_PLATFORM(WASM)

namespace imp {
namespace output {

void LogToExternalHandlers(OutputKind kind, absl::string_view body);

namespace {

class ImpressLogSink : public absl::LogSink {
 public:
  void Send(const absl::LogEntry& entry) override {
    OutputKind kind = OutputKind::kInfo;
    switch (entry.log_severity()) {
      case absl::LogSeverity::kInfo:
        kind = OutputKind::kInfo;
        break;
      case absl::LogSeverity::kWarning:
        kind = OutputKind::kWarning;
        break;
      case absl::LogSeverity::kError:
        kind = OutputKind::kError;
        break;
      case absl::LogSeverity::kFatal:
        kind = OutputKind::kFatal;
        break;
    }

    LogToExternalHandlers(kind, entry.text_message());
  }
};

using output::OutputKind;

#if IMP_PLATFORM(ANDROID)
const int LogTypeFromOutputKind(OutputKind kind) {
  switch (kind) {
    case OutputKind::kInfo:
      return ANDROID_LOG_VERBOSE;
      break;
    case OutputKind::kWarning:
      return ANDROID_LOG_WARN;
      break;
    default:
    case OutputKind::kError:
    case OutputKind::kFatal:
      return ANDROID_LOG_ERROR;
      break;
  }
}
const char* PrefixFromOutputKind(OutputKind kind) { return ""; }
void OutputLine(OutputKind kind, std::string line) {
  __android_log_print(LogTypeFromOutputKind(kind), "ImpView", "%s\n",
                      line.c_str());
}
#elif IMP_PLATFORM(WASM)
const char* PrefixFromOutputKind(OutputKind kind) {
  switch (kind) {
    case OutputKind::kInfo:
      return "Info";
      break;
    case OutputKind::kWarning:
      return "Warning";
      break;
    case OutputKind::kError:
      return "Error";
      break;
    default:
    case OutputKind::kFatal:
      return "FATAL";
      break;
  }
}
void OutputLine(OutputKind kind, absl::string_view line) {
  const char* data = line.data();
  size_t size = line.size();
  if (kind == OutputKind::kError || kind == OutputKind::kFatal) {
    emscripten_errn(data, size);
  } else {
    emscripten_outn(data, size);
  }
}
#else
const char* PrefixFromOutputKind(OutputKind kind) {
  switch (kind) {
    case OutputKind::kInfo:
      return "Info";
      break;
    case OutputKind::kWarning:
      return "Warning";
      break;
    case OutputKind::kError:
      return "Error";
      break;
    default:
    case OutputKind::kFatal:
      return "FATAL";
      break;
  }
}
void OutputLine(OutputKind kind, absl::string_view line) {
  FILE* pipe = (kind == OutputKind::kError || kind == OutputKind::kFatal)
                   ? stderr
                   : stdout;
  absl::FPrintF(pipe, "%s\n", line);
  if (kind == OutputKind::kFatal || kind == OutputKind::kError) fflush(pipe);
}
#endif  // IMP_PLATFORM
void OutputInternal(OutputKind kind, absl::string_view body) {
  std::vector<absl::string_view> lines;
  const char* prefix = PrefixFromOutputKind(kind);
  uint32_t prefix_length = strlen(prefix);
  const char* joiner = prefix_length ? ": " : "";
  Split(body, '\n', std::back_inserter(lines));
  bool first = true;
  for (auto line : lines) {
    if (first) {
      OutputLine(kind, absl::StrFormat("%s%s%s", prefix, joiner, line));
      first = false;
    } else {
      OutputLine(kind,
                 absl::StrFormat("%*c%s%s", prefix_length, ' ', joiner, line));
    }
  }

  LogToExternalHandlers(kind, body);
}

}  // namespace

ABSL_CONST_INIT absl::Mutex output_verbose_mutex(absl::kConstInit);
bool output_verbose = false;

void Configure(bool verbose) {
  absl::WriterMutexLock lock(&output_verbose_mutex);
  output_verbose = verbose;
}

struct ExternalLogHandler {
  void* context = nullptr;
  PlatformOutputFunction fn = nullptr;
};

std::vector<ExternalLogHandler>& GetExternalLogHandlers() {
  static std::vector<ExternalLogHandler>* external_log_handlers =
      new std::vector<ExternalLogHandler>();
  return *external_log_handlers;
}

ABSL_CONST_INIT absl::Mutex external_log_handlers_mutex(absl::kConstInit);

ImpressLogSink& GetImpressLogSink() {
  static ImpressLogSink* log_sink = new ImpressLogSink();
  return *log_sink;
}

void AddExternalLogHandler(void* context, PlatformOutputFunction f)
    ABSL_LOCKS_EXCLUDED(external_log_handlers_mutex) {
  bool need_to_add_sink = false;

  {
    absl::WriterMutexLock lock(&external_log_handlers_mutex);
    need_to_add_sink = GetExternalLogHandlers().empty();
    GetExternalLogHandlers().push_back(ExternalLogHandler{context, f});
  }

  if (need_to_add_sink) {
    absl::AddLogSink(&GetImpressLogSink());
  }
}

void LogToExternalHandlers(OutputKind kind, absl::string_view body)
    ABSL_LOCKS_EXCLUDED(external_log_handlers_mutex) {
  absl::ReaderMutexLock lock(&external_log_handlers_mutex);
  for (ExternalLogHandler& external_log_handler : GetExternalLogHandlers()) {
    external_log_handler.fn(external_log_handler.context, kind, body);
  }
}

void RemoveExternalLogHandler(void* context)
    ABSL_LOCKS_EXCLUDED(external_log_handlers_mutex) {
  bool need_to_remove_sink = false;

  {
    absl::WriterMutexLock lock(&external_log_handlers_mutex);
    for (auto itr = GetExternalLogHandlers().begin();
         itr != GetExternalLogHandlers().end(); ++itr) {
      if (itr->context == context) {
        GetExternalLogHandlers().erase(itr);
        need_to_remove_sink = GetExternalLogHandlers().empty();
        break;
      }
    }
  }

  if (need_to_remove_sink) {
    absl::RemoveLogSink(&GetImpressLogSink());
  }
}

void Info(absl::string_view info) {
  bool should_output = false;
  {
    absl::ReaderMutexLock lock(&output_verbose_mutex);
    should_output = output_verbose;
  }

  if (should_output) {
    OutputInternal(OutputKind::kInfo, info);
  }
}

void Warning(absl::string_view warning) {
  OutputInternal(OutputKind::kWarning, warning);
}

void Error(absl::string_view error) {
  OutputInternal(OutputKind::kError, error);
  fflush(stderr);
}

void Fatal(absl::string_view fatal) {
  OutputInternal(OutputKind::kFatal, fatal);
  std::abort();
}

void RawText(absl::string_view raw_text) {
  printf("%.*s\n", static_cast<int32_t>(raw_text.size()), raw_text.data());
}

}  // namespace output

uint32_t GetThreadId() {
#if IMP_PLATFORM(MACOS) || IMP_PLATFORM(IOS)
  uint64_t tid_storage;
  pthread_threadid_np(NULL, &tid_storage);
  uint32_t tid = static_cast<uint32_t>(tid_storage);
#elif IMP_PLATFORM(WASM)
  uint32_t tid = pthread_self();
#elif IMP_PLATFORM(ANDROID)
  uint32_t tid = gettid();
#else   // IMP_PLATFORM(MACOS) || IMP_PLATFORM(IOS)
  uint32_t tid = syscall(SYS_gettid);
#endif  // IMP_PLATFORM(MACOS) || IMP_PLATFORM(IOS)
  return tid;
}

int32_t GetThreadNiceness(uint32_t tid) {
#if IMP_PLATFORM(ANDROID) || IMP_PLATFORM(LINUX)
  return static_cast<int32_t>(getpriority(PRIO_PROCESS, static_cast<int>(tid)));
#else   // IMP_PLATFORM(ANDROID) || IMP_PLATFORM(LINUX)
  output::Warning("GetThreadNiceness not implemented on this platform");
#endif  // IMP_PLATFORM(ANDROID) || IMP_PLATFORM(LINUX)
  return 0;
}

void SetThreadNiceness(uint32_t tid, int32_t niceness) {
#if IMP_PLATFORM(ANDROID) || IMP_PLATFORM(LINUX)
  setpriority(PRIO_PROCESS, static_cast<pid_t>(tid),
              static_cast<int>(niceness));
#else   // IMP_PLATFORM(ANDROID) || IMP_PLATFORM(LINUX)
  output::Warning("GetThreadNiceness not implemented on this platform");
#endif  // IMP_PLATFORM(ANDROID) || IMP_PLATFORM(LINUX)
}

std::string GetThreadName() {
  char name_buffer[128] = {'\0'};

#if IMP_PLATFORM(ANDROID_API19)
  prctl(PR_GET_NAME, name_buffer, 0L, 0L, 0L);
#elif IMP_PLATFORM(MACOS) || IMP_PLATFORM(IOS) || IMP_PLATFORM(LINUX)
  pthread_getname_np(pthread_self(), name_buffer, sizeof(name_buffer));
#else
  constexpr auto kUnknownPlatform = "???";
  strncpy(name_buffer, kUnknownPlatform, strlen(kUnknownPlatform) + 1);
#endif  // IMP_PLATFORM(ANDROID_API19)
  return name_buffer;
}

void SetThreadName(const char* name) {
#if IMP_PLATFORM(ANDROID_API19) || IMP_PLATFORM(LINUX)
  pthread_setname_np(pthread_self(), name);
#else   // IMP_PLATFORM(ANDROID_API19) || IMP_PLATFORM(LINUX)
  output::Error("SetThreadName not supported on this platform");
#endif  // IMP_PLATFORM(ANDROID_API19) || IMP_PLATFORM(LINUX)
}

}  // namespace imp
