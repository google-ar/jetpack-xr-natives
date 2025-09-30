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

#include "core/resources/android_url_loader.h"

#include <cassert>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/cord.h"
#include "absl/strings/cord_buffer.h"
#include "absl/strings/match.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "core/async/future_interrupter.h"
#include "core/config.h"
#include "core/resources/android_resource_loader.h"

#if IMP_PLATFORM(ANDROID)
#include <android/asset_manager.h>
#include <android/asset_manager_jni.h>
#endif  // IMP_PLATFORM(ANDROID)

#include <jni.h>

#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/common/context.h"
#include "core/common/jni_helpers.h"
#include "core/resources/url_loader.h"
#include "core/view/platforms/android/wrappers/input_stream.h"

#if IMP_PLATFORM(ANDROID) || IMP_PLATFORM(ROBOLECTRIC)
#include "core/view/platforms/android/wrappers/activity_context.h"
#endif  // IMP_PLATFORM(ANDROID) || IMP_PLATFORM(ROBOLECTRIC)

namespace imp {
namespace resources {
namespace {

// Java wrapper for java.net.URLConnection
class URLConnection : public JavaWrapper {
 public:
  URLConnection(JNIEnv* env, JniUniquePtr<jobject> java_url_connection,
                const char* class_path = "java/net/URLConnection")
      : JavaWrapper(env, class_path) {
    SetSelf(java_url_connection.release());
    get_input_stream_ =
        GetMethodHandle("getInputStream", "()Ljava/io/InputStream;");
#if IMP_PLATFORM(ANDROID_API24)
    get_content_length_ = GetMethodHandle("getContentLengthLong", "()J");
#else
    get_content_length_ = GetMethodHandle("getContentLength", "()I");
#endif
    set_request_property_ = GetMethodHandle(
        "setRequestProperty", "(Ljava/lang/String;Ljava/lang/String;)V");
    assert(get_input_stream_);
    assert(get_content_length_);
    assert(set_request_property_);
  }

  std::unique_ptr<InputStream> GetInputStream() {
    JniUniquePtr<jobject> input_stream =
        WrapJni(Env(), CallObjectMethod(get_input_stream_));
    if (JavaExceptionPrintClear(Env()) || !input_stream) {
      return absl::WrapUnique<InputStream>(nullptr);
    }
    return std::make_unique<InputStream>(Env(), std::move(input_stream));
  }

  size_t GetContentLength() {
#if IMP_PLATFORM(ANDROID_API24)
    jlong content_length = CallLongMethod(get_content_length_);
#else
    jint content_length = CallIntMethod(get_content_length_);
#endif
    // URLConnection.GetContentLength() may return -1 if the content length is
    // not known. In this case we should use kUnknownContentLength to represent
    // the case and avoid integer overflow.
    if (content_length < 0) {
      return kUnknownContentLength;
    }
    return static_cast<size_t>(content_length);
  }

  void SetRequestProperty(const std::string& key, const std::string& value) {
    JniUniquePtr<jstring> key_string = ToJniString(Env(), key);
    if (JavaExceptionPrintClear(Env())) {
      return;
    }
    JniUniquePtr<jstring> value_string = ToJniString(Env(), value);
    if (JavaExceptionPrintClear(Env())) {
      return;
    }
    CallVoidMethod(set_request_property_, key_string.get(), value_string.get());
    JavaExceptionPrintClear(Env());
  }

 private:
  JniHandle get_input_stream_;
  JniHandle get_content_length_;
  JniHandle set_request_property_;
};

class HttpUrlConnection : public URLConnection {
 public:
  HttpUrlConnection(JNIEnv* env, JniUniquePtr<jobject> java_url_connection)
      : URLConnection(env, std::move(java_url_connection),
                      "java/net/HttpURLConnection") {
    disconnect_ = GetMethodHandle("disconnect", "()V");
  }

  ~HttpUrlConnection() { Disconnect(); }

  void Disconnect() { CallVoidMethod(disconnect_); }

 private:
  JniHandle disconnect_;
};

// Java wrapper for java.net.URL
class URL : public JavaWrapper {
 public:
  URL(JNIEnv* env, const std::string& url)
      : JavaWrapper(env, "java/net/URL", "(Ljava/lang/String;)V",
                    WrapJni(env, ToString(env, url)).get()),
        use_http_(absl::StartsWith(url, "http")) {
    open_connection_ =
        GetMethodHandle("openConnection", "()Ljava/net/URLConnection;");
    assert(open_connection_);
  }

  std::unique_ptr<URLConnection> OpenConnection() {
    JniUniquePtr<jobject> connection =
        WrapJni(Env(), CallObjectMethod(open_connection_));
    if (JavaExceptionPrintClear(Env()) || !connection) {
      return absl::WrapUnique<URLConnection>(nullptr);
    }

    if (use_http_) {
      return std::make_unique<HttpUrlConnection>(Env(), std::move(connection));
    } else {
      return std::make_unique<URLConnection>(Env(), std::move(connection));
    }
  }

 private:
  JniHandle open_connection_;
  bool use_http_;
};

// UrlLoader that uses JNI calls into the Java HTTP loader to do URL requests.
class AndroidUrlLoader : public UrlLoader {
 public:
  explicit AndroidUrlLoader(const Context& context);

  Future<absl::Cord> LoadUrl(const std::string& url) override;

 private:
  AndroidResourceLoader android_resource_loader_;
  const Context& context_;
};

AndroidUrlLoader::AndroidUrlLoader(const Context& context)
    : android_resource_loader_(context), context_(context) {}

Future<absl::Cord> AndroidUrlLoader::LoadUrl(const std::string& string_url) {
  if (AndroidResourceLoader::IsAndroidSpecialUrl(string_url)) {
    return android_resource_loader_.LoadUrl(string_url);
  }

  FutureInterrupter local_interrupter;
  return local_interrupter.MakeInterruptible(Future<absl::Cord>::Schedule(
      [string_url, local_interrupter,
       global_interrupter = GetGlobalInterrupter(), &config = GetConfig(),
       download_progress_info = download_progress_info_,
       context = context_]() -> absl::StatusOr<absl::Cord> {
        if (local_interrupter.IsInterrupted() ||
            global_interrupter.IsInterrupted()) {
          return absl::CancelledError("Interrupted before opening connection.");
        }

        JNIEnv* env = context.GetJniEnv();
        URL url(env, string_url);
        auto url_connection = url.OpenConnection();
        if (!url_connection) {
          return absl::InternalError("Failed to create URLConnection.");
        }

        if (local_interrupter.IsInterrupted() ||
            global_interrupter.IsInterrupted()) {
          return absl::CancelledError("Interrupted after opening connection.");
        }

        for (const auto& [key, value] : config.request_headers) {
          url_connection->SetRequestProperty(key, value);
        }

        auto input_stream = url_connection->GetInputStream();
        if (!input_stream) {
          return absl::InternalError(absl::StrFormat(
              "Failed to create InputStream for url: \"%s\"", string_url));
        }

        return input_stream->BlockingReadFromJavaInputStream(
            env, string_url, url_connection->GetContentLength(),
            download_progress_info,
            std::vector<FutureInterrupter>{local_interrupter,
                                           global_interrupter});
      },
      {.executor = Executor::Type::kBackground}));
}

}  // namespace

std::unique_ptr<UrlLoader> CreateAndroidLoader(const Context& context) {
  return std::make_unique<AndroidUrlLoader>(context);
}

}  // namespace resources
}  // namespace imp
