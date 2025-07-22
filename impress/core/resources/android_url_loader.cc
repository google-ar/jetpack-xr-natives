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
#include <cstdio>
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
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"

#if IMP_PLATFORM(ANDROID)
#include <android/asset_manager.h>
#include <android/asset_manager_jni.h>
#endif  // IMP_PLATFORM(ANDROID)

#include <jni.h>

#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/async/future_group.h"
#include "core/async/future_interrupter.h"
#include "core/common/context.h"
#include "core/common/jni_helpers.h"
#include "core/config.h"
#include "core/resources/url_loader.h"
#include "core/view/platforms/android/wrappers/input_stream.h"

#if IMP_PLATFORM(ANDROID) || IMP_PLATFORM(ROBOLECTRIC)
#include "core/view/platforms/android/wrappers/activity_context.h"
#endif  // IMP_PLATFORM(ANDROID) || IMP_PLATFORM(ROBOLECTRIC)

namespace imp {
namespace resources {
namespace {

// Prefix for URLs that reference an asset on the Android filesystem.
constexpr absl::string_view kFilePrefix = "file://";

// Prefix for URLs that reference an Android Asset file, to be loaded through
// AssetManager.  This prefix is consistent with WebView functionality, see
// https://developer.android.com/reference/android/webkit/WebSettings#setAllowFileAccess(boolean)
constexpr absl::string_view kAssetManagerPrefix = "file:///android_asset/";

// Prefix for URLs that reference a file in Android external storage.
constexpr absl::string_view kExternalStoragePrefix = "file:///storage/";
// Prefix for URLs that reference a file in Android app storage.
constexpr absl::string_view kAppStoragePrefix = "file:///data/";
// Prefix for URLs that reference a resource in the Android APK.
constexpr absl::string_view kAndroidResourcePrefix = "android.resource://";
// Prefix for URLs Android content URIs.
constexpr absl::string_view kContentPrefix = "content://";

// Loads a raw file from the Android storage by path.
// Note: path needs to be FileDescriptor-based, i.e. /proc/self/fd/...
Future<absl::Cord> LoadRawFile(absl::string_view path) {
  return Future<absl::Cord>::Schedule(
      [path = std::string(path)]() -> absl::StatusOr<absl::Cord> {
        FILE* file = fopen(path.c_str(), "r");
        if (!file) {
          return absl::NotFoundError(absl::StrCat("Cannot load file ", path));
        }
        absl::Cord result;
        int c;
        while ((c = std::fgetc(file)) != EOF) {
          char ch = static_cast<char>(c);
          result.Append(absl::string_view(&ch, 1));
        }
        fclose(file);
        return result;
      },
      Executor::Type::kBackground);
}

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

#if IMP_PLATFORM(ANDROID) || IMP_PLATFORM(ROBOLECTRIC)
// Java wrapper for android.net.Uri
class AndroidUri : public JavaWrapper {
 public:
  AndroidUri(JNIEnv* env, const std::string& string_uri)
      : JavaWrapper(env, "android/net/Uri") {
    JniHandle parse =
        GetStaticMethodHandle("parse", "(Ljava/lang/String;)Landroid/net/Uri;");
    assert(parse);

    JniUniquePtr<jstring> jni_uri = CreateJniString(env, string_uri);
    if (JavaExceptionPrintClear(Env())) {
      return;
    }

    JniUniquePtr<jobject> uri =
        WrapJni(env, CallStaticObjectMethod(parse, jni_uri.get()));
    if (JavaExceptionPrintClear(Env()) || !uri) {
      return;
    }
    if (uri) {
      SetSelf(LocalToGlobalRef(std::move(uri)));
    }
  }
};

// Java wrapper for android.content.ContentResolver
class ContentResolver : public JavaWrapper {
 public:
  ContentResolver(JNIEnv* env, JniUniquePtr<jobject> content_resolver)
      : JavaWrapper(env, std::move(content_resolver),
                    "android/content/ContentResolver") {
    open_input_stream_ = GetMethodHandle(
        "openInputStream", "(Landroid/net/Uri;)Ljava/io/InputStream;");
    assert(open_input_stream_);
  }

  std::unique_ptr<InputStream> OpenInputStream(jobject uri) {
    JniUniquePtr<jobject> input_stream =
        WrapJni(Env(), CallObjectMethod(open_input_stream_, uri));
    if (JavaExceptionPrintClear(Env()) || !input_stream) {
      // Failed to find that resource.
      return absl::WrapUnique<InputStream>(nullptr);
    }

    return std::make_unique<InputStream>(Env(), std::move(input_stream));
  }

 private:
  JniHandle open_input_stream_;
};
#endif  // IMP_PLATFORM(ANDROID) || IMP_PLATFORM(ROBOLECTRIC)

// UrlLoader that uses JNI calls into the java http loader to do url requests.
class AndroidUrlLoader : public UrlLoader {
 public:
  explicit AndroidUrlLoader(const Context& context);

  Future<absl::Cord> LoadUrl(const std::string& url,
                             std::optional<FutureGroup> future_group) override;

 private:
  Future<absl::Cord> LoadAndroidResource(
      const std::string& string_uri, std::optional<FutureGroup> future_group);

  Future<absl::Cord> LoadAndroidAsset(const std::string& asset_path,
                                      std::optional<FutureGroup> future_group);

  const Context& context_;
};

AndroidUrlLoader::AndroidUrlLoader(const Context& context)
    : context_(context) {}

Future<absl::Cord> AndroidUrlLoader::LoadUrl(
    const std::string& string_url, std::optional<FutureGroup> future_group) {
  if (absl::StartsWith(string_url, kAssetManagerPrefix)) {
    return LoadAndroidAsset(string_url.substr(kAssetManagerPrefix.size()),
                            future_group);
  }

  if (absl::StartsWith(string_url, kAndroidResourcePrefix) ||
      absl::StartsWith(string_url, kContentPrefix)) {
    return LoadAndroidResource(string_url, future_group);
  }

  if (absl::StartsWith(string_url, kExternalStoragePrefix) ||
      absl::StartsWith(string_url, kAppStoragePrefix)) {
    return LoadRawFile(string_url.substr(kFilePrefix.size()));
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
      {.executor = Executor::Type::kBackground, .future_group = future_group}));
}

Future<absl::Cord> AndroidUrlLoader::LoadAndroidResource(
    const std::string& string_uri, std::optional<FutureGroup> future_group) {
#if IMP_PLATFORM(ANDROID) || IMP_PLATFORM(ROBOLECTRIC)
  // Parse the URI and load our ContentResolver on the current thread, before
  // scheduling the actual disk I/O on background. This ensures that if we're
  // using Robolectric for testing, we'll use its class loader.
  JNIEnv* env = context_.GetJniEnv();
  auto android_uri = std::make_shared<AndroidUri>(env, std::string(string_uri));
  if (JavaExceptionPrintClear(env) || !android_uri->WeakReference()) {
    // Note: This code path is near-impossible to trigger for coverage.
    // Uri.parse() does no validation; the only thing it will throw on is
    // a null string, which won't happen since we need to have at least
    // "android.resource://" present to get to this function.
    return Future<absl::Cord>(absl::InternalError(
        absl::StrFormat("Failed to parse URI: \"%s\"", string_uri)));
  }

  imp::android::ActivityContext activity_context(env,
                                                 context_.GetActivityContext());
  auto content_resolver = std::make_shared<ContentResolver>(
      env, WrapJni(env, activity_context.GetContentResolver()));
  if (JavaExceptionPrintClear(env) || !content_resolver->WeakReference()) {
    return Future<absl::Cord>(absl::InternalError(
        absl::StrFormat("Failed to create ContentResolver")));
  }

  FutureInterrupter local_interrupter;
  return local_interrupter.MakeInterruptible(Future<absl::Cord>::Schedule(
      [string_uri, android_uri, content_resolver, local_interrupter,
       global_interrupter = GetGlobalInterrupter(),
       download_progress_info = download_progress_info_,
       context = context_]() -> absl::StatusOr<absl::Cord> {
        if (local_interrupter.IsInterrupted() ||
            global_interrupter.IsInterrupted()) {
          return absl::CancelledError("Interrupted before resolving URI.");
        }

        JNIEnv* env = context.GetJniEnv();
        auto input_stream =
            content_resolver->OpenInputStream(android_uri->WeakReference());
        if (!input_stream) {
          return absl::InternalError(absl::StrFormat(
              "Failed to create InputStream for URI: \"%s\"", string_uri));
        }

        return input_stream->BlockingReadFromJavaInputStream(
            env, string_uri, /*content_length=*/0, download_progress_info,
            std::vector<FutureInterrupter>{local_interrupter,
                                           global_interrupter});
      },
      {.executor = Executor::Type::kBackground, .future_group = future_group}));
#else   // !(IMP_PLATFORM(ANDROID) || IMP_PLATFORM(ROBOLECTRIC))
  return Future<absl::Cord>(
      absl::CancelledError("Android Resources only supported on Android"));
#endif  // IMP_PLATFORM(ANDROID) || IMP_PLATFORM(ROBOLECTRIC)
}

Future<absl::Cord> AndroidUrlLoader::LoadAndroidAsset(
    const std::string& asset_path, std::optional<FutureGroup> future_group) {
#if !IMP_PLATFORM(ANDROID)
  return Future<absl::Cord>(
      absl::CancelledError("Android Assets only supported on Android"));
#else   // IMP_PLATFORM(ANDROID)
  FutureInterrupter local_interrupter;
  return local_interrupter.MakeInterruptible(Future<absl::Cord>::Schedule(
      [asset_path, local_interrupter,
       global_interrupter = GetGlobalInterrupter(),
       context = context_]() -> absl::StatusOr<absl::Cord> {
        if (local_interrupter.IsInterrupted() ||
            global_interrupter.IsInterrupted()) {
          return absl::CancelledError("Interrupted before opening connection.");
        }

        JNIEnv* env = context.GetJniEnv();
        imp::android::ActivityContext activity_context(
            env, context.GetActivityContext());
        AAssetManager* asset_manager = activity_context.GetAssets();
        AAsset* asset = AAssetManager_open(asset_manager, asset_path.c_str(),
                                           AASSET_MODE_STREAMING);

        if (!asset) {
          return absl::UnavailableError(
              absl::StrFormat("Unable to find Android asset %s", asset_path));
        }

        absl::Cord cord;
        int remaining_size = AAsset_getLength(asset);
        while (remaining_size > 0) {
          absl::CordBuffer buffer =
              absl::CordBuffer::CreateWithDefaultLimit(remaining_size);
          absl::Span<char> data = buffer.available_up_to(remaining_size);
          const int read_length = AAsset_read(asset, data.data(), data.size());
          buffer.IncreaseLengthBy(read_length);
          remaining_size -= read_length;
          cord.Append(std::move(buffer));

          if (local_interrupter.IsInterrupted() ||
              global_interrupter.IsInterrupted()) {
            AAsset_close(asset);
            return absl::CancelledError("Interrupted while reading chunks.");
          }
        }

        AAsset_close(asset);
        return cord;
      },
      {.executor = Executor::Type::kBackground, .future_group = future_group}));
#endif  // !IMP_PLATFORM(ANDROID)
}

}  // namespace

std::unique_ptr<UrlLoader> CreateAndroidLoader(const Context& context) {
  return std::make_unique<AndroidUrlLoader>(context);
}

}  // namespace resources
}  // namespace imp
