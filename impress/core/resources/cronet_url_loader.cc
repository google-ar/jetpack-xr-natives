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

#include "core/resources/cronet_url_loader.h"

#include <jni.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "core/async/future.h"
#include "core/common/context.h"
#include "core/common/jni_helpers.h"
#include "core/resources/android_resource_loader.h"
#include "core/resources/url_loader.h"

#ifndef ENABLE_NETLOG
#define ENABLE_NETLOG 0
#endif

namespace imp {
namespace resources {
namespace {

// C++ wrapper for org.chromium.net.UrlRequest
class UrlRequest : public JavaWrapper, public UrlRequestHandle {
 public:
  UrlRequest(JNIEnv* env, JniUniquePtr<jobject> url_request);

  void Start() override;
  void Cancel() override;

 private:
  JniHandle start_method_;
  JniHandle cancel_method_;
};

// C++ wrapper for org.chromium.net.UrlRequest.Builder
class UrlRequestBuilder : public JavaWrapper {
 public:
  UrlRequestBuilder(JNIEnv* env, JniUniquePtr<jobject> url_request_builder);

  void AddHeader(const std::string& key, const std::string& value);
  std::unique_ptr<UrlRequestHandle> Build();

 private:
  JniHandle build_method_;
  JniHandle add_header_method_;
};

// C++ wrapper for org.chromium.net.CronetEngine
class CronetEngine : public JavaWrapper {
 public:
  CronetEngine(JNIEnv* env, JniUniquePtr<jobject> cronet_engine);
  ~CronetEngine() override;

  std::unique_ptr<UrlRequestBuilder> NewUrlRequestBuilder(
      absl::string_view url, JniUniquePtr<jobject> cronet_callback,
      JniUniquePtr<jobject> executor);

#if ENABLE_NETLOG
  void StartNetLogToFile(const std::string& file_name, bool log_all);
  void StopNetLog();
#endif

 private:
  JniHandle shutdown_method_;
  JniHandle new_url_request_builder_method_;

#if ENABLE_NETLOG
  JniHandle start_net_log_to_file_method_;
  JniHandle stop_net_log_method_;
#endif
};

// C++ wrapper for org.chromium.net.CronetEngine.Builder
class CronetEngineBuilder : public JavaWrapper {
 public:
  CronetEngineBuilder(JNIEnv* env, JniUniquePtr<jobject> context);

  std::unique_ptr<CronetEngine> Build();

 private:
  JniHandle build_method_;
  JniHandle enable_http2_method_;
  JniHandle enable_quic_method_;
};

CronetEngineBuilder::CronetEngineBuilder(JNIEnv* env,
                                         JniUniquePtr<jobject> context)
    : JavaWrapper(env, "org/chromium/net/CronetEngine$Builder",
                  "(Landroid/content/Context;)V", context.release()) {
  build_method_ = GetMethodHandle("build", "()Lorg/chromium/net/CronetEngine;");
  enable_http2_method_ = GetMethodHandle(
      "enableHttp2", "(Z)Lorg/chromium/net/CronetEngine$Builder;");
  enable_quic_method_ = GetMethodHandle(
      "enableQuic", "(Z)Lorg/chromium/net/CronetEngine$Builder;");

  if (JavaExceptionPrintClear(env)) return;

  
  
  
}

std::unique_ptr<CronetEngine> CronetEngineBuilder::Build() {
  // Set HTTP/2 and QUIC options.
  CallObjectMethod(enable_http2_method_, true);
  if (JavaExceptionPrintClear(Env())) {
    return nullptr;
  }
  CallObjectMethod(enable_quic_method_, true);
  if (JavaExceptionPrintClear(Env())) {
    return nullptr;
  }

  JniUniquePtr<jobject> java_cronet_engine = CallObjectMethod(build_method_);
  if (JavaExceptionPrintClear(Env())) {
    return nullptr;
  }

  auto cronet_engine = std::make_unique<CronetEngine>(
      Env(), LocalToGlobalRef(std::move(java_cronet_engine)));
  return cronet_engine;
}

CronetEngine::CronetEngine(JNIEnv* env, JniUniquePtr<jobject> cronet_engine)
    : JavaWrapper(env, std::move(cronet_engine),
                  "org/chromium/net/CronetEngine") {
  shutdown_method_ = GetMethodHandle("shutdown", "()V");
  new_url_request_builder_method_ = GetMethodHandle(
      "newUrlRequestBuilder",
      "(Ljava/lang/String;Lorg/chromium/net/UrlRequest$Callback;Ljava/util/"
      "concurrent/Executor;)Lorg/chromium/net/UrlRequest$Builder;");

#if ENABLE_NETLOG
  start_net_log_to_file_method_ =
      GetMethodHandle("startNetLogToFile", "(Ljava/lang/String;Z)V");
  stop_net_log_method_ = GetMethodHandle("stopNetLog", "()V");
#endif

  JavaExceptionPrintClear(Env());
}

CronetEngine::~CronetEngine() {
#if ENABLE_NETLOG
  StopNetLog();
#endif

  CallVoidMethod(shutdown_method_);
  JavaExceptionPrintClear(Env());
}

#if ENABLE_NETLOG
void CronetEngine::StartNetLogToFile(const std::string& file_name,
                                     bool log_all) {
  JniUniquePtr<jstring> jni_file_name = CreateJniString(Env(), file_name);
  if (JavaExceptionPrintClear(Env())) {
    return;
  }

  CallVoidMethod(start_net_log_to_file_method_, jni_file_name.get(), log_all);
  JavaExceptionPrintClear(Env());
}

void CronetEngine::StopNetLog() {
  CallVoidMethod(stop_net_log_method_);
  JavaExceptionPrintClear(Env());
}
#endif

std::unique_ptr<UrlRequestBuilder> CronetEngine::NewUrlRequestBuilder(
    absl::string_view url, JniUniquePtr<jobject> cronet_callback,
    JniUniquePtr<jobject> executor) {
  JniUniquePtr<jstring> jni_url = CreateJniString(Env(), std::string(url));
  if (JavaExceptionPrintClear(Env())) {
    return nullptr;
  }

  JniUniquePtr<jobject> result =
      CallObjectMethod(new_url_request_builder_method_, jni_url.get(),
                       cronet_callback.release(), executor.release());
  if (JavaExceptionPrintClear(Env())) {
    return nullptr;
  }

  auto url_request_builder = std::make_unique<UrlRequestBuilder>(
      Env(), LocalToGlobalRef(std::move(result)));
  return url_request_builder;
}

UrlRequestBuilder::UrlRequestBuilder(JNIEnv* env,
                                     JniUniquePtr<jobject> url_request_builder)
    : JavaWrapper(env, std::move(url_request_builder),
                  "org/chromium/net/UrlRequest$Builder") {
  add_header_method_ =
      GetMethodHandle("addHeader",
                      "(Ljava/lang/String;Ljava/lang/String;)Lorg/chromium/net/"
                      "UrlRequest$Builder;");
  build_method_ = GetMethodHandle("build", "()Lorg/chromium/net/UrlRequest;");
  JavaExceptionPrintClear(Env());
}

void UrlRequestBuilder::AddHeader(const std::string& key,
                                  const std::string& value) {
  JniUniquePtr<jstring> jni_key = CreateJniString(Env(), key);
  if (JavaExceptionPrintClear(Env())) {
    return;
  }
  JniUniquePtr<jstring> jni_value = CreateJniString(Env(), value);
  if (JavaExceptionPrintClear(Env())) {
    return;
  }
  CallObjectMethod(add_header_method_, jni_key.get(), jni_value.get());
  JavaExceptionPrintClear(Env());
}

std::unique_ptr<UrlRequestHandle> UrlRequestBuilder::Build() {
  JniUniquePtr<jobject> result = CallObjectMethod(build_method_);
  if (JavaExceptionPrintClear(Env())) {
    return nullptr;
  }

  return std::make_unique<UrlRequest>(Env(),
                                      LocalToGlobalRef(std::move(result)));
}

UrlRequest::UrlRequest(JNIEnv* env, JniUniquePtr<jobject> url_request)
    : JavaWrapper(env, std::move(url_request), "org/chromium/net/UrlRequest") {
  start_method_ = GetMethodHandle("start", "()V");
  cancel_method_ = GetMethodHandle("cancel", "()V");

  JavaExceptionPrintClear(Env());

  
  
}

void UrlRequest::Start() {
  CallVoidMethod(start_method_);
  JavaExceptionPrintClear(Env());
}

void UrlRequest::Cancel() {
  CallVoidMethod(cancel_method_);
  JavaExceptionPrintClear(Env());
}

// C++ side of the JNI bridge for Cronet callbacks.
class NativeCronetCallback {
 public:
  explicit NativeCronetCallback(const imp::Future<absl::Cord>& future)
      : future_(future) {}

  void OnSucceeded(const absl::Cord& bytes) {
    if (!future_.Ready()) {
      future_.Return(bytes);
    }
  }

  void OnFailed(const std::string& error_message) {
    if (!future_.Ready()) {
      future_.Return(absl::InternalError(error_message));
    }
  }

  void OnCanceled() {
    if (!future_.Ready()) {
      future_.Return(absl::CancelledError("Request was cancelled."));
    }
  }

 private:
  imp::Future<absl::Cord> future_;
};

// C++ wrapper for com.google.ar.imp.core.net.CronetCallback
class CronetCallback : public JavaWrapper {
 public:
  explicit CronetCallback(JNIEnv* env, NativeCronetCallback* native_peer);
};

CronetCallback::CronetCallback(JNIEnv* env, NativeCronetCallback* native_peer)
    : JavaWrapper(env, "com/google/ar/imp/core/net/CronetCallback", "(J)V",
                  reinterpret_cast<jlong>(native_peer)) {}

// CronetUrlLoader is a UrlLoader implementation that uses Cronet to load URLs.
class CronetUrlLoader : public UrlLoader {
 public:
  explicit CronetUrlLoader(
      const Context& context,
      std::unique_ptr<UrlRequestFactory> url_request_factory);
  ~CronetUrlLoader() override;

  Future<absl::Cord> LoadUrl(const std::string& url) override;

 private:
  int AddActiveRequest(std::unique_ptr<UrlRequestHandle> request);
  void RemoveActiveRequest(int request_id);
  UrlRequestHandle* GetActiveRequest(int request_id);

  const Context& context_;
  AndroidResourceLoader android_resource_loader_;
  std::unique_ptr<CronetEngine> cronet_engine_;
  JNIEnv* const env_;
  const JniUniquePtr<jobject> executor_;
  std::unique_ptr<UrlRequestFactory> url_request_factory_;
  absl::Mutex active_requests_mutex_;
  absl::flat_hash_map<int, std::unique_ptr<UrlRequestHandle>> ABSL_GUARDED_BY(
      active_requests_mutex_) active_requests_;
  uint32_t ABSL_GUARDED_BY(active_requests_mutex_) request_id_counter_ = 0;

  friend class CronetUrlRequestFactory;
};

CronetUrlLoader::~CronetUrlLoader() {
  for (const auto& [request_id, request] : active_requests_) {
    request->Cancel();
  }
}

// Default factory for creating UrlRequest objects. This factory will use
// Cronet to make URL requests.
class CronetUrlRequestFactory : public UrlRequestFactory {
 public:
  explicit CronetUrlRequestFactory(CronetUrlLoader& cronet_url_loader)
      : cronet_url_loader_(cronet_url_loader) {}

  std::unique_ptr<UrlRequestHandle> CreateRequest(
      absl::string_view url, JNIEnv* env,
      imp::Future<absl::Cord>& future) override;

 private:
  CronetUrlLoader& cronet_url_loader_;
};

std::unique_ptr<UrlRequestHandle> CronetUrlRequestFactory::CreateRequest(
    absl::string_view url, JNIEnv* env, imp::Future<absl::Cord>& future) {
  const UrlLoader::Config& config = cronet_url_loader_.GetConfig();
  auto native_callback = std::make_unique<NativeCronetCallback>(future);

  // Create a CronetCallback Java object.
  auto cronet_callback =
      std::make_unique<CronetCallback>(env, native_callback.get());

  // Create UrlRequestBuilder.
  std::unique_ptr<UrlRequestBuilder> url_request_builder =
      cronet_url_loader_.cronet_engine_->NewUrlRequestBuilder(
          url, WrapJni(env, cronet_callback->Reference()),
          CloneRef(cronet_url_loader_.executor_));

  for (const auto& [key, value] : config.request_headers) {
    url_request_builder->AddHeader(key, value);
  }

  std::unique_ptr<UrlRequestHandle> url_request = url_request_builder->Build();

  // Keep objects alive until the future is resolved.
  future.DependsOn(std::move(native_callback));
  future.DependsOn(std::move(cronet_callback));

  return url_request;
}

CronetUrlLoader::CronetUrlLoader(
    const Context& context,
    std::unique_ptr<UrlRequestFactory> url_request_factory)
    : context_(context),
      android_resource_loader_(context),
      env_(context.GetJniEnv()),
      executor_(LocalToGlobalRef(CloneRef(env_, context.GetExecutor()))),
      url_request_factory_(std::move(url_request_factory)) {
  // Create a CronetEngine object
  CronetEngineBuilder cronet_engine_builder(
      env_, CloneRef(env_, context_.GetActivityContext()));
  cronet_engine_ = cronet_engine_builder.Build();

  if (!url_request_factory_) {
    url_request_factory_ = std::make_unique<CronetUrlRequestFactory>(*this);
  }

#if ENABLE_NETLOG
  std::string netlog_file;
  JniUniquePtr<jobject> activity_context =
      CloneRef(env_, context_.GetActivityContext());

  JniUniquePtr<jclass> context_class =
      FindClass(env_, "android/content/Context");
  jmethodID get_cache_dir_method =
      env_->GetMethodID(context_class.get(), "getCacheDir", "()Ljava/io/File;");
  if (JavaExceptionPrintClear(env_)) {
    
    return;
  }

  JniUniquePtr<jobject> cache_dir_file = WrapJni(
      env_,
      env_->CallObjectMethod(activity_context.get(), get_cache_dir_method));
  if (JavaExceptionPrintClear(env_)) {
    
    return;
  };

  JniUniquePtr<jclass> file_class = FindClass(env_, "java/io/File");
  jmethodID get_absolute_path_method = env_->GetMethodID(
      file_class.get(), "getAbsolutePath", "()Ljava/lang/String;");
  if (JavaExceptionPrintClear(env_)) {
    
    return;
  };

  JniUniquePtr<jstring> cache_dir_path =
      WrapJni(env_, (jstring)env_->CallObjectMethod(cache_dir_file.get(),
                                                    get_absolute_path_method));
  if (JavaExceptionPrintClear(env_)) {
    
    return;
  };

  netlog_file = GetString(env_, cache_dir_path.get()) + "/netlog.json";
  cronet_engine_->StartNetLogToFile(netlog_file, true);
#endif  // ENABLE_NETLOG
}

int CronetUrlLoader::AddActiveRequest(
    std::unique_ptr<UrlRequestHandle> request) {
  absl::MutexLock lock(active_requests_mutex_);
  int request_id = ++request_id_counter_;
  active_requests_[request_id] = std::move(request);
  return request_id;
}

void CronetUrlLoader::RemoveActiveRequest(int request_id) {
  absl::MutexLock lock(active_requests_mutex_);
  active_requests_.erase(request_id);
}

UrlRequestHandle* CronetUrlLoader::GetActiveRequest(int request_id) {
  absl::MutexLock lock(active_requests_mutex_);
  auto it = active_requests_.find(request_id);
  if (it == active_requests_.end()) {
    return nullptr;
  }
  return it->second.get();
}

// UrlRequest
Future<absl::Cord> CronetUrlLoader::LoadUrl(const std::string& string_url) {
  if (AndroidResourceLoader::IsAndroidSpecialUrl(string_url)) {
    return android_resource_loader_.LoadUrl(string_url);
  }

  imp::Future<absl::Cord> future;
  std::unique_ptr<UrlRequestHandle> url_request =
      url_request_factory_->CreateRequest(string_url, env_, future);

  url_request->Start();
  int request_id = AddActiveRequest(std::move(url_request));

  return future.Then([this, request_id](absl::StatusOr<absl::Cord> result) {
    if (!result.ok() &&
        result.status().code() == absl::StatusCode::kCancelled) {
      UrlRequestHandle* request = GetActiveRequest(request_id);
      if (request) {
        request->Cancel();
      }
    }
    RemoveActiveRequest(request_id);
    return result;
  });
}

}  // namespace

std::unique_ptr<UrlLoader> CreateCronetLoader(
    const Context& context,
    std::unique_ptr<UrlRequestFactory> url_request_factory) {
  return std::make_unique<CronetUrlLoader>(context,
                                           std::move(url_request_factory));
}

}  // namespace resources
}  // namespace imp

extern "C" {

JNIEXPORT void JNICALL
Java_com_google_ar_imp_core_net_CronetCallback_nativeOnSucceeded(
    JNIEnv* env, jobject obj, jlong native_peer, jbyteArray data) {
  imp::resources::NativeCronetCallback* callback =
      reinterpret_cast<imp::resources::NativeCronetCallback*>(native_peer);
  
  callback->OnSucceeded(imp::ByteArrayToCord(env, data));
}

JNIEXPORT void JNICALL
Java_com_google_ar_imp_core_net_CronetCallback_nativeOnFailed(
    JNIEnv* env, jobject obj, jlong native_peer, jstring error_message) {
  imp::resources::NativeCronetCallback* callback =
      reinterpret_cast<imp::resources::NativeCronetCallback*>(native_peer);
  
  callback->OnFailed(imp::GetString(env, error_message));
}

JNIEXPORT void JNICALL
Java_com_google_ar_imp_core_net_CronetCallback_nativeOnCanceled(
    JNIEnv* env, jobject obj, jlong native_peer) {
  imp::resources::NativeCronetCallback* callback =
      reinterpret_cast<imp::resources::NativeCronetCallback*>(native_peer);
  
  callback->OnCanceled();
}

}  // extern "C"
