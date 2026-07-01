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

#include "core/common/jni_context.h"

#include <jni.h>

#include <atomic>
#include <cassert>
#include <string>

#include "absl/algorithm/container.h"
#include "core/common/log.h"

#ifdef __ANDROID__
#include <pthread.h>
#endif

// For some reason, the signature of AttachCurrentThread is different.
#ifdef __ANDROID__
#define JNI_PTR JNIEnv
#else  // __ANDROID__
#define JNI_PTR void
#endif  // __ANDROID__

namespace imp {
namespace {

// Once a valid JavaVM has been set, it should never be reset or changed.
// However, as it may be accessed from multiple threads, access needs to be
// synchronized.
std::atomic<JavaVM*> g_jvm{nullptr};
std::atomic<jint> g_jni_version{JNI_VERSION_1_6};

// These two are needed when loading custom Java classes in a background thread.
// This is because by default JVM assigns a system default ClassLoader to the
// background thread.
std::atomic<jobject> g_class_loader{nullptr};
std::atomic<jmethodID> g_load_class_method_id{nullptr};

// Attaches the current thread to the JVM with its pthread name.
JNIEnv* AttachCurrentThreadToJvm(JavaVM* jvm) {
  JNIEnv* env = nullptr;
#ifdef __ANDROID__
  JavaVMAttachArgs args;
  args.version = g_jni_version.load();
  args.group = nullptr;
  args.name = nullptr;
  if (__builtin_available(android 26, *)) {
    char threadName[16];
    if (pthread_getname_np(pthread_self(), threadName, sizeof(threadName)) ==
        0) {
      args.name = threadName;
    }
  }
  if (jvm->AttachCurrentThread(reinterpret_cast<JNI_PTR**>(&env), &args) ==
      JNI_OK) {
    return env;
  }
#else
  if (jvm->AttachCurrentThread(reinterpret_cast<JNI_PTR**>(&env), nullptr) ==
      JNI_OK) {
    return env;
  }
#endif
  return nullptr;
}

// This class is designed to only be stored as a thread_local, because it stores
// JNIEnv's that are only valid in the thread they are created in.
class ThreadLocalJniEnv {
 public:
  ~ThreadLocalJniEnv() {
    // If we call AttachCurrentThread(), we need to call DetachCurrentThread().
    // The stored result |attached_| indicates this.
    if (!attached_) {
      return;
    }

    JavaVM* jvm = g_jvm.load();
    if (!jvm) {
      return;
    }

    jint ret = jvm->DetachCurrentThread();
    assert(JNI_OK == ret);
    (void)ret;  // This prevents a build error for unused variable.
  }

  // If coming from a Java to C++ JNI call, prefer to pass in the provided
  // JNIEnv.
  void SetJniEnv(JNIEnv* env) { env_ = env; }

  // Returns the last JNIEnv Set() on this thread, or Attaches a new JNIEnv if
  // none available.  The attached JNIEnv will be automatically Detached when
  // the thread terminates.  Do not access this returned value from different
  // threads, get a new one for each thread.
  JNIEnv* GetJniEnv() {
    if (env_) {
      return env_;
    }
    if (attached_) {
      return attached_;
    }

    JavaVM* jvm = g_jvm.load();
    if (!jvm) {
      return nullptr;
    }

    // GetEnv() only returns if there is already an attached thread, which means
    // we don't need to DetachCurrentThread() afterwards.
    JNIEnv* env = nullptr;
    if (jvm->GetEnv(reinterpret_cast<void**>(&env), g_jni_version.load()) ==
        JNI_OK) {
      env_ = env;
      return env;
    }

    // If we call AttachCurrentThread(), we need to call DetachCurrentThread().
    // Store the result as |attached_| to indicate this.
    env = AttachCurrentThreadToJvm(jvm);
    if (env) {
      attached_ = env;
      return env;
    }

    return nullptr;
  }

  // Returns the last JNIEnv Set() on this thread, or Attaches a new JNIEnv if
  // none available.  The attached JNIEnv will be automatically Detached when
  // the thread terminates.  Do not access this returned value from different
  // threads, get a new one for each thread.
  JNIEnv* GetJniEnvConst() const {
    if (env_) {
      return env_;
    }
    if (attached_) {
      return attached_;
    }

    JavaVM* jvm = g_jvm.load();
    if (!jvm) {
      return nullptr;
    }

    // GetEnv() only returns if there is already an attached thread, which means
    // we don't need to DetachCurrentThread() afterwards.
    JNIEnv* env = nullptr;
    if (jvm->GetEnv(reinterpret_cast<void**>(&env), g_jni_version.load()) ==
        JNI_OK) {
      return env;
    }

    // If we call AttachCurrentThread(), we need to call DetachCurrentThread().
    // Store the result as |attached_| to indicate this.
    return AttachCurrentThreadToJvm(jvm);
  }

  // Returns whether the local thread is attached.
  bool IsAttached() const { return attached_ != nullptr; }

 private:
  JNIEnv* env_ = nullptr;
  JNIEnv* attached_ = nullptr;
};
thread_local ThreadLocalJniEnv tl_jni_env;

}  // namespace

void InitClassLoader(JNIEnv* env) {
  if (g_class_loader.load() != nullptr) {
    return;
  }

  // Get the current thread (which is main)
  jclass thread_class = env->FindClass("java/lang/Thread");
  jmethodID current_thread_method_id = env->GetStaticMethodID(
      thread_class, "currentThread", "()Ljava/lang/Thread;");
  jobject current_thread =
      env->CallStaticObjectMethod(thread_class, current_thread_method_id);

  // Get context ClassLoader from the thread
  jmethodID get_context_cl_method_id = env->GetMethodID(
      thread_class, "getContextClassLoader", "()Ljava/lang/ClassLoader;");
  jobject class_loader =
      env->CallObjectMethod(current_thread, get_context_cl_method_id);

  // Then store Global Ref and method ID
  if (class_loader != nullptr) {
    g_class_loader.store(env->NewGlobalRef(class_loader));
    jclass class_loader_class = env->FindClass("java/lang/ClassLoader");
    g_load_class_method_id.store(
        env->GetMethodID(class_loader_class, "loadClass",
                         "(Ljava/lang/String;)Ljava/lang/Class;"));
  }

  // Clean up
  env->DeleteLocalRef(current_thread);
  if (class_loader != nullptr) {
    env->DeleteLocalRef(class_loader);
  }
}

JniContext::JniContext(JNIEnv* env) {
  if (!env) {
    // Only support being created with a nullptr if there's already a g_jvm.
    assert(g_jvm.load() != nullptr);
    return;
  }

  // We aren't allowed to call any of the below methods if there's a pending
  // exception.
  if (env->ExceptionCheck()) {
    return;
  }

  JavaVM* vm = nullptr;
  env->GetJavaVM(&vm);

  JavaVM* old_jvm = g_jvm.exchange(vm);
  if (old_jvm && old_jvm != vm) {
    IMP_LOG(imp::ERROR) << "Only one valid Java VM should exist";
    return;
  }

  g_jni_version = env->GetVersion();

  InitClassLoader(env);
}

JniContext::JniContext(JavaVM* vm) {
  if (!vm) {
    return;
  }

  JavaVM* old_jvm = g_jvm.exchange(vm);
  if (old_jvm && old_jvm != vm) {
    IMP_LOG(imp::ERROR) << "Only one valid Java VM should exist";
    return;
  }

  g_jni_version = GetJniEnv()->GetVersion();

  InitClassLoader(GetJniEnv());
}

void JniContext::SetJniEnv(JNIEnv* env) { tl_jni_env.SetJniEnv(env); }

JNIEnv* JniContext::GetJniEnv() const { return tl_jni_env.GetJniEnv(); }
JNIEnv* JniContext::TryGetJniEnv() const { return tl_jni_env.GetJniEnvConst(); }

void JniContext::ResetJVM() {
  if (jobject old_loader = g_class_loader.exchange(nullptr)) {
    if (JNIEnv* env = tl_jni_env.GetJniEnvConst()) {
      env->DeleteGlobalRef(old_loader);
    }
  }
  g_load_class_method_id.exchange(nullptr);
  g_jvm.exchange(nullptr);
  tl_jni_env = ThreadLocalJniEnv();
}

jclass JniContext::FindClass(JNIEnv* env, const char* class_path) {
  // When loading custom Java classes, normally one can use env->FindClass.
  // However, this doesn't work when loading on a background thread as the Java
  // ClassLoader is local to the thread. When a new C++ thread is spawned and
  // attached to the JVM, Dalvik/ART assigns the base Zygote SystemClassLoader
  // to it, which only knows about system defaults and doesn't know about APK
  // custom classes. To mitigate this, we store a global reference to the main
  // thread's ClassLoader when initiating JniContext, and use
  // ClassLoader::loadClass. We still use env->FindClass for foreground loading,
  // as it avoids JNI reflection overhead and is slightly more performant than
  // ClassLoader::loadClass.

  if (env == nullptr || class_path == nullptr) {
    return nullptr;
  }

  jobject class_loader = g_class_loader.load();
  jmethodID load_class_mid = g_load_class_method_id.load();

  // When thread local is attached, meaning if it's a background thread, we use
  // Global ClassLoader::loadClass. Otherwise we use env->FindClass.
  if (tl_jni_env.IsAttached() && class_loader != nullptr &&
      load_class_mid != nullptr) {
    std::string class_name(class_path);
    // ClassLoader expects '.', unlike env->FindClass that expects '/'.
    absl::c_replace(class_name, '/', '.');
    jstring java_name = env->NewStringUTF(class_name.c_str());

    jobject clazz_obj =
        env->CallObjectMethod(class_loader, load_class_mid, java_name);
    env->DeleteLocalRef(java_name);
    if (env->ExceptionCheck()) {
      // We don't clear the exception here, as the caller is supposed to handle
      // the failure case.
      return nullptr;
    } else if (clazz_obj != nullptr) {
      return static_cast<jclass>(clazz_obj);
    }
  }

  // Fallback to standard JNI FindClass.
  if (env != nullptr) {
    jclass clazz = env->FindClass(class_path);
    return clazz;
  }

  return nullptr;
}

}  // namespace imp
