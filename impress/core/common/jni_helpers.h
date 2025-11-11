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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_JNI_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_JNI_HELPERS_H_

#include <jni.h>

#include <cstddef>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include "absl/base/attributes.h"
#include "absl/log/check.h"
#include "absl/strings/cord.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/types/variant.h"
#include "core/common/buffer_access.h"
#include "core/common/context.h"
#include "core/common/jni_context.h"
#include "core/common/optional_error.h"
#include "core/common/typed_id.h"
#include "core/common/typed_vector.h"

namespace imp {
namespace android {
// Add a debug method for dumping the local reference table in Android devices.
void DumpLocalReferenceTable(JNIEnv* env);

}  // namespace android

namespace details {

#if defined(_WINDOWS) || defined(_WIN32)
#define IMP_JNI __declspec(dllexport) JNIEXPORT
#else
#define IMP_JNI JNIEXPORT
#endif  // defined(_WINDOWS) || defined(_WIN32)

#define THROW_IF_ERROR(env, expr)                                    \
  switch (0)                                                         \
  case 0:                                                            \
  default:                                                           \
    if (imp::OptionalError status = (expr); !status.ok())            \
    ((env)->ThrowNew((env)->FindClass("java/lang/RuntimeException"), \
                     std::string(status.message()).c_str()))

// Compile-time helper function for use in static_assert.  Returns true iff
// T appears in the list of Candidates.
// TODO: add tests if needed.
template <class T, class... Candidates>
constexpr bool IsFirstOneOfRest() {
  bool pairs[] = {std::is_same<T, Candidates>::value...};
  for (bool p : pairs)
    if (p) return true;
  return false;
}

}  // namespace details

// Create a type that will not compile unless T occurs in the variable set of
// types Allowlist.  Client modules use the template (with the allowlist
// that makes sense for their module) by wrapping calls to its static methods,
// e.g.:
//
// template <class T>
// inline jlong ToJava(T* pointer) {
//   return JniAllowlist<T, ModuleType1, ModuleType2>::ToJava(pointer);
// }
// TODO: add tests if needed.
template <class T, class... Allowlist>
class JniAllowlist {
 public:
  static inline jlong ToJava(T* pointer) {
    static_assert(details::IsFirstOneOfRest<T, Allowlist...>(),
                  "Requested T not in declared Allowlist");
    return reinterpret_cast<intptr_t>(pointer);
  }

  static inline T* FromJava(jlong number) {
    static_assert(details::IsFirstOneOfRest<T, Allowlist...>(),
                  "Requested T not in declared Allowlist");
    return reinterpret_cast<T*>(number);
  }
};

// Helper type to prevent leaks.  Works like UniquePtr in that one must call
// Release() to revoke ownership.
// TODO: add tests if needed.
class JniObjectArray {
 public:
  JniObjectArray(JNIEnv* env, size_t length, jclass element_class,
                 jobject initial_element);
  ~JniObjectArray();
  jobjectArray Release();
  void Set(jsize index, jobject value);

 private:
  JNIEnv* env_;
  jobjectArray array_;
};

// If an exception has occurred, logs it with via abseil's IMP_LOG(imp::FATAL).
void AssertNoException(JNIEnv* env);

// Returns false if there is no pending exception on the calling thread.
// Otherwise, logs and clears the exception, and returns true.
// cf.
// (broken link)
bool JavaExceptionPrintClear(JNIEnv* env);

// Helper function to convert C++ strings to jstring.
ABSL_DEPRECATED("Use ToJniString instead")
jstring ToString(JNIEnv* env, const std::string& str);
// Helper function to convert absl::string_view to jstring.
ABSL_DEPRECATED("Use ToJniString instead")
jstring ToString(JNIEnv* env, absl::string_view view);
// Helper function to convert string views into string arrays.
jobjectArray ToStringArray(JNIEnv* env, std::vector<absl::string_view> views);

// Copy the contents of a BufferAccess into a jbyteArray.
jbyteArray ToByteArray(JNIEnv* env, const BufferAccess& access);

// Copy the contents of the string view into a jbyteArray.
jbyteArray ToByteArray(JNIEnv* env, absl::string_view str);

// Copy the contents of a jbyteArray into a BufferAccess.
BufferAccess FromByteArray(JNIEnv* env, jbyteArray byte_array);

// Copy the contents of a jbyteArray into an absl::Cord.
absl::Cord ByteArrayToCord(JNIEnv* env, jbyteArray byte_array);

// Emits a java.lang.RuntimeException with the contents of error.
void ThrowError(JNIEnv* env, const OptionalError& error);

// Emits a org.json.JSONException with the contents of error.
void ThrowJsonError(JNIEnv* env, const OptionalError& error);

// Wraps boilerplate for fetching a string from java.
std::string GetString(JNIEnv* env, jstring java_string);

jfieldID GetFieldID(JNIEnv* env, jclass clazz, const char* field_name,
                    const char* field_signature);

// Check whether the given object is a global or local ref and delete the
// ref appropriately.
void DeleteRef(JNIEnv* env, jobject object);

using JniType = absl::variant<jclass, jobject, jmethodID, jfieldID>;
using JniHandle = TypedId<JniType, int32_t>;

namespace details {

template <typename T>
class JniDeleter {
 public:
  // `env` shall be JNIEnv of the calling thread.
  JniDeleter(JNIEnv* env) : context_(env) {}

  // If `p` is local reference, it shall be local to the calling thread.
  void operator()(T p) { DeleteRef(env(), p); }

  // Returns the JNIEnv of the calling thread which may be different from the
  // one used to construct the deleter.
  JNIEnv* env() const { return context_.GetJniEnv(); }

 private:
  JniContext context_;
};

}  // namespace details

// There are libraries providing a similar functionality as JniUniquePtr below,
// but they either don't have external equivalents, or those that do are part of
// larger libraries that seem to be too much work to bring in for a small change
// like this.

// A unique_ptr for managing raw jni references. Jni references held by this
// class will be automatically deleted when the JniUniquePtr is destroyed.
template <typename T>
using JniUniquePtr =
    std::unique_ptr<std::remove_pointer_t<T>, details::JniDeleter<T>>;

// Wraps a raw jni reference in a JniUniquePtr, handing the responsibility of
// deleting the reference to this class. The reference is automatically deleted
// when the JniUniquePtr is destroyed.
template <typename T>
JniUniquePtr<T> WrapJni(JNIEnv* env, T jni_object) {
  return JniUniquePtr<T>(jni_object, details::JniDeleter<T>(env));
}

// Creates a smart pointer with a cloned local reference to the given object.
template <typename T>
JniUniquePtr<T> CloneRef(JNIEnv* env, T ref) {
  return WrapJni(env, static_cast<T>(env->NewLocalRef(ref)));
}

JniUniquePtr<jstring> ToJniString(JNIEnv* env, const std::string& str);
JniUniquePtr<jstring> ToJniString(JNIEnv* env, absl::string_view view);

JniUniquePtr<jclass> FindClass(JNIEnv* env, const char* class_path);

ABSL_DEPRECATED("Use FindClass with an explicit path instead")
JniUniquePtr<jclass> GetObjectClass(JNIEnv* env, jobject object);

JniUniquePtr<jbyteArray> CreateJniByteArray(JNIEnv* env, size_t length);
JniUniquePtr<jintArray> CreateJniIntArray(JNIEnv* env, size_t length);
JniUniquePtr<jlongArray> CreateJniLongArray(JNIEnv* env, size_t length);
JniUniquePtr<jfloatArray> CreateJniFloatArray(JNIEnv* env, size_t length);
JniUniquePtr<jbooleanArray> CreateJniBooleanArray(JNIEnv* env, size_t length);
JniUniquePtr<jobjectArray> CreateJniObjectArray(JNIEnv* env, size_t length,
                                                jclass clazz, jobject initial);
JniUniquePtr<jstring> CreateJniString(JNIEnv* env, const std::string& str);

// Takes a local reference, creates and returns a global reference to the same
// object. Then releases the given local reference.
template <typename T>
JniUniquePtr<T> LocalToGlobalRef(JniUniquePtr<T> local_ref) {
  JNIEnv* env = local_ref.get_deleter().env();
  jobject global_ref = env->NewGlobalRef(local_ref.get());
  return WrapJni(env, static_cast<T>(global_ref));
}

// Takes local or global reference, creates and returns a new local reference.
template <typename T>
JniUniquePtr<T> CloneRef(const JniUniquePtr<T>& ref) {
  JNIEnv* env = ref.get_deleter().env();
  jobject new_ref = env->NewLocalRef(ref.get());
  return WrapJni(env, static_cast<T>(new_ref));
}

// The key point of this function is to create a global ref and then release
// the local ref immediately.
JniUniquePtr<jbyteArray> CreateByteArrayGlobalRef(JNIEnv* env, size_t length);

// Helper to get a JniType (variant) value.
template <typename T>
T GetJniTypeOrNull(const JniType& variant) {
  if (absl::holds_alternative<T>(variant)) {
    return absl::get<T>(variant);
  }
  return nullptr;
}

// Base class for representing Java classes in native code.
// TODO: Add memory leak test
// TODO: Set ClassLoader when JNI context is attached. The Java
// ClassLoader is thread-local, so using a background thread to load a custom
// Java Class will cause a ClassNotFoundException since it will use the system
// ClassLoader by default.
class JavaWrapper {
 public:
  template <class... Args>
  JavaWrapper(const Context& context, const char* class_path,
              const char* init_signature, Args&&... init_args)
      : context_(context.GetJniEnv()) {
    JNIEnv* env = context_.GetJniEnv();
    JniUniquePtr<jclass> local_class_ref = FindClass(env, class_path);
    class_ = AddJniInfo(
        static_cast<jclass>(env->NewGlobalRef(local_class_ref.get())));
    class_path_ = class_path;
    if (init_signature) {
      jmethodID init = env->GetMethodID(Clazz(), "<init>", init_signature);
      init_ = AddJniInfo(init);
      JniUniquePtr<jobject> local_self_ref = WrapJni(
          env, env->NewObject(Clazz(), init, std::forward<Args>(init_args)...));
      self_ = AddJniInfo(env->NewGlobalRef(local_self_ref.get()));
    }
    JavaExceptionPrintClear(env);
  }

  template <class... Args>
  JavaWrapper(JNIEnv* env, const char* class_path, const char* init_signature,
              Args&&... init_args)
      : context_(env) {
    JNIEnv* jni_env = context_.GetJniEnv();
    JniUniquePtr<jclass> local_class_ref = FindClass(jni_env, class_path);
    class_ = AddJniInfo(
        static_cast<jclass>(env->NewGlobalRef(local_class_ref.get())));
    class_path_ = class_path;
    if (init_signature) {
      jmethodID init = env->GetMethodID(Clazz(), "<init>", init_signature);
      init_ = AddJniInfo(init);
      JniUniquePtr<jobject> local_self_ref = WrapJni(
          env, env->NewObject(Clazz(), init, std::forward<Args>(init_args)...));
      self_ = AddJniInfo(env->NewGlobalRef(local_self_ref.get()));
    }
    JavaExceptionPrintClear(env);
  }

  JavaWrapper(JNIEnv* env, const char* class_path)
      : JavaWrapper(env, class_path, nullptr) {}

  // Wraps an existing java object in a global reference, ensures the reference
  // is freed when the wrapper is destroyed.
  ABSL_DEPRECATED(
      "Use ctor JavaWrapper(JNIEnv*, JniUniquePtr<jobject>, const char*) "
      "instead")
  JavaWrapper(JNIEnv* env, jobject object) : context_(env) {
    JniUniquePtr<jclass> local_class_ref =
        GetObjectClass(context_.GetJniEnv(), object);
    class_ = AddJniInfo(
        static_cast<jclass>(env->NewGlobalRef(local_class_ref.get())));

    SetSelf(env->NewGlobalRef(object));
    JavaExceptionPrintClear(env);
  }

  ABSL_DEPRECATED(
      "Use ctor JavaWrapper(JNIEnv*, JniUniquePtr<jobject>, const char*) "
      "instead")
  JavaWrapper(JNIEnv* env, jobject object, const char* class_path)
      : JavaWrapper(env, object) {
    class_path_ = class_path;

    JniUniquePtr<jclass> local_class_ref = FindClass(env, class_path);

    if (local_class_ref == nullptr) {
      IMP_LOG(imp::FATAL) << "type passed to JavaWrapper::ctor is not found, perhaps it"
                 << " was proguarded away. class_path=" << class_path
                 << ", object's type=" << GetObjectClassName(env, object);
    } else {
      
    }
  }

  // The benefit of this ctor is that the ownership of the passed reference is
  // explicitly tracked by the smart pointer. Concretely, the reference will be
  // released after this ctor returns. This makes it impossible to forget to
  // release the reference.
  //
  // This ctor is the preferred way to initialize a JavaWrapper object.
  JavaWrapper(JNIEnv* env, JniUniquePtr<jobject> object, const char* class_path)
      : JavaWrapper(env, object.get(), class_path) {}

  JavaWrapper(JNIEnv* env) : context_(env) {}

  JavaWrapper(const JavaWrapper&) = delete;
  JavaWrapper(JavaWrapper&&) = delete;
  JavaWrapper& operator=(const JavaWrapper&) = delete;
  JavaWrapper& operator=(JavaWrapper&&) = delete;

  virtual ~JavaWrapper() {
    if (Self() != nullptr) {
      DeleteRef(Env(), Self());
    }
    if (Clazz() != nullptr) {
      DeleteRef(Env(), Clazz());
    }
    // Empty the handle.
    class_ = JniHandle();
    init_ = JniHandle();
    self_ = JniHandle();
  }

  // Release the jobject handle to java, use to release ownership to Java.
  jobject Release() {
    auto self_reference = Self();
    jobject ret = Env()->NewLocalRef(self_reference);
    if (self_reference) {
      DeleteRef(Env(), self_reference);
      jni_info_list_[self_] = jobject{nullptr};
    }
    return ret;
  }

  // Get the underlying object and increment the reference counter.
  jobject Reference() {
    jobject ret = Self();
    if (ret != nullptr) {
      return Env()->NewGlobalRef(ret);
    }
    return nullptr;
  }

  // Get the underlying object but don't increment the reference counter.
  jobject WeakReference() { return Self(); }

 protected:
  JNIEnv* Env() { return context_.GetJniEnv(); }
  jclass Clazz() {
    if (!class_) {
      return nullptr;
    }
    return ToClass(class_);
  }
  jmethodID Init() {
    if (!init_) {
      return nullptr;
    }
    return ToMethodID(init_);
  }
  jobject Self() {
    if (!self_) {
      return nullptr;
    }
    return ToObject(self_);
  }

  /* Create handles from JNI types. */

  JniHandle GetStaticFieldHandle(const char* name, const char* class_path) {
    return AddJniInfo(Env()->GetStaticFieldID(Clazz(), name, class_path));
  }

  JniHandle GetFieldHandle(const char* name, const char* class_path) {
    return AddJniInfo(Env()->GetFieldID(Clazz(), name, class_path));
  }

  JniHandle GetMethodHandle(const char* name, const char* signature) {
    return AddJniInfo(Env()->GetMethodID(Clazz(), name, signature));
  }

  JniHandle GetStaticMethodHandle(const char* name, const char* signature) {
    return AddJniInfo(Env()->GetStaticMethodID(Clazz(), name, signature));
  }

  /* From handle to JNI. */

  jfieldID ToFieldID(JniHandle handle) {
    return CastToJNIType<jfieldID>(handle);
  }

  jobject ToObject(JniHandle handle) { return CastToJNIType<jobject>(handle); }

  jmethodID ToMethodID(JniHandle handle) {
    return CastToJNIType<jmethodID>(handle);
  }

  jclass ToClass(JniHandle handle) { return CastToJNIType<jclass>(handle); }

  /* Get a Java value or call a method. */

  jobject GetStaticObjectField(JniHandle handle) {
    return Env()->GetStaticObjectField(Clazz(), ToFieldID(handle));
  }

  jobject GetObjectField(JniHandle handle) {
    return Env()->GetObjectField(Clazz(), ToFieldID(handle));
  }

  jint GetStaticIntField(JniHandle handle) {
    return Env()->GetStaticIntField(Clazz(), ToFieldID(handle));
  }

  template <class... Args>
  void CallVoidMethod(JniHandle handle, Args&&... args) {
    if (!handle) {
      ThrowError(Env(), Error("Jni handle is invalid."));
    }
    jmethodID id = ToMethodID(handle);
    Env()->CallVoidMethod(Self(), id, std::forward<Args>(args)...);
  }

  template <class... Args>
  void CallStaticVoidMethod(JniHandle handle, Args&&... args) {
    if (!handle) {
      ThrowError(Env(), Error("Jni handle is invalid."));
    }
    jmethodID id = ToMethodID(handle);
    Env()->CallStaticVoidMethod(Clazz(), id, std::forward<Args>(args)...);
  }

  template <class... Args>
  jlong CallLongMethod(JniHandle handle, Args&&... args) {
    if (!handle) {
      ThrowError(Env(), Error("Jni handle is invalid."));
    }
    jmethodID id = ToMethodID(handle);
    return Env()->CallLongMethod(Self(), id, std::forward<Args>(args)...);
  }

  template <class... Args>
  jfloat CallFloatMethod(JniHandle handle, Args&&... args) {
    if (!handle) {
      ThrowError(Env(), Error("Jni handle is invalid."));
    }
    jmethodID id = ToMethodID(handle);
    return Env()->CallFloatMethod(Self(), id, std::forward<Args>(args)...);
  }

  template <class... Args>
  jboolean CallBooleanMethod(JniHandle handle, Args&&... args) {
    if (!handle) {
      ThrowError(Env(), Error("Jni handle is invalid."));
    }
    jmethodID id = ToMethodID(handle);
    return Env()->CallBooleanMethod(Self(), id, std::forward<Args>(args)...);
  }

  template <class... Args>
  jint CallIntMethod(JniHandle handle, Args&&... args) {
    if (!handle) {
      ThrowError(Env(), Error("Jni handle is invalid."));
    }
    jmethodID id = ToMethodID(handle);
    return Env()->CallIntMethod(Self(), id, std::forward<Args>(args)...);
  }

  template <class... Args>
  jint CallStaticIntMethod(JniHandle handle, Args&&... args) {
    if (!handle) {
      ThrowError(Env(), Error("Jni handle is invalid."));
    }
    jmethodID id = ToMethodID(handle);
    return Env()->CallStaticIntMethod(Clazz(), id, std::forward<Args>(args)...);
  }

  // Calls a Java method that returns a Java string and converts it to an
  // std::string so the Java string reference can be released.
  template <class... Args>
  std::string CallStringMethod(JniHandle handle, Args&&... args) {
    if (!handle) {
      ThrowError(Env(), Error("Jni handle is invalid."));
    }
    jmethodID id = ToMethodID(handle);
    jobject java_string =
        Env()->CallObjectMethod(Self(), id, std::forward<Args>(args)...);
    std::string str = GetString(Env(), (jstring)java_string);
    Env()->DeleteLocalRef(java_string);
    return str;
  }

  template <class... Args>
  jobject CallObjectMethod(JniHandle handle, Args&&... args) {
    if (!handle) {
      ThrowError(Env(), Error("Jni handle is invalid."));
    }
    jmethodID id = ToMethodID(handle);
    return Env()->CallObjectMethod(Self(), id, std::forward<Args>(args)...);
  }

  template <class... Args>
  jobject CallStaticObjectMethod(JniHandle handle, Args&&... args) {
    if (!handle) {
      ThrowError(Env(), Error("Jni handle is invalid."));
    }
    jmethodID id = ToMethodID(handle);
    return Env()->CallStaticObjectMethod(Clazz(), id,
                                         std::forward<Args>(args)...);
  }

  // Adapted from
  // (broken link)
  template <class... Args>
  jbyteArray CallByteArrayMethod(JniHandle handle, Args&&... args) {
    auto env = Env();
    jmethodID id = ToMethodID(handle);
    return (jbyteArray)env->CallObjectMethod(Self(), id,
                                             std::forward<Args>(args)...);
  }

  template <class... Args>
  jintArray CallIntArrayMethod(JniHandle handle, Args&&... args) {
    auto env = Env();
    jmethodID id = ToMethodID(handle);
    return (jintArray)env->CallObjectMethod(Self(), id,
                                            std::forward<Args>(args)...);
  }

  template <class... Args>
  jfloatArray CallFloatArrayMethod(JniHandle handle, Args&&... args) {
    auto env = Env();
    jmethodID id = ToMethodID(handle);
    return (jfloatArray)env->CallObjectMethod(Self(), id,
                                              std::forward<Args>(args)...);
  }

  // Helper for adding jni info.
  JniHandle AddJniInfo(const JniType& jni_info) {
    return jni_info_list_.Append<JniHandle>(jni_info);
  }

  template <typename T>
  T CastToJNIType(JniHandle handle) {
    if (!jni_info_list_.IsValid(handle)) {
      ThrowError(Env(), Error("No JNI info for handle (out of range)."));
    }
    return GetJniTypeOrNull<T>(jni_info_list_[handle]);
  }

  // Setter for descendant classes to use when setting up a JavaWrapper with an
  // already-constructed jobject.
  void SetSelf(jobject self) {
    if (Self() != nullptr) {
      DeleteRef(Env(), Self());
    }

    self_ = AddJniInfo(self);
  }

  void SetSelf(JniUniquePtr<jobject> self) { SetSelf(self.release()); }

 private:
  static std::string GetObjectClassName(JNIEnv* env, jobject object);

 protected:
  std::string class_path_;
  JniContext context_;
  JniHandle class_;

 private:
  TypedVector<JniType> jni_info_list_;
  JniHandle init_;
  JniHandle self_;
};

template <typename T>
class JavaProtoEnumWrapper : public imp::JavaWrapper {
 public:
  template <class... Args>
  explicit JavaProtoEnumWrapper(Args&&... args)
      : imp::JavaWrapper(std::forward<Args>(args)...) {
    values_method_ = GetStaticMethodHandle(
        "values", absl::StrCat("()[L", class_path_, ";").c_str());
    ordinal_method_ = GetMethodHandle("ordinal", "()I");
  }

  T FromEnum(jobject enum_value) {
    return static_cast<T>(
        Env()->CallIntMethod(enum_value, ToMethodID(ordinal_method_)));
  }

  jobject GetEnum(T message) {
    return Env()->GetObjectArrayElement(
        static_cast<jobjectArray>(CallStaticObjectMethod(values_method_)),
        message);
  }

 private:
  JniHandle values_method_;
  JniHandle ordinal_method_;
};

// Helper for wrapping Java enums over the JNI.
// Uses a C++ enum class to correspond to the Java enum as the template type.
template <typename T>
class JavaEnumWrapper : public imp::JavaWrapper {
 public:
  template <class... Args>
  explicit JavaEnumWrapper(Args&&... args)
      : imp::JavaWrapper(std::forward<Args>(args)...) {
    values_method_ = GetStaticMethodHandle(
        "values", absl::StrCat("()[L", class_path_, ";").c_str());
    ordinal_method_ = GetMethodHandle("ordinal", "()I");
  }

  T FromEnum(jobject enum_value) {
    return static_cast<T>(
        Env()->CallIntMethod(enum_value, ToMethodID(ordinal_method_)));
  }

  jobject GetEnum(T message) {
    return Env()->GetObjectArrayElement(
        static_cast<jobjectArray>(CallStaticObjectMethod(values_method_)),
        static_cast<jsize>(message));
  }

 private:
  JniHandle values_method_;
  JniHandle ordinal_method_;
};

// Helper for working with ArrayList<> class in Java.
// TODO: add tests if needed.
template <class T>
class JavaArrayList {
 public:
  // Conversion method to transform from native to java type.
  using NativeToJObject = std::function<jobject(JNIEnv* env, const T& value)>;

  // Clean up global references.
  ~JavaArrayList() {
    if (jni_info_.self) {
      jni_info_.env->DeleteGlobalRef(jni_info_.self);
    }
    if (jni_info_.clazz) {
      jni_info_.env->DeleteGlobalRef(jni_info_.clazz);
    }
    jni_info_.self = nullptr;
    jni_info_.clazz = nullptr;
  }

  // Create a new ArrayList<>.
  static JavaArrayList Create(JNIEnv* env, NativeToJObject method) {
    JavaArrayList array_list = Wrap(env, nullptr, method);
    array_list.jni_info_.self =
        env->NewGlobalRef(env->NewObject(array_list.jni_info_.clazz,          //
                                         array_list.jni_info_.method_init));  //
    return array_list;
  }

  // Wrap an existing ArrayList<> passed from Java.
  static JavaArrayList Wrap(JNIEnv* env,                   //
                            jobjectArray java_array_list,  //
                            NativeToJObject method) {      //
    jclass clazz = static_cast<jclass>(
        env->NewGlobalRef(env->FindClass("java/util/ArrayList")));
    return JavaArrayList(JniInfo{
        env,
        clazz,
        java_array_list ? env->NewGlobalRef(java_array_list) : nullptr,
        env->GetMethodID(clazz, "<init>", "()V"),
        env->GetMethodID(clazz, "add", "(Ljava/lang/Object;)Z"),
        env->GetMethodID(clazz, "clear", "()V"),
        method,
    });
  }

  // Add an element to the array.
  void Add(const T& value) {
    auto env = jni_info_.env;
    jobject element = jni_info_.native_to_java_method(env, value);
    env->CallBooleanMethod(jni_info_.self,        //
                           jni_info_.method_add,  //
                           element);              //
    DeleteRef(env, element);
  }

  // Clear the list.
  void Clear() {
    jni_info_.env->CallVoidMethod(jni_info_.self, jni_info_.method_clear);
  }

  // Release ownership of object.
  jobject Release() {
    jobject ret = jni_info_.env->NewLocalRef(jni_info_.self);
    jni_info_.env->DeleteGlobalRef(jni_info_.self);
    jni_info_.self = nullptr;
    return ret;
  }

 private:
  struct JniInfo {
    JNIEnv* env = nullptr;
    jclass clazz = nullptr;
    jobject self;
    jmethodID method_init;
    jmethodID method_add;
    jmethodID method_clear;
    NativeToJObject native_to_java_method;
  } jni_info_;

  explicit JavaArrayList(JniInfo&& jni) : jni_info_(std::move(jni)) {}
};

// A RAII helper that creates a JNI global reference and then deletes it when it
// goes out of scope. Usage is:
// ScopedGlobalRef<jobject> scoped_global(context, local_jobject);
template <typename T>
class ScopedGlobalRef {
 public:
  ScopedGlobalRef(const Context& context, T local_ref)
      : context_(context),
        global_ref_(context_.GetJniEnv()->NewGlobalRef(local_ref)) {}

  ~ScopedGlobalRef() {
    if (global_ref_ != nullptr) {
      context_.GetJniEnv()->DeleteGlobalRef(global_ref_);
    }
  }

  ScopedGlobalRef(ScopedGlobalRef&& other)
      : context_(other.context_), global_ref_(other.global_ref_) {
    other.global_ref_ = nullptr;
  }

  T get() { return global_ref_; }

 private:
  const Context& context_;
  T global_ref_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_JNI_HELPERS_H_
