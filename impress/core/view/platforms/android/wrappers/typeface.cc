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

#include "core/view/platforms/android/wrappers/typeface.h"

#include <jni.h>

#include <string>
#include <utility>

#include "absl/strings/string_view.h"
#include "core/common/jni_helpers.h"

namespace imp::android {

Typeface::Typeface(JNIEnv* env, absl::string_view family_name, jint style)
    : JavaWrapper(env, "android/graphics/Typeface") {
  font_family_name_ = std::string(family_name);

  auto create_method_handle = GetStaticMethodHandle(
      "create", "(Ljava/lang/String;I)Landroid/graphics/Typeface;");
  JniUniquePtr<jstring> family_name_string = ToJniString(Env(), family_name);

  JniUniquePtr<jobject> typeface =
      WrapJni(Env(), CallStaticObjectMethod(create_method_handle,
                                            family_name_string.get(), style));
  SetSelf(LocalToGlobalRef(std::move(typeface)));
}

Typeface::Typeface(JNIEnv* env, Typeface& family, jint weight, jboolean italic)
    : JavaWrapper(env, "android/graphics/Typeface") {
  font_family_name_ = family.GetFontFamilyName();

  auto create_method_handle = GetStaticMethodHandle(
      "create", "(Landroid/graphics/Typeface;IZ)Landroid/graphics/Typeface;");

  JniUniquePtr<jobject> typeface = WrapJni(
      Env(), CallStaticObjectMethod(create_method_handle,
                                    family.WeakReference(), weight, italic));
  SetSelf(LocalToGlobalRef(std::move(typeface)));
}

Typeface::Typeface(JNIEnv* env, jint style)
    : JavaWrapper(env, "android/graphics/Typeface") {
  auto default_field_handle =
      GetStaticFieldHandle("DEFAULT", "Landroid/graphics/Typeface;");
  JniUniquePtr<jobject> default_typeface =
      WrapJni(Env(), GetStaticObjectField(default_field_handle));

  auto create_method_handle = GetStaticMethodHandle(
      "create", "(Landroid/graphics/Typeface;I)Landroid/graphics/Typeface;");

  JniUniquePtr<jobject> typeface =
      WrapJni(Env(), CallStaticObjectMethod(create_method_handle,
                                            default_typeface.get(), style));
  SetSelf(LocalToGlobalRef(std::move(typeface)));
}

Typeface::Typeface(JNIEnv* env, jobject j_typeface)
    : JavaWrapper(env, j_typeface) {}

absl::string_view Typeface::GetFontFamilyName() const {
  return font_family_name_;
}

}  // namespace imp::android
