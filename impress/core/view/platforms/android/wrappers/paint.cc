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

#include "core/view/platforms/android/wrappers/paint.h"

#include <jni.h>

#include <memory>
#include <utility>
#include <vector>

#include "absl/strings/string_view.h"
#include "core/common/context.h"
#include "core/common/jni_helpers.h"
#include "core/math/vec.h"
#include "core/view/platforms/android/wrappers/graphics_helpers.h"
#include "core/view/platforms/android/wrappers/rect.h"

namespace imp::android {

class AlignWrapper : public JavaEnumWrapper<Paint::Align> {
 public:
  explicit AlignWrapper(JNIEnv* env)
      : JavaEnumWrapper<Paint::Align>(env, "android/graphics/Paint$Align") {}
};

class StyleWrapper : public JavaEnumWrapper<Paint::Style> {
 public:
  explicit StyleWrapper(JNIEnv* env)
      : JavaEnumWrapper<Paint::Style>(env, "android/graphics/Paint$Style") {}
};

Paint::Paint(const Context& context)
    : JavaWrapper(context, "android/graphics/Paint", "()V"),
      clear_xfermode_(EmptyJniUniquePtr<jobject>(context.GetJniEnv())) {
  set_text_size_ = GetMethodHandle("setTextSize", "(F)V");
  set_letter_spacing_ = GetMethodHandle("setLetterSpacing", "(F)V");
  set_stroke_width_ = GetMethodHandle("setStrokeWidth", "(F)V");
  set_style_ = GetMethodHandle("setStyle", "(Landroid/graphics/Paint$Style;)V");
  set_text_align_ =
      GetMethodHandle("setTextAlign", "(Landroid/graphics/Paint$Align;)V");
  set_color_ = GetMethodHandle("setColor", "(I)V");
  set_typeface_ = GetMethodHandle(
      "setTypeface",
      "(Landroid/graphics/Typeface;)Landroid/graphics/Typeface;");
  set_anti_alias_ = GetMethodHandle("setAntiAlias", "(Z)V");
  get_text_bounds_ = GetMethodHandle(
      "getTextBounds", "(Ljava/lang/String;IILandroid/graphics/Rect;)V");
  measure_text_ = GetMethodHandle("measureText", "(Ljava/lang/String;)F");
  get_font_spacing_ = GetMethodHandle("getFontSpacing", "()F");
  ascent_ = GetMethodHandle("ascent", "()F");
  descent_ = GetMethodHandle("descent", "()F");
  get_font_metrics_ = GetMethodHandle("getFontMetrics",
                                      "()Landroid/graphics/Paint$FontMetrics;");
  get_text_widths_ =
      GetMethodHandle("getTextWidths", "(Ljava/lang/String;[F)I");

  JNIEnv* env = context.GetJniEnv();
  JniUniquePtr<jclass> blend_mode_class =
      FindClass(env, "android/graphics/PorterDuff$Mode");
  auto clear_mode_id = env->GetStaticFieldID(
      blend_mode_class.get(), "CLEAR", "Landroid/graphics/PorterDuff$Mode;");
  JniUniquePtr<jobject> clear_mode = WrapJni(
      env, env->GetStaticObjectField(blend_mode_class.get(), clear_mode_id));

  JniUniquePtr<jclass> xfermode_class =
      FindClass(env, "android/graphics/PorterDuffXfermode");
  jmethodID xfermode_ctor = env->GetMethodID(
      xfermode_class.get(), "<init>", "(Landroid/graphics/PorterDuff$Mode;)V");
  clear_xfermode_ = LocalToGlobalRef(WrapJni(
      env,
      env->NewObject(xfermode_class.get(), xfermode_ctor, clear_mode.get())));

  set_xfermode_ = GetMethodHandle(
      "setXfermode",
      "(Landroid/graphics/Xfermode;)Landroid/graphics/Xfermode;");
}

void Paint::SetTextSize(float text_size) {
  if (last_text_size_.has_value() && *last_text_size_ == text_size) {
    return;
  }
  CallVoidMethod(set_text_size_, text_size);
  last_text_size_ = text_size;
}

void Paint::SetLetterSpacing(float text_tracking) {
  if (last_text_tracking_.has_value() &&
      *last_text_tracking_ == text_tracking) {
    return;
  }
  CallVoidMethod(set_letter_spacing_, text_tracking);
  last_text_tracking_ = text_tracking;
}

void Paint::SetStrokeWidth(float stroke_width) {
  if (last_stroke_width_.has_value() && *last_stroke_width_ == stroke_width) {
    return;
  }
  CallVoidMethod(set_stroke_width_, stroke_width);
  last_stroke_width_ = stroke_width;
}

void Paint::SetStyle(Style style) {
  if (last_style_.has_value() && *last_style_ == style) {
    return;
  }
  StyleWrapper style_wrapper(Env());
  CallVoidMethod(set_style_, style_wrapper.GetEnum(style).get());
  last_style_ = style;
}

void Paint::SetTextAlign(Align align) {
  if (last_align_.has_value() && *last_align_ == align) {
    return;
  }
  AlignWrapper align_wrapper(Env());
  CallVoidMethod(set_text_align_, align_wrapper.GetEnum(align).get());
  last_align_ = align;
}

void Paint::SetColor(float4 color) {
  if (last_color_.has_value() && *last_color_ == color) {
    return;
  }
  CallVoidMethod(set_color_, ToColorInt(color));
  last_color_ = color;
}

void Paint::SetTypeface(jobject typeface) {
  if (last_typeface_.has_value() && *last_typeface_ == typeface) {
    return;
  }
  CallObjectMethod(set_typeface_, typeface);
  last_typeface_ = typeface;
}

void Paint::SetAntiAlias(bool anti_alias) {
  if (last_anti_alias_.has_value() && *last_anti_alias_ == anti_alias) {
    return;
  }
  CallVoidMethod(set_anti_alias_, anti_alias);
  last_anti_alias_ = anti_alias;
}

void Paint::SetXfermodeClear(bool clear) {
  if (last_xfermode_clear_.has_value() && *last_xfermode_clear_ == clear) {
    return;
  }
  CallObjectMethod(set_xfermode_, clear ? clear_xfermode_.get() : nullptr);
  last_xfermode_clear_ = clear;
}

std::unique_ptr<Rect> Paint::GetTextBounds(absl::string_view text) {
  auto rect = std::make_unique<Rect>(Env());

  jstring jtext = ToString(Env(), text);

  CallVoidMethod(get_text_bounds_, jtext, 0, Env()->GetStringLength(jtext),
                 rect->WeakReference());

  Env()->DeleteLocalRef(jtext);

  return rect;
}

float Paint::MeasureText(absl::string_view text) {
  return CallFloatMethod(measure_text_, ToString(Env(), text));
}

std::vector<float> Paint::GetTextWidths(absl::string_view text) {
  jfloatArray out_widths_array = Env()->NewFloatArray(text.size());
  int out_widths_count =
      CallIntMethod(get_text_widths_, ToString(Env(), text), out_widths_array);

  jfloat* out_widths_array_ptr =
      Env()->GetFloatArrayElements(out_widths_array, nullptr);

  std::vector<float> result;
  result.reserve(out_widths_count);
  for (int i = 0; i < out_widths_count; ++i) {
    result.push_back(out_widths_array_ptr[i]);
  }

  Env()->ReleaseFloatArrayElements(out_widths_array, out_widths_array_ptr, 0);

  return result;
}

float Paint::GetFontSpacing() { return CallFloatMethod(get_font_spacing_); }

float Paint::Ascent() { return CallFloatMethod(ascent_); }

float Paint::Descent() { return CallFloatMethod(descent_); }

std::unique_ptr<Paint::FontMetrics> Paint::GetFontMetrics() {
  return std::make_unique<Paint::FontMetrics>(
      Env(), CallObjectMethod(get_font_metrics_));
}

Paint::FontMetrics::FontMetrics(JNIEnv* env,
                                JniUniquePtr<jobject> j_font_metrics)
    : JavaWrapper(env, std::move(j_font_metrics),
                  "android/graphics/Paint$FontMetrics") {
  leading_ = GetFieldHandle("leading", "F");
}

float Paint::FontMetrics::Leading() {
  return Env()->GetFloatField(Self(), ToFieldID(leading_));
}

}  // namespace imp::android
