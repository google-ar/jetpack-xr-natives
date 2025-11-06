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

#include "core/canvas/android_glyph_source.h"

#include <jni.h>

#include <memory>
#include <utility>
#include <vector>

#include "absl/strings/string_view.h"
#include "core/canvas/android_glyph_advance.h"
#include "core/canvas/fonts/android_font_font_holder.h"
#include "core/canvas/fonts/font_holder.h"
#include "core/canvas/scoped_canvas.h"
#include "core/common/context.h"
#include "core/common/jni_helpers.h"
#include "core/math/vec.h"
#include "core/view/platforms/android/wrappers/canvas.h"
#include "core/view/platforms/android/wrappers/font.h"
#include "core/view/platforms/android/wrappers/paint.h"

// TODO: This feature adds a non-trivial amount of space to the
// binary, but is not critical to Impress's overall functionality. Blaze also
// doesn't offer a way to automatically pull in a JNI->Java interface's
// corresponding Java dependency, so clients need to explicitly specify the JVM
// dependency themselves. At the same time, we don't want to mysteriously crash
// every single client for missing this dependency, especially if they never
// even touch the glyph-specific APIs.
#define LOG_MISSING_DEPENDENCY_MESSAGE(level)                            \
  IMP_LOG(imp::level)                                                             \
      << "Error resolving symbol in the Impress Android glyph package. " \
         "Likely missing Android Java dependency on "                    \
         "\"//java/com/google/ar/imp/core/glyph\". " \
         "Also ensure that your proguard_specs list includes "           \
         "\"//java/com/google/android/apps/common/"                      \
         "proguard:annotations.pgcfg\"."

namespace imp {

class MethodWrapper : public JavaEnumWrapper<AndroidGlyphSource::Method> {
 public:
  explicit MethodWrapper(JNIEnv* env)
      : JavaEnumWrapper<AndroidGlyphSource::Method>(
            env, "com/google/ar/imp/core/glyph/GlyphSource$Method") {}
};

AndroidGlyphSource::AndroidGlyphSource(const Context& context, Method method)
    : JavaWrapper(context.GetJniEnv()) {
  const char* class_path = "com/google/ar/imp/core/glyph/GlyphSource";

  // This check should really be available in JavaWrapper, but the variadic
  // argument constructor makes adding any new signatures difficult.
  JNIEnv* env = context_.GetJniEnv();
  JniUniquePtr<jclass> local_class_ref = FindClass(env, class_path);
  if (env->ExceptionCheck()) {
    env->ExceptionClear();
    LOG_MISSING_DEPENDENCY_MESSAGE(ERROR);
    return;
  }
  class_ = AddJniInfo(LocalToGlobalRef(std::move(local_class_ref)).release());
  class_path_ = class_path;

  jmethodID init =
      env->GetMethodID(Clazz(), "<init>",
                       "(Lcom/google/ar/imp/core/glyph/GlyphSource$Method;)V");
  if (env->ExceptionCheck()) {
    env->ExceptionClear();
    LOG_MISSING_DEPENDENCY_MESSAGE(ERROR);
    DeleteRef(env, Clazz());
    class_ = {};
    return;
  }
  AddJniInfo(init);

  {
    MethodWrapper method_wrapper(context.GetJniEnv());
    JniUniquePtr<jobject> local_self_ref = WrapJni(
        env, env->NewObject(Clazz(), init, method_wrapper.GetEnum(method)));
    SetSelf(LocalToGlobalRef(std::move(local_self_ref)));
  }

  get_glyph_metrics_ = GetMethodHandle(
      "getGlyphMetrics", "(ILjava/lang/Object;FLandroid/graphics/Paint;[F)V");
  get_text_glyphs_ =
      GetMethodHandle("getTextGlyphs",
                      "(Ljava/lang/String;Landroid/graphics/Paint;)[Lcom/"
                      "google/ar/imp/core/glyph/GlyphAdvance;");
  get_combined_character_groups_ =
      GetMethodHandle("getCombinedCharacterGroups",
                      "(Ljava/lang/String;Landroid/graphics/Paint;)[I");
  draw_glyph_ =
      GetMethodHandle("drawGlyph",
                      "(Landroid/graphics/Canvas;IFFLjava/lang/Object;F"
                      "Landroid/graphics/Paint;Landroid/graphics/Paint;)V");
}

bool AndroidGlyphSource::IsAvailable() const {
  return static_cast<bool>(class_);
}

ScopedCanvas::TextMetrics AndroidGlyphSource::GetGlyphMetrics(
    int glyph_id, FontHolder* font, float stroke_width, android::Paint& paint) {
  if (!IsAvailable()) {
    LOG_MISSING_DEPENDENCY_MESSAGE(FATAL);
  }

  JniUniquePtr<jfloatArray> out_bounds_array = CreateJniFloatArray(Env(), 7);

  jobject font_jobject =
      font ? static_cast<jobject>(font->GetPlatformFont()) : nullptr;
  CallVoidMethod(get_glyph_metrics_, glyph_id, font_jobject, stroke_width,
                 paint.WeakReference(), out_bounds_array.get());

  jfloat* out_bounds_array_ptr =
      Env()->GetFloatArrayElements(out_bounds_array.get(), /*isCopy=*/nullptr);

  ScopedCanvas::TextMetrics metrics{
      float2{out_bounds_array_ptr[0], out_bounds_array_ptr[1]},
      float2{out_bounds_array_ptr[2], out_bounds_array_ptr[3]},
      out_bounds_array_ptr[4],
      out_bounds_array_ptr[5],
      out_bounds_array_ptr[6],
  };

  Env()->ReleaseFloatArrayElements(out_bounds_array.get(), out_bounds_array_ptr,
                                   /*mode=*/0);

  return metrics;
}

std::vector<ScopedCanvas::GlyphGroup>
AndroidGlyphSource::GetCombinedCharacterGroups(absl::string_view text,
                                               android::Paint& paint) {
  if (!IsAvailable()) {
    LOG_MISSING_DEPENDENCY_MESSAGE(FATAL);
  }

  JniUniquePtr<jstring> text_jstring = ToJniString(Env(), text);
  JniUniquePtr<jintArray> indices_array = WrapJni(
      Env(), CallIntArrayMethod(get_combined_character_groups_,
                                text_jstring.get(), paint.WeakReference()));

  jint* indices_array_ptr =
      Env()->GetIntArrayElements(indices_array.get(), /*isCopy=*/nullptr);

  jsize out_size = Env()->GetArrayLength(indices_array.get());
  std::vector<ScopedCanvas::GlyphGroup> result;
  result.reserve(out_size);
  for (int i = 0; i < out_size; ++i) {
    result.push_back(indices_array_ptr[i]);
  }

  Env()->ReleaseIntArrayElements(indices_array.get(), indices_array_ptr,
                                 /*mode=*/0);

  return result;
}

std::vector<ScopedCanvas::GlyphAdvance> AndroidGlyphSource::GetTextGlyphs(
    absl::string_view text, android::Paint& paint) {
  if (!IsAvailable()) {
    LOG_MISSING_DEPENDENCY_MESSAGE(FATAL);
  }

  JniUniquePtr<jstring> text_jstring = ToJniString(Env(), text);

  JniUniquePtr<jobjectArray> out_jobject_array = WrapJni(
      Env(), static_cast<jobjectArray>(CallObjectMethod(
                 get_text_glyphs_, text_jstring.get(), paint.WeakReference())));
  jsize out_size = Env()->GetArrayLength(out_jobject_array.get());

  std::vector<ScopedCanvas::GlyphAdvance> result;
  result.reserve(out_size);
  for (int i = 0; i < out_size; i++) {
    AndroidGlyphAdvance glyph_advance(
        Env(), Env()->GetObjectArrayElement(out_jobject_array.get(), i));

    std::unique_ptr<FontHolder> fallback_font;
    JniUniquePtr<jobject> font_local_ref =
        WrapJni(Env(), glyph_advance.GetFont());
    if (font_local_ref) {
      std::unique_ptr<android::Font> font =
          std::make_unique<android::Font>(Env(), font_local_ref.get());
      fallback_font = std::make_unique<AndroidFontFontHolder>(std::move(font));
    }

    result.push_back(ScopedCanvas::GlyphAdvance{
        .glyph = glyph_advance.GetId(),
        .width = glyph_advance.GetWidth(),
        .fallback_font = std::move(fallback_font),
        .is_emoji = glyph_advance.IsEmoji(),
    });
  }

  return result;
}

void AndroidGlyphSource::DrawGlyph(android::Canvas& canvas, int glyph_id,
                                   float x, float y, FontHolder* font,
                                   float stroke_width,
                                   android::Paint& fillPaint,
                                   android::Paint& strokePaint) {
  if (!IsAvailable()) {
    LOG_MISSING_DEPENDENCY_MESSAGE(FATAL);
  }

  jobject font_jobject =
      font ? static_cast<jobject>(font->GetPlatformFont()) : nullptr;
  CallVoidMethod(draw_glyph_, canvas.WeakReference(), glyph_id, x, y,
                 font_jobject, stroke_width, fillPaint.WeakReference(),
                 strokePaint.WeakReference());
}

}  // namespace imp
