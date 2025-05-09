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

#include <memory>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/strings/string_view.h"
#include "core/canvas/fonts/android_font_font_holder.h"
#include "core/canvas/fonts/font_holder.h"
#include "core/canvas/scoped_canvas.h"
#include "core/math/vec.h"
#include "core/view/platforms/android/wrappers/font.h"
#include "core/view/platforms/android/wrappers/graphics_helpers.h"

#define LOG_MISSING_DEPENDENCY_MESSAGE(level)                            \
  IMP_LOG(imp::level)                                                             \
      << "Likely missing Android Java dependency on "                    \
         "\"//java/com/google/ar/imp/core/glyph\". " \
         "Please adjust your build files accordingly."

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
    // TODO: This feature adds a non-trivial amount of space to the
    // binary, but is not critical to Impress's overall functionality. Blaze
    // also doesn't offer a way to automatically pull in a JNI->Java interface's
    // corresponding Java dependency, so clients need to explicitly specify the
    // JVM dependency themselves. At the same time, we don't want to
    // mysteriously crash every single client for missing this dependency,
    // especially if they never even touch the glyph-specific APIs.
    env->ExceptionDescribe();
    env->ExceptionClear();
    LOG_MISSING_DEPENDENCY_MESSAGE(ERROR);
    return;
  }
  class_ =
      AddJniInfo(static_cast<jclass>(env->NewGlobalRef(local_class_ref.get())));
  class_path_ = class_path;

  jmethodID init =
      env->GetMethodID(Clazz(), "<init>",
                       "(Lcom/google/ar/imp/core/glyph/GlyphSource$Method;)V");
  AddJniInfo(init);

  {
    MethodWrapper method_wrapper(context.GetJniEnv());
    JniUniquePtr<jobject> local_self_ref = WrapJni(
        env, env->NewObject(Clazz(), init, method_wrapper.GetEnum(method)));
    SetSelf(env->NewGlobalRef(local_self_ref.get()));
  }

  get_glyph_metrics_ =
      GetMethodHandle("getGlyphMetrics", "(ILjava/lang/Object;IFF[F)V");
  get_text_glyphs_ = GetMethodHandle(
      "getTextGlyphs", "(Ljava/lang/String;IF[I[F[Ljava/lang/Object;[Z)I");
  draw_glyph_ = GetMethodHandle(
      "drawGlyph", "(Landroid/graphics/Canvas;IFFLjava/lang/Object;IFIIF)V");
}

bool AndroidGlyphSource::IsAvailable() const {
  return static_cast<bool>(class_);
}

ScopedCanvas::TextMetrics AndroidGlyphSource::GetGlyphMetrics(
    int glyph_id, FontHolder* font, int font_size, float stroke_width,
    float text_tracking) {
  if (!IsAvailable()) {
    LOG_MISSING_DEPENDENCY_MESSAGE(FATAL);
  }

  JniUniquePtr<jfloatArray> out_bounds_array = CreateJniFloatArray(Env(), 7);

  jobject font_jobject =
      font ? static_cast<jobject>(font->GetPlatformFont()) : nullptr;
  CallVoidMethod(get_glyph_metrics_, glyph_id, font_jobject, font_size,
                 stroke_width, text_tracking, out_bounds_array.get());

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

std::vector<ScopedCanvas::GlyphAdvance> AndroidGlyphSource::GetTextGlyphs(
    absl::string_view text, int font_size, float text_tracking) {
  if (!IsAvailable()) {
    LOG_MISSING_DEPENDENCY_MESSAGE(FATAL);
  }

  JniUniquePtr<jclass> object_clazz = FindClass(Env(), "java/lang/Object");

  JniUniquePtr<jintArray> out_ids_array = CreateJniIntArray(Env(), text.size());
  JniUniquePtr<jfloatArray> out_widths_array =
      CreateJniFloatArray(Env(), text.size());
  JniUniquePtr<jobjectArray> out_fonts_array = CreateJniObjectArray(
      Env(), text.size(), object_clazz.get(), /*initialElement=*/nullptr);
  JniUniquePtr<jbooleanArray> out_is_emoji_array =
      CreateJniBooleanArray(Env(), text.size());

  jint out_size =
      CallIntMethod(get_text_glyphs_, ToString(Env(), text), font_size,
                    text_tracking, out_ids_array.get(), out_widths_array.get(),
                    out_fonts_array.get(), out_is_emoji_array.get());

  jint* out_ids_array_ptr =
      Env()->GetIntArrayElements(out_ids_array.get(), /*isCopy=*/nullptr);
  jfloat* out_widths_array_ptr =
      Env()->GetFloatArrayElements(out_widths_array.get(), /*isCopy=*/nullptr);
  jboolean* out_is_emoji_array_ptr = Env()->GetBooleanArrayElements(
      out_is_emoji_array.get(), /*isCopy=*/nullptr);

  std::vector<ScopedCanvas::GlyphAdvance> result;
  result.reserve(out_size);
  for (int i = 0; i < out_size; i++) {
    std::unique_ptr<FontHolder> fallback_font;
    JniUniquePtr<jobject> font_local_ref =
        WrapJni(Env(), Env()->GetObjectArrayElement(out_fonts_array.get(), i));
    if (font_local_ref) {
      std::unique_ptr<android::Font> font =
          std::make_unique<android::Font>(Env(), font_local_ref.get());
      fallback_font = std::make_unique<AndroidFontFontHolder>(std::move(font));
    }
    result.push_back(ScopedCanvas::GlyphAdvance{
        .glyph = out_ids_array_ptr[i],
        .width = out_widths_array_ptr[i],
        .fallback_font = std::move(fallback_font),
        .is_emoji = !!out_is_emoji_array_ptr[i],
    });
  }

  Env()->ReleaseIntArrayElements(out_ids_array.get(), out_ids_array_ptr,
                                 /*mode=*/0);
  Env()->ReleaseFloatArrayElements(out_widths_array.get(), out_widths_array_ptr,
                                   /*mode=*/0);
  Env()->ReleaseBooleanArrayElements(out_is_emoji_array.get(),
                                     out_is_emoji_array_ptr,
                                     /*mode=*/0);

  return result;
}

void AndroidGlyphSource::DrawGlyph(android::Canvas& canvas, int glyph_id,
                                   float x, float y, FontHolder* font,
                                   int font_size, float stroke_width,
                                   float4 fill_color, float4 stroke_color,
                                   float text_tracking) {
  if (!IsAvailable()) {
    LOG_MISSING_DEPENDENCY_MESSAGE(FATAL);
  }

  jobject font_jobject =
      font ? static_cast<jobject>(font->GetPlatformFont()) : nullptr;
  CallVoidMethod(draw_glyph_, canvas.WeakReference(), glyph_id, x, y,
                 font_jobject, font_size, stroke_width,
                 android::ToColorInt(fill_color),
                 android::ToColorInt(stroke_color), text_tracking);
}

}  // namespace imp
