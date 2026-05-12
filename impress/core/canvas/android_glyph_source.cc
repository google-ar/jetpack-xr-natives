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

#include <cassert>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "core/canvas/fonts/android_font_font_holder.h"
#include "core/canvas/fonts/font_holder.h"
#include "core/canvas/scoped_canvas.h"
#include "core/common/context.h"
#include "core/common/jni_helpers.h"
#include "core/common/trace.h"
#include "core/text/text_metrics.proto.h"
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
         "\"//third_party/impress/java/com/google/ar/imp/core/glyph\". " \
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

AndroidGlyphSource::AndroidGlyphSource(const Context& context, Method method,
                                       int cache_size_bytes)
    : JavaWrapper(context.GetJniEnv()),
      glyph_advance_class_(WrapJni(Env(), static_cast<jclass>(nullptr))) {
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
                       "(Lcom/google/ar/imp/core/glyph/GlyphSource$Method;I)V");
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
        env, env->NewObject(Clazz(), init, method_wrapper.GetEnum(method),
                            cache_size_bytes));
    SetSelf(LocalToGlobalRef(std::move(local_self_ref)));
  }

  get_glyph_metrics_ = GetMethodHandle(
      "getGlyphMetrics", "(ILjava/lang/Object;Landroid/graphics/Paint;)[F");
  get_text_glyphs_ =
      GetMethodHandle("getTextGlyphs",
                      "(Ljava/lang/String;Landroid/graphics/Paint;)[Lcom/"
                      "google/ar/imp/core/glyph/GlyphAdvance;");
  release_text_glyphs_ = GetMethodHandle("releaseTextGlyphs", "([I)V");
  release_text_glyph_ = GetMethodHandle("releaseTextGlyph", "(I)V");
  get_combined_character_groups_ =
      GetMethodHandle("getCombinedCharacterGroups",
                      "(Ljava/lang/String;Landroid/graphics/Paint;)[I");
  draw_glyph_ =
      GetMethodHandle("drawGlyph",
                      "(Landroid/graphics/Canvas;IFFLjava/lang/Object;F"
                      "Landroid/graphics/Paint;Landroid/graphics/Paint;)V");
  dispose_ = GetMethodHandle("dispose", "()V");

  // GlyphAdvance.
  glyph_advance_class_ = LocalToGlobalRef(WrapJni(
      Env(), Env()->FindClass("com/google/ar/imp/core/glyph/GlyphAdvance")));
  glyph_advance_get_id_ =
      env->GetMethodID(glyph_advance_class_.get(), "getId", "()I");
  glyph_advance_get_width_ =
      env->GetMethodID(glyph_advance_class_.get(), "getWidth", "()F");
  glyph_advance_get_font_ = env->GetMethodID(glyph_advance_class_.get(),
                                             "getFont", "()Ljava/lang/Object;");
  glyph_advance_is_emoji_ =
      env->GetMethodID(glyph_advance_class_.get(), "isEmoji", "()Z");
}

AndroidGlyphSource::~AndroidGlyphSource() {
  if (IsAvailable()) {
    CallVoidMethod(dispose_);
  }
}

bool AndroidGlyphSource::IsAvailable() const {
  return static_cast<bool>(class_);
}

TextMetrics AndroidGlyphSource::GetGlyphMetrics(int glyph_id, FontHolder* font,
                                                float stroke_width,
                                                android::Paint& paint) {
  IMP_TRACE();

  if (!IsAvailable()) {
    LOG_MISSING_DEPENDENCY_MESSAGE(FATAL);
  }

  jobject font_jobject =
      font ? static_cast<jobject>(font->GetPlatformFont()) : nullptr;

  JniUniquePtr<jfloatArray> out_array =
      WrapJni(Env(), CallFloatArrayMethod(get_glyph_metrics_, glyph_id,
                                          font_jobject, paint.WeakReference()));
  AssertNoException(Env());
  assert(Env()->GetArrayLength(out_array.get()) >= 8);

  jfloat* out_array_ptr =
      Env()->GetFloatArrayElements(out_array.get(), /*isCopy=*/nullptr);

  float padding = out_array_ptr[0] * stroke_width;

  TextMetrics metrics;
  metrics.set_origin_x(out_array_ptr[1]);
  metrics.set_origin_y(out_array_ptr[2]);
  metrics.set_size_x(out_array_ptr[3] + padding);
  metrics.set_size_y(out_array_ptr[4] + padding);
  metrics.set_typographical_width(out_array_ptr[5]);
  metrics.set_font_origin_y(out_array_ptr[6]);
  metrics.set_font_size_y(out_array_ptr[7] + padding);

  Env()->ReleaseFloatArrayElements(out_array.get(), out_array_ptr, JNI_ABORT);

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
  AssertNoException(Env());

  jint* indices_array_ptr =
      Env()->GetIntArrayElements(indices_array.get(), /*isCopy=*/nullptr);

  jsize out_size = Env()->GetArrayLength(indices_array.get());
  std::vector<ScopedCanvas::GlyphGroup> result;
  result.reserve(out_size);
  for (int i = 0; i < out_size; ++i) {
    result.push_back(indices_array_ptr[i]);
  }

  Env()->ReleaseIntArrayElements(indices_array.get(), indices_array_ptr,
                                 JNI_ABORT);

  return result;
}

std::vector<ScopedCanvas::GlyphAdvance> AndroidGlyphSource::GetTextGlyphs(
    absl::string_view text, android::Paint& paint) {
  IMP_TRACE();

  if (!IsAvailable()) {
    LOG_MISSING_DEPENDENCY_MESSAGE(FATAL);
  }

  JniUniquePtr<jstring> text_jstring = ToJniString(Env(), text);

  JniUniquePtr<jobjectArray> out_jobject_array = WrapJni(
      Env(), static_cast<jobjectArray>(CallObjectMethod(
                 get_text_glyphs_, text_jstring.get(), paint.WeakReference())));
  AssertNoException(Env());

  jsize out_size = Env()->GetArrayLength(out_jobject_array.get());

  std::vector<ScopedCanvas::GlyphAdvance> result;
  result.reserve(out_size);
  for (int i = 0; i < out_size; i++) {
    JniUniquePtr<jobject> glyph_advance = WrapJni(
        Env(), Env()->GetObjectArrayElement(out_jobject_array.get(), i));

    std::unique_ptr<FontHolder> fallback_font;
    JniUniquePtr<jobject> font_local_ref =
        WrapJni(Env(), GlyphAdvanceGetFont(glyph_advance.get()));
    if (font_local_ref) {
      std::unique_ptr<android::Font> font =
          std::make_unique<android::Font>(Env(), font_local_ref.get());
      fallback_font = std::make_unique<AndroidFontFontHolder>(std::move(font));
    }

    result.push_back(ScopedCanvas::GlyphAdvance{
        .glyph = ScopedCanvas::GlyphId(
            GlyphAdvanceGetId(glyph_advance.get()),
            std::make_optional([this](int32_t glyph_id) {
              ReleaseTextGlyphs(absl::MakeSpan(&glyph_id, 1));
            })),
        .width = GlyphAdvanceGetWidth(glyph_advance.get()),
        .fallback_font = std::move(fallback_font),
        .is_emoji = GlyphAdvanceIsEmoji(glyph_advance.get()),
    });
  }

  return result;
}

void AndroidGlyphSource::ReleaseTextGlyphs(absl::Span<int> glyph_ids) {
  IMP_TRACE();

  if (!IsAvailable()) {
    LOG_MISSING_DEPENDENCY_MESSAGE(FATAL);
  }

  if (glyph_ids.empty()) {
    return;
  }

  if (glyph_ids.size() == 1) {
    // Optimize for the single-element RAII wrapper case.
    CallVoidMethod(release_text_glyph_, glyph_ids[0]);
  } else {
    JniUniquePtr<jintArray> glyph_ids_java =
        CreateJniIntArray(Env(), glyph_ids);
    CallVoidMethod(release_text_glyphs_, glyph_ids_java.get());
  }
  AssertNoException(Env());
}

void AndroidGlyphSource::DrawGlyph(android::Canvas& canvas, int glyph_id,
                                   float x, float y, FontHolder* font,
                                   float stroke_width,
                                   android::Paint& fillPaint,
                                   android::Paint& strokePaint) {
  IMP_TRACE();

  if (!IsAvailable()) {
    LOG_MISSING_DEPENDENCY_MESSAGE(FATAL);
  }

  jobject font_jobject =
      font ? static_cast<jobject>(font->GetPlatformFont()) : nullptr;
  CallVoidMethod(draw_glyph_, canvas.WeakReference(), glyph_id, x, y,
                 font_jobject, stroke_width, fillPaint.WeakReference(),
                 strokePaint.WeakReference());
  AssertNoException(Env());
}

int AndroidGlyphSource::GlyphAdvanceGetId(jobject object) {
  return Env()->CallIntMethod(object, glyph_advance_get_id_);
}

float AndroidGlyphSource::GlyphAdvanceGetWidth(jobject object) {
  return Env()->CallFloatMethod(object, glyph_advance_get_width_);
}

jobject AndroidGlyphSource::GlyphAdvanceGetFont(jobject object) {
  return Env()->CallObjectMethod(object, glyph_advance_get_font_);
}

bool AndroidGlyphSource::GlyphAdvanceIsEmoji(jobject object) {
  return Env()->CallBooleanMethod(object, glyph_advance_is_emoji_);
}

}  // namespace imp
