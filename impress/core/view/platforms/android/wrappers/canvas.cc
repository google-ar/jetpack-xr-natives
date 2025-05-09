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

#include "core/view/platforms/android/wrappers/canvas.h"

#include <jni.h>

#include "core/common/jni_helpers.h"
#include "core/geometry/shapes/rect.h"
#include "core/math/vec.h"
#include "core/view/platforms/android/wrappers/graphics_helpers.h"

namespace imp::android {

Canvas::Canvas(JNIEnv* env, jobject j_canvas) : JavaWrapper(env, j_canvas) {
  JniUniquePtr<jclass> blend_mode_class =
      FindClass(env, "android/graphics/PorterDuff$Mode");
  auto clear_mode_id = env->GetStaticFieldID(
      blend_mode_class.get(), "CLEAR", "Landroid/graphics/PorterDuff$Mode;");
  clear_mode_ =
      env->GetStaticObjectField(blend_mode_class.get(), clear_mode_id);

  draw_color_ = GetMethodHandle("drawColor", "(I)V");
  draw_color_with_mode_ =
      GetMethodHandle("drawColor", "(ILandroid/graphics/PorterDuff$Mode;)V");
  draw_rect_ = GetMethodHandle("drawRect", "(FFFFLandroid/graphics/Paint;)V");
  draw_round_rect_ =
      GetMethodHandle("drawRoundRect", "(FFFFFFLandroid/graphics/Paint;)V");
  draw_picture_ =
      GetMethodHandle("drawPicture", "(Landroid/graphics/Picture;)V");
  draw_text_ = GetMethodHandle(
      "drawText", "(Ljava/lang/String;FFLandroid/graphics/Paint;)V");
}

void Canvas::DrawColor(float3 color) { DrawColor(float4(color, 1.0f)); }

void Canvas::DrawColor(float4 color) {
  CallVoidMethod(draw_color_, ToColorInt(color));
}

void Canvas::Clear() {
  CallVoidMethod(draw_color_with_mode_, ToColorInt(kZero4), clear_mode_);
}

void Canvas::DrawRect(const imp::Rect& rect, Paint& paint) {
  float2 rect_min = rect.GetMin();
  float2 rect_max = rect.GetMax();
  CallVoidMethod(draw_rect_, rect_min.x, rect_min.y, rect_max.x, rect_max.y,
                 paint.WeakReference());
}

void Canvas::DrawRoundRect(const imp::Rect& rect, float2 corner_radius,
                           Paint& paint) {
  float2 rect_min = rect.GetMin();
  float2 rect_max = rect.GetMax();
  CallVoidMethod(draw_round_rect_, rect_min.x, rect_min.y, rect_max.x,
                 rect_max.y, corner_radius.x, corner_radius.y,
                 paint.WeakReference());
}

void Canvas::DrawText(absl::string_view text, float2 pos, Paint& paint) {
  CallVoidMethod(draw_text_, ToString(Env(), text), pos.x, pos.y,
                 paint.WeakReference());
}

void Canvas::DrawPicture(jobject picture) {
  CallVoidMethod(draw_picture_, picture);
}

}  // namespace imp::android
