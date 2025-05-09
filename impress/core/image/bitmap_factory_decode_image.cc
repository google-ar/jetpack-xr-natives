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

#include "core/image/bitmap_factory_decode_image.h"

#include <android/bitmap.h>
#include <jni.h>

#include <memory>
#include <string>

#include "absl/memory/memory.h"
#include "absl/status/statusor.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/common/jni_helpers.h"
#include "core/common/platform_helpers.h"
#include "core/image/android_bitmap_image_contents.h"
#include "core/image/image_contents.h"
#include "core/view/platforms/android/wrappers/bitmap.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::image::details {
namespace {

// Based on OpenGL ES limits.
static constexpr size_t kMaxTextureSizeAndroid = 4096;

class BitmapFactory_Options : public imp::JavaWrapper {
 public:
  BitmapFactory_Options(JNIEnv* env)
      : JavaWrapper(env, "android/graphics/BitmapFactory$Options", "()V") {
    in_sample_size_ = GetFieldHandle("inSampleSize", "I");
    in_premultiplied_ = GetFieldHandle("inPremultiplied", "Z");
    in_just_decode_bounds_ = GetFieldHandle("inJustDecodeBounds", "Z");
    out_width_ = GetFieldHandle("outWidth", "I");
    out_height_ = GetFieldHandle("outHeight", "I");
  }

  void SetSampleSize(int sample_size) {
    Env()->SetIntField(Self(), ToFieldID(in_sample_size_), sample_size);
  }

  void SetPremultiplied(bool premultiplied) {
    Env()->SetBooleanField(Self(), ToFieldID(in_premultiplied_), premultiplied);
  }

  void SetJustDecodeBounds(bool just_decode_bounds) {
    Env()->SetBooleanField(Self(), ToFieldID(in_just_decode_bounds_),
                           just_decode_bounds);
  }

  int GetOutWidth() {
    return Env()->GetIntField(Self(), ToFieldID(out_width_));
  }

  int GetOutHeight() {
    return Env()->GetIntField(Self(), ToFieldID(out_height_));
  }

 private:
  imp::JniHandle in_sample_size_;
  imp::JniHandle in_premultiplied_;
  imp::JniHandle in_just_decode_bounds_;
  imp::JniHandle out_width_;
  imp::JniHandle out_height_;
};

}  // namespace

absl::StatusOr<std::unique_ptr<ImageContents>> BitmapFactoryDecodeImage(
    const imp::Context& context, absl::string_view name,
    resources::Resource resource) {
  JNIEnv* env = context.TryGetJniEnv();
  JniUniquePtr<jclass> bitmap_factory_class =
      FindClass(env, "android/graphics/BitmapFactory");
  jmethodID create_bitmap = env->GetStaticMethodID(
      bitmap_factory_class.get(), "decodeByteArray",
      "([BIILandroid/graphics/BitmapFactory$Options;)Landroid/graphics/"
      "Bitmap;");

  JniUniquePtr<jbyteArray> encoded_array =
      CreateJniByteArray(env, resource.GetData().Size());
  void* array_base = env->GetPrimitiveArrayCritical(
      static_cast<jarray>(encoded_array.get()), nullptr);
  memcpy(array_base, resource.GetData().Data(), resource.GetData().Size());
  env->ReleasePrimitiveArrayCritical(encoded_array.get(), array_base, 0);

  BitmapFactory_Options get_size_options(env);
  get_size_options.SetJustDecodeBounds(true);
  env->CallStaticObjectMethod(bitmap_factory_class.get(), create_bitmap,
                              encoded_array.get(), 0, resource.GetData().Size(),
                              get_size_options.WeakReference());

  // See if we need to subsample
  int width = get_size_options.GetOutWidth();
  int height = get_size_options.GetOutHeight();

  // BitmapFactory rounds down the sample size to the nearest power of two so we
  // double the sample size if it's not a power of two.
  int sampleSize = 1 + ((std::max(width, height) - 1) / kMaxTextureSizeAndroid);
  if (((sampleSize - 1) & sampleSize) != 0) {
    sampleSize <<= 1;
  }

  BitmapFactory_Options read_options(env);
  read_options.SetPremultiplied(false);
  read_options.SetSampleSize(sampleSize);
  jobject bitmap = env->CallStaticObjectMethod(
      bitmap_factory_class.get(), create_bitmap, encoded_array.get(), 0,
      resource.GetData().Size(), read_options.Release());

  if (bitmap == nullptr) {
    return absl::InternalError(
        FormatString("Failed to decode image '%.*s' (@%p, %d bytes)",
                     static_cast<int>(name.size()), name.data(),
                     resource.GetData().Data(), resource.GetData().Size()));
  }

  auto jbitmap = std::make_unique<android::Bitmap>(context.GetJniEnv(), bitmap);
  MP_ASSIGN_OR_RETURN(auto bitmap_config, jbitmap->GetBitmapConfig());

  return std::make_unique<BitmapImageContents>(context, std::move(jbitmap),
                                               bitmap_config);
}
}  // namespace imp::image::details
