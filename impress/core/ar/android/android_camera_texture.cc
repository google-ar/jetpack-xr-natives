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

#include "core/ar/android/android_camera_texture.h"

#include "core/common/log.h"
#include "core/common/platform_helpers.h"
#include "core/config.h"
#include "core/render/texture_factory.h"

namespace imp {
namespace ar {

AndroidCameraTexture::AndroidCameraTexture(
    filament::Engine* engine, imp::TextureFactory* texture_factory,
    uint2 texture_dimensions, int num_textures,
    std::function<void(std::vector<GLuint>)>& texture_id_handler)
    : texture_ids_(num_textures, 0), engine_(engine) {
  if (engine->getBackend() == filament::Engine::Backend::OPENGL) {
#if IMP_PLATFORM(ANDROID_API19)
    glGenTextures(texture_ids_.size(), texture_ids_.data());
    for (int i = 0; i < texture_ids_.size(); i++) {
      glBindTexture(GL_TEXTURE_EXTERNAL_OES, texture_ids_[i]);
      glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_WRAP_S,
                      GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_WRAP_T,
                      GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MIN_FILTER,
                      GL_NEAREST);
      glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MAG_FILTER,
                      GL_NEAREST);
    }
#else   // IMP_PLATFORM(ANDROID_API19)
    IMP_LOG(imp::ERROR) << "Not supported in robolectric";
#endif  // IMP_PLATFORM(ANDROID_API19)
  } else {
    // Assign some fake texture Ids, this is only used for testing on the noop
    // backend.
    for (int i = 0; i < num_textures; i++) {
      texture_ids_[i] = i + 1;
    }
  }
  current_texture_id_ = texture_ids_[0];

  texture_id_handler(texture_ids_);

  // Create a filament texture for each texture id.
  for (GLuint texture_id : texture_ids_) {
    assert(num_textures > 1);
    // If we are using multiple textures, then we tell filament to render them
    // directly instead of using a stream. This basically means that we are
    // are relying on the round-robin textures provided by ARCore to prevent
    // running into issues caused by ARCore trying to write to the camera
    // texture for a new frame while filament is using it to render the
    // previous frame on it's render thread.
    textures_[texture_id] = texture_factory->CreateExternalTexture(texture_id);
  }
}

AndroidCameraTexture::~AndroidCameraTexture() {
  if (engine_->getBackend() == filament::Engine::Backend::OPENGL) {
#if IMP_PLATFORM(ANDROID_API19)
    glDeleteTextures(texture_ids_.size(), texture_ids_.data());
#else   // IMP_PLATFORM(ANDROID_API19)
    IMP_LOG(imp::ERROR) << "Not supported in robolectric";
#endif  // IMP_PLATFORM(ANDROID_API19)
  }
}
}  // namespace ar
}  // namespace imp
