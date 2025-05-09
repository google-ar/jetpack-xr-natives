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

#ifndef THIRD_PARTY_IMPRESS_CORE_AR_ANDROID_AR_CAMERA_TEXTURE_ARCORE_H_
#define THIRD_PARTY_IMPRESS_CORE_AR_ANDROID_AR_CAMERA_TEXTURE_ARCORE_H_

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>

#include <utility>
#include <vector>

#include "core/common/robin_map.h"
#include "core/math/math.h"
#include "core/render/texture.h"
#include "filament/filament/include/filament/Engine.h"

namespace imp {
namespace ar {

// Container for camera texture, filament::Stream and texture id.
//
// The implementation of AndroidCameraTexture is platform specific, it uses
// OpenGL. All contents will be destroyed when this class goes out of scope.
class AndroidCameraTexture {
 public:
  // set_texture_id_handler will be called after an id obtained for the
  // texture.
  AndroidCameraTexture(
      filament::Engine* engine, imp::TextureFactory* texture_factory,
      uint2 texture_dimensions, int num_textures,
      std::function<void(std::vector<GLuint>)>& set_texture_id_handler);
  ~AndroidCameraTexture();

  void SetTextureId(GLuint texture_id) { current_texture_id_ = texture_id; }
  GLuint GetTextureId() const { return current_texture_id_; }
  const imp::Texture* GetTexture() const {
    return textures_.at(current_texture_id_).get();
  }

 private:
  GLuint current_texture_id_ = 0;
  std::vector<GLuint> texture_ids_;
  RobinMap<GLuint, TexturePtr> textures_;
  filament::Engine* engine_;
};
}  // namespace ar
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_AR_ANDROID_AR_CAMERA_TEXTURE_ARCORE_H_
