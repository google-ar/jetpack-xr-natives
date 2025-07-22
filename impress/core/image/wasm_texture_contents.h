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

#ifndef THIRD_PARTY_IMPRESS_CORE_IMAGE_TEXTURE_CONTENTS_H_
#define THIRD_PARTY_IMPRESS_CORE_IMAGE_TEXTURE_CONTENTS_H_

#include <cstdint>

#include "third_party/GL/gl/include/GLES3/gl3.h"

namespace imp {

class WasmTextureContents {
 public:
  explicit WasmTextureContents(GLuint texture_id, int width, int height)
      : texture_id_(texture_id), width_(width), height_(height) {}

  GLuint GetTextureId() const { return texture_id_; }
  uint32_t GetWidth() const { return width_; }
  uint32_t GetHeight() const { return height_; }

 private:
  GLuint texture_id_;
  uint32_t width_;
  uint32_t height_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_IMAGE_TEXTURE_CONTENTS_H_
