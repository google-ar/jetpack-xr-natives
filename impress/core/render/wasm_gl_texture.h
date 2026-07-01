/*
 * Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_RENDER_WASM_GL_TEXTURE_H_
#define THIRD_PARTY_IMPRESS_CORE_RENDER_WASM_GL_TEXTURE_H_

#include "third_party/GL/gl/include/GLES3/gl3.h"

namespace imp {

// A move-only RAII wrapper around an OpenGL texture ID.
// Automatically deletes the texture using glDeleteTextures when destroyed.
class WasmGlTexture {
 public:
  WasmGlTexture() : id_(0) {}
  explicit WasmGlTexture(GLuint id) : id_(id) {}

  ~WasmGlTexture() { Reset(); }

  // Move-only
  WasmGlTexture(const WasmGlTexture&) = delete;
  WasmGlTexture& operator=(const WasmGlTexture&) = delete;

  WasmGlTexture(WasmGlTexture&& other) noexcept : id_(other.id_) {
    other.id_ = 0;
  }

  WasmGlTexture& operator=(WasmGlTexture&& other) noexcept {
    if (this != &other) {
      Reset();
      id_ = other.id_;
      other.id_ = 0;
    }
    return *this;
  }

  GLuint Get() const { return id_; }
  bool IsValid() const { return id_ != 0; }
  void Reset(GLuint new_id = 0) {
    if (id_ != 0) {
      glDeleteTextures(1, &id_);
    }
    id_ = new_id;
  }

 private:
  GLuint id_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RENDER_WASM_GL_TEXTURE_H_
