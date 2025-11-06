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

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_BINDINGS_TEXTURE_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_BINDINGS_TEXTURE_H_

#include "apibindings/bindings_object.h"
#include "core/common/small_source_location.h"
#include "core/render/texture.h"

namespace imp {

// Wraps a borrowed texture pointer so that this object can be destroyed
// from Java without affecting the owned pointer of the texture.
class BindingsTexture : public BindingsObject {
 public:
  explicit BindingsTexture(BorrowedTexturePtr texture);

  // Returns the borrowed texture pointer that BindingsTexture wraps.
  BorrowedTexturePtr GetTexture(
      SmallSourceLocation loc = SmallSourceLocation::Current());

 private:
  BorrowedTexturePtr texture_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_BINDINGS_TEXTURE_H_
