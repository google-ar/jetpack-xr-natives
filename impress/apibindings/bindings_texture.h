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

// Wraps an owned or borrowed texture pointer so that the ownership of the
// BindingsTexture pointer can be released to Java without affecting the
// ownership of the actual texture pointer.
class BindingsTexture : public BindingsObject {
 public:
  explicit BindingsTexture(OwnedOrBorrowedTexturePtr texture);

  // Calls Borrow() on the OwnedOrBorrowedTexturePtr to return a
  // BorrowedTexturePtr to the texture pointer wrapped by this class.
  BorrowedTexturePtr GetTexture(
      SmallSourceLocation loc = SmallSourceLocation::Current());

 private:
  // OwnedOrBorrowedTexturePtr is used so that texture_ can hold both an
  // OwnedTexturePtr and a BorrowedTexturePtr. A texture pointer is owned when
  // it is loaded from the application. It is considered owned when it is
  // received from the system.
  OwnedOrBorrowedTexturePtr texture_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_BINDINGS_TEXTURE_H_
