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

#include "apibindings/bindings_texture.h"

#include <utility>

#include "apibindings/bindings_object.h"
#include "core/common/small_source_location.h"
#include "core/render/texture.h"

namespace imp {

BindingsTexture::BindingsTexture(BorrowedTexturePtr texture)
    : BindingsObject(), texture_(std::move(texture)) {}

BorrowedTexturePtr BindingsTexture::GetTexture(SmallSourceLocation loc) {
  return texture_.WithNewLocation(loc);
}

}  // namespace imp
