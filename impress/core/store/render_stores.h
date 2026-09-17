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

#ifndef THIRD_PARTY_IMPRESS_CORE_STORE_RENDER_STORES_H_
#define THIRD_PARTY_IMPRESS_CORE_STORE_RENDER_STORES_H_

#include "core/store/owned_ptr_store.h"

namespace imp {

class Material;
class Texture;
class Mesh;

using MaterialStore = OwnedPtrStore<Material>;
using TextureStore = OwnedPtrStore<Texture, MaterialStore>;
using MeshStore = OwnedPtrStore<Mesh>;

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_STORE_RENDER_STORES_H_
