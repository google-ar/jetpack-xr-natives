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

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_BINDINGS_MATERIAL_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_BINDINGS_MATERIAL_H_

#include "apibindings/bindings_object.h"
#include "core/common/hash.h"
#include "core/common/small_source_location.h"
#include "core/materials/material.h"

namespace imp {

// Wraps a borrowed material pointer so that this object can be destroyed
// from Java without affecting the owned pointer of the material.
class BindingsMaterial : public BindingsObject {
 public:
  explicit BindingsMaterial(BorrowedMaterialPtr material, HashValue type_hash);

  // Returns the borrowed material pointer that BindingsMaterial wraps.
  BorrowedMaterialPtr GetMaterial(
      SmallSourceLocation loc = SmallSourceLocation::Current());

  // Returns the hash of the type of the material.
  HashValue GetTypeHash() const { return type_hash_; }

 private:
  BorrowedMaterialPtr material_;
  // The hash of the type of the material. This is used to verify that the
  // correct type is being requested.
  HashValue type_hash_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_BINDINGS_MATERIAL_H_
