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

#include <memory>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "apibindings/bindings_object.h"
#include "core/common/hash.h"
#include "core/common/type_traits.h"
#include "core/split_engine/materials/split_engine_material.h"

namespace imp {

// Wraps a Split Engine material pointer so that a pointer to
// BindingsMaterial can be released to Java without affecting the ownership of
// the actual material pointer.
class BindingsMaterial : public BindingsObject {
 public:
  // Constructor.
  template <typename T>
  explicit BindingsMaterial(std::unique_ptr<T> material);

  // Returns the native pointer of the specific material type that is being
  // wrapped.
  template <typename T>
  absl::StatusOr<T*> GetMaterial();

  // Returns the native pointer of the SplitEngineMaterial parent type that is
  // being wrapped.
  split_engine::SplitEngineMaterial* GetBaseMaterial();

 private:
  // Holds a reference to the material. Here we hold a unique_ptr to the
  // material rather than the BorrowedPtr itself because we need to be able to
  // access both the SplitEngineMaterial parent type and the specific material
  // type (for example the WaterReflectMaterial) that inherit from it.
  std::unique_ptr<split_engine::SplitEngineMaterial> material_;
  // The hash of the type of the material. This is used to verify that the
  // correct type is being requested.
  HashValue type_hash_;
};

template <typename T>
BindingsMaterial::BindingsMaterial(std::unique_ptr<T> material)
    : material_(std::move(material)) {
  type_hash_ = type_traits::kTypeHash<T>;
}

template <typename T>
absl::StatusOr<T*> BindingsMaterial::GetMaterial() {
  if (type_hash_ != type_traits::kTypeHash<T>) {
    return absl::InvalidArgumentError(
        "Provided material handle is not of the correct type.");
  }

  return static_cast<T*>(material_.get());
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_BINDINGS_MATERIAL_H_
