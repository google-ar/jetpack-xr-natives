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
//
// This file contains the definitions of the Filament resource Owned and
// BorrowedPtrs used in Split Engine. Using this, you can build Owned and
// BorrowedPtrs with the SplitEngineFilamentResourceDeleter.
// Example:
//  filament::VertexBuffer* vertex_buffer = BuildVertexBuffer(...);
//    OwnedVertexBufferPtr owned_vertex_buffer =
//       OwnedVertexBufferPtr(vertex_buffer);
//   BorrowedVertexBufferPtr borrowed_vertex_buffer =
//       owned_vertex_buffer.Borrow();

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_FILAMENT_RESOURCE_PTRS_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_FILAMENT_RESOURCE_PTRS_H_

#include "filament/filament/include/filament/IndexBuffer.h"
#include "filament/filament/include/filament/Material.h"
#include "filament/filament/include/filament/MorphTargetBuffer.h"
#include "filament/filament/include/filament/VertexBuffer.h"
#include "core/common/owned_ptr.h"
#include "core/view/base_view.h"

namespace imp::split_engine {

// Deleter for Filament resources, as they need a custom Deleter to be
// destroyed by the Filament engine.
template <typename T>
class SplitEngineFilamentResourceDeleter {
 public:
  void operator()(T* filament_resource) {
    BaseView::GetSharedEngine()->destroy(filament_resource);
  }
};

// Used to create an OwnedPtr for Filament resources. This is a convenient way
// to create an OwnedPtr for Filament resources, as they need a custom Deleter
// to be destroyed by the Filament engine.
template <typename T>
using OwnedFilamentResourcePtr =
    imp::OwnedPtr<T, SplitEngineFilamentResourceDeleter<T>>;

// Used to create an OwnedPtr for a Filament MorphTargetBuffer that uses the
// SplitEngineFilamentResourceDeleter.
using OwnedMorphTargetBufferPtr =
    OwnedFilamentResourcePtr<filament::MorphTargetBuffer>;

// Used to create a BorrowedPtr for a Filament MorphTargetBuffer that uses the
// SplitEngineFilamentResourceDeleter.
using BorrowedMorphTargetBufferPtr =
    imp::BorrowedPtr<filament::MorphTargetBuffer>;

// Used to create a stable OwnedPtr for a Filament VertexBuffer that uses the
// SplitEngineFilamentResourceDeleter.
using OwnedVertexBufferPtr = OwnedFilamentResourcePtr<filament::VertexBuffer>;

// Used to create a stable BorrowedPtr for a Filament VertexBuffer that uses the
// SplitEngineFilamentResourceDeleter.
using BorrowedVertexBufferPtr = imp::BorrowedPtr<filament::VertexBuffer>;

// Used to create a stable OwnedPtr for a Filament IndexBuffer that uses the
// SplitEngineFilamentResourceDeleter.
using OwnedIndexBufferPtr = OwnedFilamentResourcePtr<filament::IndexBuffer>;

// Used to create a stable BorrowedPtr for a Filament IndexBuffer that uses the
// SplitEngineFilamentResourceDeleter.
using BorrowedIndexBufferPtr = imp::BorrowedPtr<filament::IndexBuffer>;

// Used to create a stable OwnedPtr for a Filament Material that uses the
// SplitEngineFilamentResourceDeleter.
using OwnedFilamentMaterialPtr = OwnedFilamentResourcePtr<filament::Material>;

// Used to create a stable BorrowedPtr for a Filament Material that uses the
// SplitEngineFilamentResourceDeleter.
using BorrowedFilamentMaterialPtr = imp::BorrowedPtr<filament::Material>;

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_FILAMENT_RESOURCE_PTRS_H_
