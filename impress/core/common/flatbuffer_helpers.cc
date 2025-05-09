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

#include "core/common/flatbuffer_helpers.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "absl/types/span.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/common/buffer_access.h"
#include "core/common/optional_error.h"
#include "core/common/schemas/math_generated.h"
#include "core/math/mat.h"

namespace imp {

OptionalError ReleaseToBufferAccess(flatbuffers::FlatBufferBuilder* fbb,
                                    BufferAccess* access) {
  size_t size = 0;
  size_t offset = 0;
  std::unique_ptr<uint8_t[]> storage(fbb->ReleaseRaw(size, offset));
  auto span = absl::MakeSpan(storage.get() + offset, size - offset);

  if (!storage || !size || (offset >= size)) {
    return Error("Could not retrieve buffer from builder");
  }

  *access = BufferAccess(std::move(storage), span);
  return NoError();
}

std::vector<mat3f> FromFlatbuffer(schemas::Mat3fArray* v) {
  std::vector<mat3f> value;
  value.reserve(v->mats()->size());
  for (size_t i = 0; i < v->mats()->size(); ++i) {
    const schemas::Mat3f* mat = (*v->mats())[i];
    value.emplace_back(mat3f{mat->m00(), mat->m01(), mat->m02(),  //
                             mat->m10(), mat->m11(), mat->m12(),  //
                             mat->m20(), mat->m21(), mat->m22()});
  }
  return value;
}

}  // namespace imp
