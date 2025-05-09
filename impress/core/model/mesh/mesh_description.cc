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

#include "core/model/mesh/mesh_description.h"

#include "core/common/log.h"
#include "core/common/platform_helpers.h"

namespace imp {

bool MeshDescription::operator==(const MeshDescription& rhs) const {
  return vertex_format == rhs.vertex_format && index_type == rhs.index_type &&
         vertex_count == rhs.vertex_count && index_count == rhs.index_count;
}

bool MeshDescription::operator!=(const MeshDescription& rhs) const {
  return !(*this == rhs);
}

size_t MeshDescription::GetIndexSize() const {
  switch (index_type) {
    case IndexType::USHORT:
      return sizeof(uint16_t);
    case IndexType::UINT:
      return sizeof(uint32_t);
    default:
      IMP_LOG(imp::FATAL) << "Unsupported index type: " << static_cast<int>(index_type);
      return 0;
  }
}

}  // namespace imp
