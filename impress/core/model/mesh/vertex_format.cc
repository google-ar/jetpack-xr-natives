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

#include "core/model/mesh/vertex_format.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <iterator>

#include "core/common/log.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "core/loader/details/bundle_resource_helpers.h"

namespace imp {
namespace {

inline bool operator==(const VertexFormat::AttributeInfo& a,
                       const VertexFormat::AttributeInfo& b) {
  return (a.attribute == b.attribute && a.type == b.type &&
          a.normalized == b.normalized &&
          a.attribute_group_override == b.attribute_group_override &&
          a.byte_offset == b.byte_offset);
}

inline bool operator!=(const VertexFormat::AttributeInfo& a,
                       const VertexFormat::AttributeInfo& b) {
  return !(a == b);
}

}  // namespace

absl::string_view VertexAttributeToString(
    VertexFormat::VertexAttribute attribute) noexcept {
  switch (attribute) {
    case VertexFormat::VertexAttribute::POSITION:
      return "POSITION";
    case VertexFormat::VertexAttribute::TANGENTS:
      return "TANGENTS";
    case VertexFormat::VertexAttribute::COLOR:
      return "COLOR";
    case VertexFormat::VertexAttribute::UV0:
      return "UV0";
    case VertexFormat::VertexAttribute::UV1:
      return "UV1";
    case VertexFormat::VertexAttribute::BONE_INDICES:
      return "BONE_INDICES";
    case VertexFormat::VertexAttribute::BONE_WEIGHTS:
      return "BONE_WEIGHTS";
    case VertexFormat::VertexAttribute::CUSTOM0:
      return "CUSTOM0";
    case VertexFormat::VertexAttribute::CUSTOM1:
      return "CUSTOM1";
    case VertexFormat::VertexAttribute::CUSTOM2:
      return "CUSTOM2";
    case VertexFormat::VertexAttribute::CUSTOM3:
      return "CUSTOM3";
    case VertexFormat::VertexAttribute::CUSTOM4:
      return "CUSTOM4";
    case VertexFormat::VertexAttribute::CUSTOM5:
      return "CUSTOM5";
    case VertexFormat::VertexAttribute::CUSTOM6:
      return "CUSTOM6";
    case VertexFormat::VertexAttribute::CUSTOM7:
      return "CUSTOM7";
  }
  return "UNKNOWN";
}

VertexFormat::VertexFormat() = default;

VertexFormat::VertexFormat(std::initializer_list<AttributeInfo> attributes)
    : VertexFormat(std::begin(attributes), std::end(attributes)) {}

void VertexFormat::AppendAttribute(const AttributeInfo& attribute) {
  const uint8_t group_idx = attribute.attribute_group_override;
  if (group_idx >= attribute_groups_.size()) {
    attribute_groups_.resize(group_idx + 1);
  }
  AttributeGroup& group = attribute_groups_[group_idx];
  if (group.num_attributes == kMaxAttributes) {
    IMP_LOG(imp::FATAL) << "Cannot exceed max attributes size of " << kMaxAttributes;
    return;
  }
  for (size_t g_idx = 0; g_idx < GetAttributeGroupsCount(); ++g_idx) {
    for (size_t i = 0; i < GetNumAttributes(g_idx); ++i) {
      if (attribute.attribute == GetAttributeAt(i, g_idx).attribute) {
        IMP_LOG(imp::FATAL) << "Vertex attribute "
                   << VertexAttributeToString(attribute.attribute)
                   << " is already present in group " << g_idx << " at index "
                   << i;
        return;
      }
    }
  }

  group.attributes[group.num_attributes] = attribute;
  ++group.num_attributes;

  size_t size = GetAttributeSize(attribute);
  size_t offset = GetAttributeOffsetAt(group.num_attributes - 1, group_idx);

  // Check for overlap with existing attributes in the same group.
  for (size_t i = 0; i < group.num_attributes - 1; ++i) {
    const AttributeInfo& existing_attr = group.attributes[i];
    size_t existing_offset = GetAttributeOffsetAt(i, group_idx);
    size_t existing_size = GetAttributeSize(existing_attr);
    if (offset < existing_offset + existing_size &&
        existing_offset < offset + size) {
      IMP_LOG(imp::FATAL) << "Attribute " << VertexAttributeToString(attribute.attribute)
                 << " at offset " << offset << " of size " << size
                 << " overlaps with existing attribute "
                 << VertexAttributeToString(existing_attr.attribute)
                 << " at offset " << existing_offset << " of size "
                 << existing_size;
      return;
    }
  }

  group.vertex_size = std::max(group.vertex_size, offset + size);

  if (group.custom_stride > 0 && group.custom_stride < group.vertex_size) {
    IMP_LOG(imp::FATAL) << "Attribute group " << static_cast<int>(group_idx)
               << " byte stride " << group.custom_stride
               << " should be either 0 or larger than the group vertex size "
               << group.vertex_size;
    return;
  }
}

VertexFormat& VertexFormat::SetGroupByteStride(uint8_t group_idx,
                                               size_t stride) {
  if (group_idx >= attribute_groups_.size()) {
    attribute_groups_.resize(group_idx + 1);
  }
  AttributeGroup& group = attribute_groups_[group_idx];
  if (stride > 0 && stride < group.vertex_size) {
    IMP_LOG(imp::FATAL) << "Attribute group " << static_cast<int>(group_idx)
               << " byte stride " << stride
               << " should be either 0 or larger than the group vertex size "
               << group.vertex_size;
    return *this;
  }
  group.custom_stride = stride;
  return *this;
}

size_t VertexFormat::GetVertexSize(uint8_t group_idx) const {
  if (attribute_groups_.size() <= group_idx) {
    return 0;
  }
  const AttributeGroup& group = attribute_groups_[group_idx];
  return group.custom_stride > 0 ? group.custom_stride : group.vertex_size;
}

size_t VertexFormat::GetNumAttributes(size_t group_idx) const {
  return attribute_groups_.size() <= group_idx
             ? 0
             : attribute_groups_[group_idx].num_attributes;
}

absl::optional<size_t> VertexFormat::GetIndexForAttribute(
    VertexAttribute attribute, size_t group_idx) const {
  if (group_idx >= attribute_groups_.size()) {
    return absl::nullopt;
  }

  for (size_t i = 0; i < attribute_groups_[group_idx].num_attributes; ++i) {
    if (attribute_groups_[group_idx].attributes[i].attribute == attribute) {
      return i;
    }
  }
  return absl::nullopt;
}

absl::optional<VertexFormat::AttributeKey> VertexFormat::GetKeyForAttribute(
    VertexAttribute attribute) const {
  for (size_t i = 0; i < attribute_groups_.size(); ++i) {
    size_t offset = 0;
    for (size_t j = 0; j < attribute_groups_[i].num_attributes; ++j) {
      const AttributeInfo& attr = attribute_groups_[i].attributes[j];
      // If the attribute has a defined offset, use it. Otherwise, it follows
      // the previous attribute, so we use the offset we've accumulated so far.
      if (attr.byte_offset > AttributeInfo::kUnset) {
        offset = attr.byte_offset;
      }
      size_t attribute_size = GetAttributeSize(attr);
      if (attr.attribute == attribute) {
        return VertexFormat::AttributeKey{.group_index = i,
                                          .attribute_offset = offset,
                                          .attribute_size = attribute_size};
      }
      offset += attribute_size;
    }
  }
  return absl::nullopt;
}

const VertexFormat::AttributeInfo& VertexFormat::GetAttributeAt(
    size_t index, size_t group_idx) const {
  if (group_idx >= attribute_groups_.size()) {
    IMP_LOG(imp::FATAL) << "Group index " << group_idx << " out of bounds "
               << attribute_groups_.size();
  }
  if (index >= attribute_groups_[group_idx].num_attributes) {
    IMP_LOG(imp::FATAL) << "Index " << index << " out of bounds "
               << attribute_groups_[group_idx].num_attributes;
  }
  return attribute_groups_[group_idx].attributes[index];
}

size_t VertexFormat::GetAttributeOffsetAt(size_t index,
                                          size_t group_idx) const {
  if (group_idx >= attribute_groups_.size()) {
    IMP_LOG(imp::FATAL) << "Group index " << group_idx << " out of bounds "
               << attribute_groups_.size();
  }
  if (index >= attribute_groups_[group_idx].num_attributes) {
    IMP_LOG(imp::FATAL) << "Index " << index << " out of bounds "
               << attribute_groups_[group_idx].num_attributes;
  }
  size_t offset = 0;
  for (size_t i = 0; i <= index; ++i) {
    const AttributeInfo& attr = attribute_groups_[group_idx].attributes[i];
    // If the attribute has a defined offset, use it. Otherwise, it follows the
    // previous attribute, so we use the offset we've accumulated so far.
    if (attr.byte_offset > AttributeInfo::kUnset) {
      offset = attr.byte_offset;
    }
    if (i == index) {
      return offset;
    }
    // Advance offset to point to the end of the current attribute. This will be
    // used as the start of the next attribute, unless that attribute has its
    // own byte_offset.
    offset += GetAttributeSize(attr);
  }
  // Should be unreachable.
  IMP_LOG(imp::FATAL) << "Failed to find attribute offset.";
  return 0;
}

size_t VertexFormat::GetAttributeGroupsCount() const {
  return attribute_groups_.size();
}

bool VertexFormat::operator==(const VertexFormat& rhs) const {
  if (attribute_groups_.size() != rhs.attribute_groups_.size()) {
    return false;
  }
  for (size_t group = 0; group < attribute_groups_.size(); ++group) {
    if (attribute_groups_[group].vertex_size !=
        rhs.attribute_groups_[group].vertex_size) {
      return false;
    }
    if (attribute_groups_[group].custom_stride !=
        rhs.attribute_groups_[group].custom_stride) {
      return false;
    }
    if (attribute_groups_[group].num_attributes !=
        rhs.attribute_groups_[group].num_attributes) {
      return false;
    }
    for (size_t i = 0; i < attribute_groups_[group].num_attributes; ++i) {
      if (attribute_groups_[group].attributes[i] !=
          rhs.attribute_groups_[group].attributes[i]) {
        return false;
      }
    }
  }
  return true;
}

bool VertexFormat::operator!=(const VertexFormat& rhs) const {
  return !(*this == rhs);
}

size_t VertexFormat::GetAttributeSize(const AttributeInfo& attr) {
  return loader::details::GetAttributeTypeSize(attr.type);
}

}  // namespace imp
