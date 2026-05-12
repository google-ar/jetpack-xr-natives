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

#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <iterator>

#include "core/common/log.h"
#include "absl/types/optional.h"
#include "core/loader/details/bundle_resource_helpers.h"

namespace imp {
namespace {

inline bool operator==(const VertexFormat::AttributeInfo& a,
                       const VertexFormat::AttributeInfo& b) {
  return (a.attribute == b.attribute && a.type == b.type);
}

inline bool operator!=(const VertexFormat::AttributeInfo& a,
                       const VertexFormat::AttributeInfo& b) {
  return !(a == b);
}

}  // namespace

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
  for (int group_idx = 0; group_idx < GetAttributeGroupsCount(); ++group_idx) {
    for (size_t i = 0; i < GetNumAttributes(group_idx); ++i) {
      if (attribute.attribute == GetAttributeAt(i, group_idx).attribute) {
        IMP_LOG(imp::FATAL) << "Repeated vertex attribute " << attribute.attribute;
        return;
      }
    }
  }
  group.attributes[group.num_attributes] = attribute;
  group.vertex_size += GetAttributeSize(attribute);
  ++group.num_attributes;
}

size_t VertexFormat::GetVertexSize(uint8_t group_idx) const {
  return attribute_groups_.size() <= group_idx
             ? 0
             : attribute_groups_[group_idx].vertex_size;
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
      size_t attribute_size =
          GetAttributeSize(attribute_groups_[i].attributes[j]);
      if (attribute_groups_[i].attributes[j].attribute == attribute) {
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
    IMP_LOG(imp::FATAL) << "Group index " << index << " out of bounds "
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
  for (size_t i = 0; i < index; ++i) {
    offset += GetAttributeSize(attribute_groups_[group_idx].attributes[i]);
  }
  return offset;
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
