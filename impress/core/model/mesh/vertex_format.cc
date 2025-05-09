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

#include <initializer_list>
#include <iterator>

#include "core/common/log.h"
#include "core/common/platform_helpers.h"
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
  if (num_attributes_ == kMaxAttributes) {
    IMP_LOG(imp::FATAL) << "Cannot exceed max attributes size of " << kMaxAttributes;
    return;
  }
  for (size_t i = 0; i < GetNumAttributes(); ++i) {
    if (attribute.attribute == GetAttributeAt(i).attribute) {
      IMP_LOG(imp::FATAL) << "Repeated vertex attribute " << attribute.attribute;
      return;
    }
  }
  attributes_[num_attributes_] = attribute;
  vertex_size_ += GetAttributeSize(attribute);
  ++num_attributes_;
}

size_t VertexFormat::GetVertexSize() const { return vertex_size_; }

size_t VertexFormat::GetNumAttributes() const { return num_attributes_; }

absl::optional<size_t> VertexFormat::GetIndexForAttribute(
    VertexAttribute attribute) const {
  for (size_t i = 0; i < num_attributes_; ++i) {
    if (attributes_[i].attribute == attribute) {
      return i;
    }
  }
  return absl::nullopt;
}

const VertexFormat::AttributeInfo& VertexFormat::GetAttributeAt(
    size_t index) const {
  if (index >= num_attributes_) {
    IMP_LOG(imp::FATAL) << "Index " << index << " out of bounds " << num_attributes_;
  }
  return attributes_[index];
}

size_t VertexFormat::GetAttributeOffsetAt(size_t index) const {
  if (index >= num_attributes_) {
    IMP_LOG(imp::FATAL) << "Index " << index << " out of bounds " << num_attributes_;
    return 0;
  }
  size_t offset = 0;
  for (size_t i = 0; i < index; ++i) {
    offset += GetAttributeSize(attributes_[i]);
  }
  return offset;
}

bool VertexFormat::operator==(const VertexFormat& rhs) const {
  if (vertex_size_ != rhs.vertex_size_) {
    return false;
  }
  if (num_attributes_ != rhs.num_attributes_) {
    return false;
  }
  for (size_t i = 0; i < num_attributes_; ++i) {
    if (attributes_[i] != rhs.attributes_[i]) {
      return false;
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
