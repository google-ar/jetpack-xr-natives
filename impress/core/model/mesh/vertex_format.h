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

#ifndef THIRD_PARTY_IMPRESS_CORE_MODEL_MESH_VERTEX_FORMAT_H_
#define THIRD_PARTY_IMPRESS_CORE_MODEL_MESH_VERTEX_FORMAT_H_

#include <initializer_list>

#include "absl/types/optional.h"
#include "filament/filament/include/filament/VertexBuffer.h"

namespace imp {

// A list of attributes and types to describe an interleaved vertex buffer.
class VertexFormat {
 public:
  using VertexAttribute = filament::VertexAttribute;
  using AttributeType = filament::VertexBuffer::AttributeType;

  // Every attribute has a type, e.g. attribute POSITION could have type FLOAT3.
  struct AttributeInfo {
    VertexAttribute attribute;
    AttributeType type;
    bool normalized = false;
  };

  // Matches filament::backend::MAX_VERTEX_ATTRIBUTE_COUNT.
  static constexpr size_t kMaxAttributes = 16;

  VertexFormat();

  template <typename Iterator>
  VertexFormat(Iterator begin, Iterator end);

  // Enable braced initializer list, e.g.:
  //   VertexFormat({{VertexAttribute::POSITION, AttributeType::FLOAT3}});
  VertexFormat(std::initializer_list<AttributeInfo> attributes);

  // Appends |attribute| to the internal list of attributes. Fatals if an
  // attribute is repeated or the number of attributes exceeds kMaxAttributes.
  void AppendAttribute(const AttributeInfo& attribute);

  // Returns the size of the whole vertex in bytes.
  size_t GetVertexSize() const;

  size_t GetNumAttributes() const;

  // Returns the index for |attribute|, or nullopt if not present. This
  // is recalculated and not cached to save space and is only a short vector.
  absl::optional<size_t> GetIndexForAttribute(VertexAttribute attribute) const;

  // Returns the attribute at |index|. Fatals if out of bounds.
  const AttributeInfo& GetAttributeAt(size_t index) const;

  // Returns the offset of the attribute at |index|. Fatals if out of bounds.
  // This is recalculated and not cached to save space and is only a short
  // vector.
  size_t GetAttributeOffsetAt(size_t index) const;

  // Tests if two VertexFormats are equal.
  bool operator==(const VertexFormat& rhs) const;
  bool operator!=(const VertexFormat& rhs) const;

  // Returns the size of a vertex attribute.
  static size_t GetAttributeSize(const AttributeInfo& attr);

 private:
  // This class's data must be restricted to POD, so we can use static const
  // VertexFormats for dynamic rendering.
  AttributeInfo attributes_[kMaxAttributes];
  size_t num_attributes_ = 0;
  size_t vertex_size_ = 0;
};

template <typename Iterator>
VertexFormat::VertexFormat(Iterator begin, Iterator end) {
  for (auto attrib = begin; attrib != end; ++attrib) {
    AppendAttribute(*attrib);
  }
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MODEL_MESH_VERTEX_FORMAT_H_
