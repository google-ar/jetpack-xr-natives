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

#include "core/split_engine/flatbuffer_size_calculator.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace imp::split_engine {

namespace {

// A flat buffer reference is 4 bytes.
constexpr size_t kReferenceSize = 4;

}  // namespace

void FlatbufferSizeCalculator::TrackMinAlign(size_t elem_size) {
  min_align_ = std::max(min_align_, elem_size);
  if (elem_size > min_align_) min_align_ = elem_size;
}

void FlatbufferSizeCalculator::PreAlign() { Pad(min_align_); }

void FlatbufferSizeCalculator::Pad(size_t alignment) {
  offset_ = alignment * ((offset_ + alignment - 1) / alignment);
}

FlatbufferSizeCalculator& FlatbufferSizeCalculator::AddReferenceVector(
    size_t length) {
  AddVector(length, kReferenceSize);
  return *this;
}

FlatbufferSizeCalculator& FlatbufferSizeCalculator::AddVector(
    size_t length, size_t element_size) {
  offset_ += 4;                      // size_field
  offset_ += element_size * length;  // data;
  TrackMinAlign(4);                  // For length field
  TrackMinAlign(element_size);
  Pad(4);  // For length field
  return *this;
}

FlatbufferSizeCalculator& FlatbufferSizeCalculator::AddVectorOfStructs(
    size_t length, size_t element_size) {
  offset_ += 4;                      // size_field
  offset_ += element_size * length;  // data;
  TrackMinAlign(4);                  // For length field
  Pad(4);                            // For length field
  return *this;
}

FlatbufferSizeCalculator& FlatbufferSizeCalculator::AddTable(
    const std::vector<size_t>& field_sizes) {
  // Compute size contributions of the table contents.
  for (size_t field_size : field_sizes) {
    // Table data should be aligned.
    TrackMinAlign(field_size);
    Pad(field_size);
    offset_ += field_size;
  }

  // Compute size contributions of the vtable.
  Pad(4);
  offset_ += 4;                             // table offset
  offset_ += (field_sizes.size() + 2) * 2;  // vtable size
  return *this;
}

// LINT.IfChange
FlatbufferSizeCalculator& FlatbufferSizeCalculator::AddImageParams() {
  return AddTable({
      1,  // level
      1,  // format
      1,  // type
      1,  // alignment
      4,  // left
      4,  // top
      4   // stride
  });
}

FlatbufferSizeCalculator& FlatbufferSizeCalculator::AddPixelBuffer() {
  return AddTable({
      kReferenceSize  // buffer
  });
}

FlatbufferSizeCalculator& FlatbufferSizeCalculator::AddTexture() {
  return AddTable({
      8,               // id
      4,               // width
      4,               // height
      2,               // format
      1,               // levels
      1,               // sampler
      1,               // mips
      kReferenceSize,  // image_params
      kReferenceSize   // pixel_buffers
  });
}

FlatbufferSizeCalculator& FlatbufferSizeCalculator::AddAttributeVector(
    size_t num_attributes) {
  const size_t kAttributeSize = 12;
  return AddVectorOfStructs(num_attributes, kAttributeSize);
}

FlatbufferSizeCalculator& FlatbufferSizeCalculator::AddFloat3Vector(
    size_t num_elements) {
  const size_t kFloat3Size = 12;
  return AddVectorOfStructs(num_elements, kFloat3Size);
}

FlatbufferSizeCalculator& FlatbufferSizeCalculator::AddVertexBlockInfo() {
  return AddTable({
      kReferenceSize,  // attributes
      kReferenceSize,  // buffer
      4,               // stride
  });
}

FlatbufferSizeCalculator& FlatbufferSizeCalculator::AddVertexBufferInfo() {
  return AddTable({
      8,               // vertex_count
      kReferenceSize,  // blocks
      1                // advanced_skinning
  });
}

FlatbufferSizeCalculator& FlatbufferSizeCalculator::AddVertexBuffer() {
  return AddTable({
      8,               // id
      kReferenceSize,  // buffer
      1                // advanced_skinning
  });
}

FlatbufferSizeCalculator& FlatbufferSizeCalculator::AddIndexBufferInfo() {
  return AddTable({
      1,              // type
      kReferenceSize  // buffer
  });
}

FlatbufferSizeCalculator& FlatbufferSizeCalculator::AddIndexBuffer() {
  return AddTable({
      8,               // id
      kReferenceSize,  // buffer
      1,               // vertex_access_flags
  });
}

FlatbufferSizeCalculator& FlatbufferSizeCalculator::AddImageContents() {
  return AddTable({
      4,               // width
      4,               // height
      kReferenceSize,  // memory
  });
}

FlatbufferSizeCalculator& FlatbufferSizeCalculator::AddFaceOffsets() {
  return AddTable({
      4,  // px
      4,  // nx
      4,  // py
      4,  // ny
      4,  // pz
      4,  // nz
  });
}

FlatbufferSizeCalculator& FlatbufferSizeCalculator::AddCubemapLevel() {
  return AddTable({
      kReferenceSize,  // face_offsets
      4                // face_size
  });
}

FlatbufferSizeCalculator&
FlatbufferSizeCalculator::AddCubemapLevelImageContents() {
  return AddTable({
      kReferenceSize,  // cubemap_level
      kReferenceSize,  // stitched_face_image
  });
}

FlatbufferSizeCalculator& FlatbufferSizeCalculator::AddSphericalHarmonics() {
  return AddTable({
      kReferenceSize,  // coefficients
      1,               // num_bands
  });
}

FlatbufferSizeCalculator&
FlatbufferSizeCalculator::AddImageBasedLightingAsset() {
  return AddTable({
      8,               // id
      kReferenceSize,  // ibl_cubemap_level_image_contents
      kReferenceSize,  // skybox_cubemap_level_image_contents
      kReferenceSize,  // spherical_harmonics
  });
}

FlatbufferSizeCalculator&
FlatbufferSizeCalculator::AddMorphTargetAttributeInfo() {
  return AddTable({
      kReferenceSize,  // positions
      kReferenceSize,  // tangents
  });
}

FlatbufferSizeCalculator& FlatbufferSizeCalculator::AddMorphTargetBufferInfo() {
  return AddTable({
      8,               // vertex_count
      kReferenceSize,  // targets
  });
}

FlatbufferSizeCalculator& FlatbufferSizeCalculator::AddMorphTargetBuffer() {
  return AddTable({
      8,               // id
      kReferenceSize,  // buffer
  });
}

FlatbufferSizeCalculator&
FlatbufferSizeCalculator::AddCubemapLevelImageContentsAndDependentData(
    size_t image_buffer_size) {
  AddFaceOffsets();
  AddCubemapLevel();
  AddVector(image_buffer_size, sizeof(uint8_t));
  AddImageContents();
  AddCubemapLevelImageContents();
  return *this;
}

FlatbufferSizeCalculator& FlatbufferSizeCalculator::AddTextureAndDependentData(
    const std::vector<size_t>& buffer_sizes) {
  const size_t kNumBuffers = buffer_sizes.size();
  for (size_t i = 0; i < kNumBuffers; ++i) {
    AddImageParams();
  }
  // Create a vector of references to the image params.
  AddReferenceVector(kNumBuffers);

  for (size_t i = 0; i < kNumBuffers; ++i) {
    // Create a vector of the buffer contents.
    AddVector(buffer_sizes[i], sizeof(uint8_t));
    AddPixelBuffer();
  }
  // Create a vector of references to the pixel buffers.
  AddReferenceVector(kNumBuffers);

  AddTexture();
  return *this;
}

FlatbufferSizeCalculator&
FlatbufferSizeCalculator::AddVertexBufferAndDependentData(
    size_t vertex_buffer_size, size_t attribute_count) {
  AddAttributeVector(attribute_count);
  AddVector(vertex_buffer_size, sizeof(uint8_t));
  AddVertexBlockInfo();
  AddReferenceVector(1);
  AddVertexBufferInfo();
  AddVertexBuffer();
  return *this;
}

FlatbufferSizeCalculator&
FlatbufferSizeCalculator::AddIndexBufferAndDependentData(
    size_t index_buffer_size) {
  AddVector(index_buffer_size, sizeof(uint8_t));
  AddIndexBufferInfo();
  AddIndexBuffer();
  return *this;
}

// LINT.ThenChange(//depot/google3/third_party/split_engine/schemas/split_engine_data.fbs)

// LINT.IfChange
FlatbufferSizeCalculator& FlatbufferSizeCalculator::AddAddTextureRequest(
    size_t num_textures) {
  return AddTable({
      kReferenceSize  // textures
  });
}

FlatbufferSizeCalculator& FlatbufferSizeCalculator::AddAddMeshData() {
  return AddTable({
      kReferenceSize,  // vertex_buffers
      kReferenceSize,  // index_buffers
  });
}

FlatbufferSizeCalculator&
FlatbufferSizeCalculator::AddAddImageBasedLightingAssets() {
  return AddTable({
      kReferenceSize,  // image_based_lighting_assets
  });
}

FlatbufferSizeCalculator& FlatbufferSizeCalculator::AddAddMorphTargetBuffers() {
  return AddTable({
      kReferenceSize,  // morph_target_buffers
  });
}

FlatbufferSizeCalculator& FlatbufferSizeCalculator::AddBeginMessageGroup() {
  return AddTable({});
}

FlatbufferSizeCalculator& FlatbufferSizeCalculator::AddEndMessageGroup() {
  return AddTable({});
}

FlatbufferSizeCalculator& FlatbufferSizeCalculator::AddMessageGroup() {
  return AddTable({
      8,  // groupId
      4,  // offset
      1   // MessageGroupTypes
  });
}

FlatbufferSizeCalculator& FlatbufferSizeCalculator::AddRequest() {
  return AddTable({
      4,  // offset
      1   // RequestType
  });
}

FlatbufferSizeCalculator& FlatbufferSizeCalculator::AddScratchSpace() {
  // Scratch space is actually a function of the # of tables and fields
  // in the flatbuffer, but for now, just use a fixed amount >> the expected
  // size.
  const size_t kScratchSpaceBytes = 1024;
  offset_ += kScratchSpaceBytes;
  return *this;
}

// LINT.ThenChange(//depot/google3/third_party/split_engine/schemas/split_engine_ipc.fbs)

FlatbufferSizeCalculator& FlatbufferSizeCalculator::Finish() {
  offset_ += kReferenceSize;  // root_offset
  PreAlign();
  return *this;
}

size_t FlatbufferSizeCalculator::ComputeSize() const { return offset_; }

}  // namespace imp::split_engine
