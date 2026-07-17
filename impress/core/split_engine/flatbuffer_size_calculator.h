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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_FLATBUFFER_SIZE_CALCULATOR_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_FLATBUFFER_SIZE_CALCULATOR_H_

#include <cstddef>
#include <vector>

namespace imp::split_engine {

// A helper class that can be used to calculate the max size of a flatbuffer
// without actually serializing it and incurring memory overhead. The actual
// flatbuffer size may be smaller due to optimizations by the builder (eg: not
// writing default values, de-duping internal tables)
//
// This is derived from the documentation for the flatbuffer
// ((broken link)), as well as reviewing
// the flatbuffer implementation itself.
class FlatbufferSizeCalculator {
 public:
  FlatbufferSizeCalculator() = default;
  ~FlatbufferSizeCalculator() = default;

  FlatbufferSizeCalculator(const FlatbufferSizeCalculator&) = delete;
  FlatbufferSizeCalculator& operator=(const FlatbufferSizeCalculator&) = delete;

  FlatbufferSizeCalculator(FlatbufferSizeCalculator&&) = delete;
  FlatbufferSizeCalculator& operator=(FlatbufferSizeCalculator&&) = delete;

  // Adds a vector of given length containing elements of given size.
  FlatbufferSizeCalculator& AddVector(size_t length, size_t element_size);

  // Adds a string of given length (without null terminator).
  FlatbufferSizeCalculator& AddString(size_t length);

  // Adds a vector of given length containing structs of given size.
  FlatbufferSizeCalculator& AddVectorOfStructs(size_t length,
                                               size_t element_size);

  // Adds a vector of references of a given length.
  FlatbufferSizeCalculator& AddReferenceVector(size_t length);

  // Each method adds a table from the split engine schema of the same name.
  FlatbufferSizeCalculator& AddImageParams();
  FlatbufferSizeCalculator& AddPixelBuffer();
  FlatbufferSizeCalculator& AddTexture(bool has_name);
  FlatbufferSizeCalculator& AddAttributeVector(size_t num_attributes);
  FlatbufferSizeCalculator& AddVertexBlockInfo();
  FlatbufferSizeCalculator& AddVertexBufferInfo();
  FlatbufferSizeCalculator& AddVertexBuffer();
  FlatbufferSizeCalculator& AddIndexBufferInfo();
  FlatbufferSizeCalculator& AddIndexBuffer();
  FlatbufferSizeCalculator& AddImageContents();
  FlatbufferSizeCalculator& AddFaceOffsets();
  FlatbufferSizeCalculator& AddCubemapLevel();
  FlatbufferSizeCalculator& AddCubemapLevelImageContents();
  FlatbufferSizeCalculator& AddSphericalHarmonics();
  FlatbufferSizeCalculator& AddFloat3Vector(size_t num_elements);
  FlatbufferSizeCalculator& AddImageBasedLightingAsset();
  FlatbufferSizeCalculator& AddMorphTargetAttributeInfo();
  FlatbufferSizeCalculator& AddMorphTargetBufferInfo();
  FlatbufferSizeCalculator& AddMorphTargetBuffer();
  FlatbufferSizeCalculator& AddMessageGroup();
  FlatbufferSizeCalculator& AddBeginMessageGroup();
  FlatbufferSizeCalculator& AddEndMessageGroup();
  FlatbufferSizeCalculator& AddAddTextureRequest(size_t num_textures);
  FlatbufferSizeCalculator& AddAddImageBasedLightingAssets();
  FlatbufferSizeCalculator& AddAddMorphTargetBuffers();

  FlatbufferSizeCalculator& AddAddMeshData();
  FlatbufferSizeCalculator& AddRequest();

  // During flatbuffer construction, there is some amount of scratch space that
  // is temporarily used. Use this function to include that overhead in the
  // size estimate.
  FlatbufferSizeCalculator& AddScratchSpace();

  // Finishes the flatbuffer. This must be the final operation after all Add*
  // methods have been called. ComputeSize() can still be called after Finish().
  FlatbufferSizeCalculator& Finish();

  // Given a vector of buffer sizes, computes the size of a texture and all the
  // dependent data. This assumes a specific order of assembly of the texture
  // and its dependencies, otherwise the size may differ (eg: due to padding).
  // Expected order:
  //   - ImageParams for each buffer
  //   - Vector of references for the ImageParams
  //   - For each buffer:
  //     - A vector containing the buffer contents
  //     - A PixelBuffer object for the buffer
  //   - Vector of references to the PixelBuffer objects
  //   - The texture object
  //   - The name string (optional)
  FlatbufferSizeCalculator& AddTextureAndDependentData(
      const std::vector<size_t>& buffer_sizes, size_t name_size = 0);

  // Given an image buffer size, computes the size of a
  // CubemapLevelImageContents and all the
  // dependent data. This assumes a specific order of assembly of the
  // CubemapLevelImageContents and its dependencies, otherwise the size may
  // differ (eg: due to padding). Expected order:
  //   - Vector of image buffer bytes
  //   - Image contents object
  //   - Face offsets object
  //   - Cubemap level object
  //   - Cubemap level image contents object
  FlatbufferSizeCalculator& AddCubemapLevelImageContentsAndDependentData(
      size_t image_buffer_size);

  // Given a number of vertices and attributes, computes the size of a vertex
  // buffer and all the dependent data. This assumes a specific order of
  // assembly of the vertex buffer and its dependencies, otherwise the size may
  // differ (eg: due to padding). Expected order:
  //   - Vector of Attribute objects
  //   - Vector of vertex buffer data
  //   - VertexBlockInfo
  //   - VertexBufferInfo
  //   - Vector referencing VertexBlockInfo
  //   - The VertexBuffer
  FlatbufferSizeCalculator& AddVertexBufferAndDependentData(
      size_t vertex_buffer_size, size_t attribute_count);

  // Given a number of indices, computes the size of an index
  // buffer and all the dependent data. This assumes a specific order of
  // assembly of the index buffer and its dependencies, otherwise the size may
  // differ (eg: due to padding). Expected order:
  //   - Vector of index buffer data
  //   - IndexBufferInfo
  //   - IndexBuffer
  FlatbufferSizeCalculator& AddIndexBufferAndDependentData(
      size_t index_buffer_size);

  // Return the size of the flatbuffer corresponding to the series of AddX
  // operations. This can still be called after Finish() has been called.
  size_t ComputeSize() const;

 private:
  // Equivalent to the flatbuffer builder, need to track the largest element
  // to determine the final alignment when the flatbuffer is finished.
  void TrackMinAlign(size_t elem_size);

  // Pre-aligns the buffer to the largest element size.
  void PreAlign();

  // Applies padding to the buffer to align to the given size.
  void Pad(size_t alignment);

  FlatbufferSizeCalculator& AddTable(const std::vector<size_t>& field_sizes);
  size_t offset_ = 0;
  size_t min_align_ = 1;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_FLATBUFFER_SIZE_CALCULATOR_H_
