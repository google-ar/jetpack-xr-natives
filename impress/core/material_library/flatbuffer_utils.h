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

#ifndef THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_SCHEMA_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_SCHEMA_HELPERS_H_

#include "filament/filament/include/filament/TextureSampler.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/math/mat.h"
#include "core/math/vec.h"

namespace imp {

// Converts a flatbuffer sampler schema to a filament::TextureSampler.
// Note: the static casts are safe because the enums are verified to match in
// flatbuffer_utils.cc.
// Note: this is a template so that it can be used with both the Impress and
// Split Engine schemas. The split engine schemas are also verified to match.
template <typename SamplerSchema>
filament::TextureSampler ConvertSampler(const SamplerSchema& schema) {
  filament::TextureSampler sampler;
  sampler.setMagFilter(
      static_cast<filament::TextureSampler::MagFilter>(schema.mag_filter()));
  sampler.setMinFilter(
      static_cast<filament::TextureSampler::MinFilter>(schema.min_filter()));
  sampler.setWrapModeS(
      static_cast<filament::TextureSampler::WrapMode>(schema.wrap_mode_s()));
  sampler.setWrapModeT(
      static_cast<filament::TextureSampler::WrapMode>(schema.wrap_mode_t()));
  sampler.setWrapModeR(
      static_cast<filament::TextureSampler::WrapMode>(schema.wrap_mode_r()));
  sampler.setCompareMode(
      static_cast<filament::TextureSampler::CompareMode>(schema.compare_mode()),
      static_cast<filament::TextureSampler::CompareFunc>(
          schema.compare_func()));
  sampler.setAnisotropy(schema.anisotropy_log2());
  return sampler;
}

// Converts a filament::TextureSampler to a flatbuffer sampler schema.
// Note: the static casts are safe because the enums are verified to match in
// flatbuffer_utils.cc.
// Note: this is a template so that it can be used with both the Impress and
// Split Engine schemas. The split engine schemas are also verified to match.
template <typename TextureSamplerCreator>
flatbuffers::Offset<typename TextureSamplerCreator::TextureSampler>
CreateTextureSampler(flatbuffers::FlatBufferBuilder& builder,
                     const filament::TextureSampler& sampler) {
  return TextureSamplerCreator::CreateTextureSampler(
      builder,
      static_cast<typename TextureSamplerCreator::MinFilter>(
          sampler.getMinFilter()),
      static_cast<typename TextureSamplerCreator::MagFilter>(
          sampler.getMagFilter()),
      static_cast<typename TextureSamplerCreator::WrapMode>(
          sampler.getWrapModeS()),
      static_cast<typename TextureSamplerCreator::WrapMode>(
          sampler.getWrapModeT()),
      static_cast<typename TextureSamplerCreator::WrapMode>(
          sampler.getWrapModeR()),
      static_cast<typename TextureSamplerCreator::CompareMode>(
          sampler.getCompareMode()),
      static_cast<typename TextureSamplerCreator::CompareFunc>(
          sampler.getCompareFunc()),
      sampler.getAnisotropy());
}

template <typename BoolSchema>
bool FromBoolFlatbuffer(const BoolSchema& b) {
  return b.value();
}

template <typename FloatSchema>
float FromFloatFlatbuffer(const FloatSchema& f1) {
  return f1.value();
}

template <typename Float2Schema>
float2 FromFloat2Flatbuffer(const Float2Schema& f2) {
  return float2(f2.x(), f2.y());
}

template <typename Float3Schema>
float3 FromFloat3Flatbuffer(const Float3Schema& f3) {
  return float3(f3.x(), f3.y(), f3.z());
}

template <typename Float4Schema>
float4 FromFloat4Flatbuffer(const Float4Schema& f4) {
  return float4(f4.x(), f4.y(), f4.z(), f4.w());
}

template <typename Mat3fSchema>
mat3f FromMat3fFlatbuffer(const Mat3fSchema& m) {
  return mat3f(m.m00(), m.m01(), m.m02(), m.m10(), m.m11(), m.m12(), m.m20(),
               m.m21(), m.m22());
}

template <typename Mat4, typename Mat4Schema>
Mat4 FromMat4Flatbuffer(const Mat4Schema& m) {
  return Mat4(m.m00(), m.m01(), m.m02(), m.m03(), m.m10(), m.m11(), m.m12(),
              m.m13(), m.m20(), m.m21(), m.m22(), m.m23(), m.m30(), m.m31(),
              m.m32(), m.m33());
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_SCHEMA_HELPERS_H_
