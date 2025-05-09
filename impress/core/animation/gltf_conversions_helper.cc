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

#include "core/animation/gltf_conversions_helper.h"

#include <algorithm>
#include <cstddef>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/optional.h"
#include "absl/types/span.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/animation/schemas/gltf_animation_generated.h"
#include "core/loader/provider/gltf/accessor_reader.h"
#include "core/loader/provider/gltf/dense_data_access.h"
#include "core/loader/provider/gltf/gltf.proto.imp.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::animation {
namespace {

using ::imp::loader::details::AccessorReader;
using ::imp::loader::details::DenseDataAccess;

}  // namespace

template <>
absl::StatusOr<flatbuffers::Offset<void>> Serialize(
    schemas::ChannelFloat3 type, absl::Span<const float> time_data,
    absl::Span<const float> value_data, flatbuffers::FlatBufferBuilder* fbb) {
  auto times = CreateVector<schemas::FrameTime>(time_data, fbb);
  switch (type) {
    case schemas::ChannelFloat3::kStep: {
      auto values = CreateVector<schemas::StepFrameFloat3>(value_data, fbb);
      return schemas::CreateStepCurveFloat3(*fbb, times, values).Union();
      break;
    }
    case schemas::ChannelFloat3::kLinear: {
      auto values = CreateVector<schemas::LinearFrameFloat3>(value_data, fbb);
      return schemas::CreateLinearCurveFloat3(*fbb, times, values).Union();
      break;
    }
    case schemas::ChannelFloat3::kCubic: {
      auto values = CreateVector<schemas::CubicFrameFloat3>(value_data, fbb);
      return schemas::CreateCubicCurveFloat3(*fbb, times, values).Union();
      break;
    }
    default: {
      break;
    }
  }
  return absl::InternalError("Invalid animation");
}

template <>
absl::StatusOr<flatbuffers::Offset<void>> Serialize(
    schemas::ChannelFloat4 type, absl::Span<const float> time_data,
    absl::Span<const float> value_data, flatbuffers::FlatBufferBuilder* fbb) {
  auto times = CreateVector<schemas::FrameTime>(time_data, fbb);
  switch (type) {
    case schemas::ChannelFloat4::kStep: {
      auto values = CreateVector<schemas::StepFrameFloat4>(value_data, fbb);
      return schemas::CreateStepCurveFloat4(*fbb, times, values).Union();
      break;
    }
    case schemas::ChannelFloat4::kLinear: {
      auto values = CreateVector<schemas::LinearFrameFloat4>(value_data, fbb);
      return schemas::CreateLinearCurveFloat4(*fbb, times, values).Union();
      break;
    }
    case schemas::ChannelFloat4::kCubic: {
      auto values = CreateVector<schemas::CubicFrameFloat4>(value_data, fbb);
      return schemas::CreateCubicCurveFloat4(*fbb, times, values).Union();
      break;
    }
    default: {
      break;
    }
  }
  return absl::InternalError("Invalid animation");
}

template <>
absl::StatusOr<flatbuffers::Offset<void>> Serialize(
    schemas::ChannelQuatf type, absl::Span<const float> time_data,
    absl::Span<const float> value_data, flatbuffers::FlatBufferBuilder* fbb) {
  auto times = CreateVector<schemas::FrameTime>(time_data, fbb);
  switch (type) {
    case schemas::ChannelQuatf::kStep: {
      auto values = CreateVector<schemas::StepFrameQuatf>(value_data, fbb);
      return schemas::CreateStepCurveQuatf(*fbb, times, values).Union();
      break;
    }
    case schemas::ChannelQuatf::kLinear: {
      auto values = CreateVector<schemas::LinearFrameQuatf>(value_data, fbb);
      return schemas::CreateLinearCurveQuatf(*fbb, times, values).Union();
      break;
    }
    case schemas::ChannelQuatf::kCubic: {
      auto values = CreateVector<schemas::CubicFrameQuatf>(value_data, fbb);
      return schemas::CreateCubicCurveQuatf(*fbb, times, values).Union();
      break;
    }
    default: {
      break;
    }
  }
  return absl::InternalError("Invalid animation");
}

template <>
absl::StatusOr<flatbuffers::Offset<void>> Serialize(
    schemas::ChannelFloat type, absl::Span<const float> time_data,
    absl::Span<const float> value_data, flatbuffers::FlatBufferBuilder* fbb) {
  auto times = CreateVector<schemas::FrameTime>(time_data, fbb);
  switch (type) {
    case schemas::ChannelFloat::kStep: {
      auto values = CreateVector<schemas::StepFrameFloat>(value_data, fbb);
      return schemas::CreateStepCurveFloat(*fbb, times, values).Union();
      break;
    }
    case schemas::ChannelFloat::kLinear: {
      auto values = CreateVector<schemas::LinearFrameFloat>(value_data, fbb);
      return schemas::CreateLinearCurveFloat(*fbb, times, values).Union();
      break;
    }
    case schemas::ChannelFloat::kCubic: {
      auto values = CreateVector<schemas::CubicFrameFloat>(value_data, fbb);
      return schemas::CreateCubicCurveFloat(*fbb, times, values).Union();
      break;
    }
    default: {
      break;
    }
  }
  return absl::InternalError("Invalid animation");
}

template <>
absl::StatusOr<flatbuffers::Offset<void>> Serialize(
    schemas::ChannelFloat2 type, absl::Span<const float> time_data,
    absl::Span<const float> value_data, flatbuffers::FlatBufferBuilder* fbb) {
  auto times = CreateVector<schemas::FrameTime>(time_data, fbb);
  switch (type) {
    case schemas::ChannelFloat2::kStep: {
      auto values = CreateVector<schemas::StepFrameFloat2>(value_data, fbb);
      return schemas::CreateStepCurveFloat2(*fbb, times, values).Union();
      break;
    }
    case schemas::ChannelFloat2::kLinear: {
      auto values = CreateVector<schemas::LinearFrameFloat2>(value_data, fbb);
      return schemas::CreateLinearCurveFloat2(*fbb, times, values).Union();
      break;
    }
    case schemas::ChannelFloat2::kCubic: {
      auto values = CreateVector<schemas::CubicFrameFloat2>(value_data, fbb);
      return schemas::CreateCubicCurveFloat2(*fbb, times, values).Union();
      break;
    }
    default: {
      break;
    }
  }
  return absl::InternalError("Invalid animation");
}

absl::StatusOr<absl::Span<const float>> GetTimeData(
    const imp::gltf::Gltf& gltf, const AnimationSampler sampler) {
  // Fetch and verify the input accessor.
  if (!sampler.input) return absl::InternalError("No input");
  MP_ASSIGN_OR_RETURN(AccessorReader reader,
                   AccessorReader::Create(gltf, *sampler.input));

  if (reader.GetComponentType() != imp::gltf::ComponentType::FLOAT ||
      reader.GetStride() != sizeof(float)) {
    return absl::InternalError("Invalid time data");
  }

  DenseDataAccess dense_data_access = reader.GetData();
  return absl::Span<const float>(dense_data_access.ReadRawData<float>(),
                                 reader.GetCount());
}

absl::Status ValidateKeyframeData(absl::Span<const float> time_data,
                                  absl::Span<const float> value_data) {
  for (size_t i = 0, c = time_data.size(); i < c; i++) {
    float f = time_data[i];
    if (f < 0.0f || (i && f < time_data[i - 1])) {
      return absl::InternalError("Invalid channel");
    }
  }

  return absl::OkStatus();
}

absl::optional<Domain> MergeDomains(const absl::optional<Domain>& a,
                                    const absl::optional<Domain>& b) {
  if (!a && !b) return {};
  if (!a) return b;
  if (!b) return a;
  return Domain{std::min(a->min, b->min), std::max(a->max, b->max)};
}

}  // namespace imp::animation
