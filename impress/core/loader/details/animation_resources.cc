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

#include "core/loader/details/animation_resources.h"

#include <cstddef>
#include <cstdint>
#include <iterator>
#include <memory>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "flatbuffers/vector.h"
#include "flatbuffers/verifier.h"
#include "core/animation/gltf_animation.h"
#include "core/animation/schemas/gltf_animation_generated.h"
#include "core/common/flatbuffer_helpers.h"
#include "core/common/optional_error.h"
#include "core/loader/provider/schemas/loaded_model_generated.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::loader::details {

namespace {

template <typename T>
absl::Span<const T> MakeSpan(const flatbuffers::Vector<const T *> *vector) {
  return absl::MakeSpan(reinterpret_cast<const T *>(vector->Get(0)),
                        vector->size());
}

OptionalError VerifyTimes(
    absl::Span<const animation::schemas::FrameTime> frame_times) {
  for (size_t index = 1, count = frame_times.size(); index < count; ++index) {
    if (frame_times[index - 1].t() > frame_times[index].t()) {
      return Error("Domain not monotonically increasing.");
    }
  }
  return NoError();
}

OptionalError VerifyChannel(animation::schemas::ChannelFloat3 type,
                            const void *channel) {
  switch (type) {
    default:
    case animation::schemas::ChannelFloat3::NONE:
    case animation::schemas::ChannelFloat3::kConstant:
      // Nothing to verify.
      break;
    case animation::schemas::ChannelFloat3::kStep: {
      const auto *step =
          static_cast<const animation::schemas::StepCurveFloat3 *>(channel);
      if (step->times()->size() == 0 ||
          step->times()->size() != step->values()->size()) {
        return Error("Invalid Step curve");
      }
      MP_RETURN_IF_ERROR(VerifyTimes(MakeSpan(step->times())));
    } break;
    case animation::schemas::ChannelFloat3::kLinear: {
      const auto *linear =
          static_cast<const animation::schemas::LinearCurveFloat3 *>(channel);
      if (linear->times()->size() == 0 ||
          linear->times()->size() != linear->values()->size()) {
        return Error("Invalid Linear curve");
      }
      MP_RETURN_IF_ERROR(VerifyTimes(MakeSpan(linear->times())));
    } break;
    case animation::schemas::ChannelFloat3::kCubic: {
      const auto *cubic =
          static_cast<const animation::schemas::CubicCurveFloat3 *>(channel);
      if (cubic->times()->size() == 0 ||
          cubic->times()->size() != cubic->values()->size()) {
        return Error("Invalid Cubic curve");
      }
      MP_RETURN_IF_ERROR(VerifyTimes(MakeSpan(cubic->times())));
    } break;
  }
  return NoError();
}

OptionalError VerifyChannel(animation::schemas::ChannelQuatf type,
                            const void *channel) {
  switch (type) {
    default:
    case animation::schemas::ChannelQuatf::NONE:
    case animation::schemas::ChannelQuatf::kConstant:
      // Nothing to verify.
      break;
    case animation::schemas::ChannelQuatf::kStep: {
      const auto *step =
          static_cast<const animation::schemas::StepCurveQuatf *>(channel);
      if (step->times()->size() == 0 ||
          step->times()->size() != step->values()->size()) {
        return Error("Invalid Step curve");
      }
      MP_RETURN_IF_ERROR(VerifyTimes(MakeSpan(step->times())));
    } break;
    case animation::schemas::ChannelQuatf::kLinear: {
      const auto *linear =
          static_cast<const animation::schemas::LinearCurveQuatf *>(channel);
      if (linear->times()->size() == 0 ||
          linear->times()->size() != linear->values()->size()) {
        return Error("Invalid Linear curve");
      }
      MP_RETURN_IF_ERROR(VerifyTimes(MakeSpan(linear->times())));
    } break;
    case animation::schemas::ChannelQuatf::kCubic: {
      const auto *cubic =
          static_cast<const animation::schemas::CubicCurveQuatf *>(channel);
      if (cubic->times()->size() == 0 ||
          cubic->times()->size() != cubic->values()->size()) {
        return Error("Invalid Cubic curve");
      }
      MP_RETURN_IF_ERROR(VerifyTimes(MakeSpan(cubic->times())));
    } break;
  }
  return NoError();
}

OptionalError VerifyChannel(animation::schemas::ChannelFloatVector type,
                            const void *channel) {
  switch (type) {
    default:
    case animation::schemas::ChannelFloatVector::NONE: {
      return Error("Invalid Channel");
    } break;
    case animation::schemas::ChannelFloatVector::kConstant: {
      // Constant value, nothing to verify.
    } break;
    case animation::schemas::ChannelFloatVector::kStep: {
      const auto *step =
          static_cast<const animation::schemas::StepCurveFloatVector *>(
              channel);
      if (step->times()->size() == 0 ||
          step->times()->size() != step->values()->size()) {
        return Error("Invalid Step curve");
      }
      MP_RETURN_IF_ERROR(VerifyTimes(MakeSpan(step->times())));
    } break;
    case animation::schemas::ChannelFloatVector::kLinear: {
      const auto *linear =
          static_cast<const animation::schemas::LinearCurveFloatVector *>(
              channel);
      if (linear->times()->size() == 0 ||
          linear->times()->size() != linear->values()->size()) {
        return Error("Invalid Linear curve");
      }
      MP_RETURN_IF_ERROR(VerifyTimes(MakeSpan(linear->times())));
    } break;
    case animation::schemas::ChannelFloatVector::kCubic: {
      const auto *cubic =
          static_cast<const animation::schemas::CubicCurveFloatVector *>(
              channel);
      if (cubic->times()->size() == 0 ||
          cubic->times()->size() != cubic->values()->size()) {
        return Error("Invalid Cubic curve");
      }
      MP_RETURN_IF_ERROR(VerifyTimes(MakeSpan(cubic->times())));
    } break;
  }
  return NoError();
}

OptionalError VerifyGltfAnimationInfo(
    const schemas::GltfAnimationInfo *gltf_info, size_t bone_count) {
  const flatbuffers::Vector<uint8_t> *buffer = gltf_info->buffer();
  if (!buffer) return Error("missing animation");
  flatbuffers::Verifier verifier(buffer->data(), buffer->size());
  if (!verifier.VerifyBuffer<animation::schemas::GltfAnimation>()) {
    return Error("Verification failed");
  }

  const animation::schemas::GltfAnimation *animation =
      gltf_info->buffer_nested_root();

  if (animation->first_t() > animation->last_t()) return Error("Invalid times");
  if (animation->node_animations()->size() != animation->node_targets()->size())
    return Error("mismatched targets");

  if (animation->morph_target_animations()->size() !=
      animation->morph_target_node_targets()->size())
    return Error("mismatched targets");

  for (const animation::schemas::GltfNodeAnimationTarget *target :
       *animation->node_targets()) {
    if (target->bone() >= bone_count) return Error("invalid target");
  }

  for (const animation::schemas::GltfNodeAnimationTarget *mt_target :
       *animation->morph_target_node_targets()) {
    if (mt_target->bone() >= bone_count) return Error("invalid target");
  }

  for (const animation::schemas::GltfNodeAnimation *node_animation :
       *animation->node_animations()) {
    MP_RETURN_IF_ERROR(
        VerifyChannel(node_animation->scale_type(), node_animation->scale()));
    MP_RETURN_IF_ERROR(VerifyChannel(node_animation->rotation_type(),
                                  node_animation->rotation()));
    MP_RETURN_IF_ERROR(VerifyChannel(node_animation->translation_type(),
                                  node_animation->translation()));
  }

  for (const animation::schemas::MorphTargetAnimation *morph_target_animation :
       *animation->morph_target_animations()) {
    MP_RETURN_IF_ERROR(VerifyChannel(morph_target_animation->weights_type(),
                                  morph_target_animation->weights()));
  }
  return NoError();
}

}  // namespace

OptionalError VerifyAnimations(const schemas::LoadedModel *loaded_model) {
  auto bone_count = loaded_model->skeleton()->child_counts()->size();
  for (const schemas::GltfAnimationInfo *anim_info :
       *loaded_model->animations()) {
    MP_RETURN_IF_ERROR(VerifyGltfAnimationInfo(anim_info, bone_count));
  }

  return NoError();
}

std::vector<absl::string_view> GetAnimationNames(
    const schemas::LoadedModel *loaded_model) {
  std::vector<absl::string_view> result;
  absl::c_transform(*loaded_model->animations(), std::back_inserter(result),
                    [](const schemas::GltfAnimationInfo *anim_info) {
                      const animation::schemas::GltfAnimation *animation =
                          anim_info->buffer_nested_root();
                      return animation->name()
                                 ? absl::string_view(
                                       reinterpret_cast<const char *>(
                                           animation->name()->Data()),
                                       animation->name()->size())
                                 : "";
                    });
  return result;
}

absl::StatusOr<std::unique_ptr<animation::GltfAnimation>>
CreateAnimationResources(const schemas::LoadedModel *loaded_model,
                         size_t animation_index) {
  if (loaded_model->animations()->size() <= animation_index) {
    return Error("Invalid Index");
  }

  const schemas::GltfAnimationInfo *anim_info =
      loaded_model->animations()->Get(animation_index);
  if (!anim_info) {
    return Error("Missing animation");
  }

  const animation::schemas::GltfAnimation *anim_fb =
      anim_info->buffer_nested_root();

  // Make a copy of the animation data (it's already in a packed buffer)
  auto *buffer = anim_info->buffer();
  if (!buffer || !buffer->size()) return Error("missing animation");
  auto anim_copy = FlatBufferAccess<
      animation::schemas::GltfAnimation>::CloneNestedFlatBuffer(anim_fb,
                                                                buffer->data(),
                                                                buffer->size());

  return animation::GltfAnimation::Create(std::move(anim_copy));
}

}  // namespace imp::loader::details
