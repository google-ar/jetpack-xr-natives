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

#include "core/recipes/language/recipe_system.h"

#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "core/assets/gltf/gltf_audio_extension.h"
#include "core/async/future.h"
#include "core/common/filament_helpers.h"
#include "core/common/invocable.h"
#include "core/common/registry.h"
#include "core/common/type_traits.h"
#include "core/math/almost_equal.h"
#include "core/math/mat.h"
#include "core/math/math.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/path_manager.h"
#include "core/proto/any.proto.imp.h"
#include "core/recipes/language/functions/math/math.h"
#include "core/recipes/language/recipe_custom_statement.h"
#include "core/recipes/language/recipe_scope.h"
#include "core/recipes/language/recipe_utils.h"
#include "core/recipes/language/registered_function.h"
#include "core/view/base_view.h"
#include "core/view/framework/animation/gltf_animator.h"
#include "core/view/framework/animation/gltf_animator_state.proto.imp.h"
#include "core/view/framework/assets/gltf_scene.h"
#include "core/view/framework/camera/camera_manager.h"
#include "core/view/framework/scene/scene_system.h"
#include "proposed/delayed_future_scheduler.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {

namespace {

template <typename T>
absl::Status CheckValidity(
    T value, std::optional<absl::string_view> error_message = std::nullopt) {
  if (!value) {
    if (error_message) {
      return absl::InvalidArgumentError(*error_message);
    } else {
      return absl::InvalidArgumentError(
          absl::StrFormat("%s is invalid.", type_traits::kTypeName<T>));
    }
  }
  return absl::OkStatus();
}

}  // namespace

// TOOD((broken link)): Consider creating constants for each System Recipe
// Function name.
RecipeSystem::RecipeSystem(BaseView& view) {
  RegisterFunction("ConsoleLog",
                   [](std::string output) { IMP_LOG(imp::INFO) << output; });

  // TODO: Define a proper set of standard Recipe functions.
  RegisterFunction("GetDeltaSeconds", [&view]() -> float {
    return view.GetFrameTime().GetDeltaSeconds();
  });

  RegisterFunction("QuatFromAxisAngle", [](float3 left, float right) -> quatf {
    return quatf::fromAxisAngle(left, right);
  });

  RegisterFunction("GetLocalRotation",
                   [](NodeHandle node) -> absl::StatusOr<quatf> {
                     MP_RETURN_IF_ERROR(CheckValidity(node));

                     return node->GetLocalRotation();
                   });

  RegisterFunction("SetLocalRotation",
                   [](NodeHandle node, quatf rotation) -> absl::Status {
                     MP_RETURN_IF_ERROR(CheckValidity(node));

                     node->SetLocalRotation(rotation);

                     return absl::OkStatus();
                   });

  RegisterFunction(
      "AddComponent",
      [](NodeHandle node, const google::protobuf::imp_proto::Any& component_any)
          -> Future<absl::Status> {
        absl::Status status = CheckValidity(node);
        if (!status.ok()) {
          return Future<absl::Status>(status);
        }

        return node->GetView().GetSceneSystem().AddComponent(node,
                                                             component_any);
      });

  RegisterFunction(
      "FindNodeByName",
      [](NodeHandle root_node, std::string name) -> absl::StatusOr<NodeHandle> {
        MP_RETURN_IF_ERROR(CheckValidity(root_node));

        return root_node->FindByName(name);
      });

  RegisterFunction(
      "IsAncestorOf",
      [](NodeHandle parent, NodeHandle child) -> absl::StatusOr<bool> {
        MP_RETURN_IF_ERROR(CheckValidity(parent, "parent NodeHandle is invalid."));

        MP_RETURN_IF_ERROR(CheckValidity(child, "child NodeHandle is invalid."));

        return parent->GetView().GetPathManager().IsAncestorOf(parent, child);
      });

  RegisterFunction("GetLocalPosition",
                   [](NodeHandle node) -> absl::StatusOr<float3> {
                     MP_RETURN_IF_ERROR(CheckValidity(node));

                     return node->GetLocalPosition();
                   });

  RegisterFunction("SetLocalPosition",
                   [](NodeHandle node, float3 position) -> absl::Status {
                     MP_RETURN_IF_ERROR(CheckValidity(node));

                     node->SetLocalPosition(position);
                     return absl::OkStatus();
                   });

  RegisterFunction("GetLocalRotationVec4",
                   [](NodeHandle node) -> absl::StatusOr<float4> {
                     MP_RETURN_IF_ERROR(CheckValidity(node));

                     return node->GetLocalRotation().xyzw;
                   });

  RegisterFunction("SetLocalRotationVec4",
                   [](NodeHandle node, float4 rotation) -> absl::Status {
                     MP_RETURN_IF_ERROR(CheckValidity(node));

                     node->SetLocalRotation(quatf(rotation));
                     return absl::OkStatus();
                   });

  RegisterFunction("GetLocalScale",
                   [](NodeHandle node) -> absl::StatusOr<float3> {
                     MP_RETURN_IF_ERROR(CheckValidity(node));

                     return node->GetLocalScale();
                   });

  RegisterFunction("SetLocalScale",
                   [](NodeHandle node, float3 scale) -> absl::Status {
                     MP_RETURN_IF_ERROR(CheckValidity(node));

                     node->SetLocalScale(scale);
                     return absl::OkStatus();
                   });

  RegisterFunction("GetLocalTransformMatrix",
                   [](NodeHandle node) -> absl::StatusOr<mat4f> {
                     MP_RETURN_IF_ERROR(CheckValidity(node));

                     return node->GetLocalTrs();
                   });

  RegisterFunction(
      "GetSceneTransformMatrix",
      [](NodeHandle node, NodeHandle root) -> absl::StatusOr<mat4f> {
        MP_RETURN_IF_ERROR(CheckValidity(node));

        ComponentHandle<GltfScene> gltf_scene = root->GetComponent<GltfScene>();
        if (!gltf_scene) {
          return absl::InternalError("No GltfScene found on the root node.");
        }

        if (node->GetView().IsPreciseTranslationEnabled()) {
          mat4 transform =
              node->GetView().GetPathManager().GetRelativeTransformPrecise(
                  gltf_scene->GetRoot(), node);

          return mat4f(transform);
        } else {
          return node->GetView().GetPathManager().GetRelativeTransform(
              gltf_scene->GetRoot(), node);
        }
      });

  RegisterFunction("EulerFromQuat", [](quatf q) { return EulerFromQuat(q); });
  RegisterFunction("QuatFromEuler",
                   [](float3 euler) { return QuatFromEuler(euler); });

  RegisterFunction("StartGltfAnimation",
                   [](NodeHandle node, int animation_index, float speed,
                      float start_time, float end_time) -> absl::Status {
                     MP_RETURN_IF_ERROR(CheckValidity(node));

                     ComponentHandle<GltfAnimator> gltf_animator =
                         node->GetOrAddComponent<GltfAnimator>();
                     MP_RETURN_IF_ERROR(CheckValidity(gltf_animator));

                     GltfAnimator::PlayCommand play_command;
                     play_command.options.speed_multiplier = speed;
                     play_command.options.start_time_seconds = start_time;
                     play_command.options.playback_channel =
                         GltfAnimator::PlaybackChannelId{.id = animation_index};
                     play_command.animation = animation_index;
                     return gltf_animator->PlaySafely(play_command);
                   });

  RegisterFunction("StopGltfAnimation",
                   [](NodeHandle node, int animation_index) -> absl::Status {
                     MP_RETURN_IF_ERROR(CheckValidity(node));
                     ComponentHandle<GltfAnimator> gltf_animator =
                         node->GetComponent<GltfAnimator>();
                     MP_RETURN_IF_ERROR(CheckValidity(gltf_animator));

                     gltf_animator->Stop(GltfAnimatorState::PlaybackChannelId{
                         .id = animation_index});

                     return absl::OkStatus();
                   });

  RegisterFunction("CastBoolToInt", [](bool input) { return input ? 1 : 0; });
  RegisterFunction("CastBoolToFloat",
                   [](bool input) { return input ? 1.0f : 0.0f; });
  RegisterFunction("CastIntToBool", [](int input) { return input != 0; });
  RegisterFunction("CastIntToFloat",
                   [](int input) { return static_cast<float>(input); });
  RegisterFunction("CastFloatToBool",
                   [](float input) { return !imp::AlmostEqual(input, 0.0f); });
  RegisterFunction("CastFloatToInt",
                   [](float input) { return static_cast<int>(input); });

  RegisterFunction("MakeVector2",
                   [](float x, float y) { return float2(x, y); });
  RegisterFunction("MakeVector3",
                   [](float x, float y, float z) { return float3(x, y, z); });
  RegisterFunction("MakeVector4", [](float x, float y, float z, float w) {
    return float4(x, y, z, w);
  });

  RegisterFunction("BreakVector2", [](float2 input) {
    recipe::Variables variables;
    variables["x"] = input.x;
    variables["y"] = input.y;
    return variables;
  });
  RegisterFunction("BreakVector3", [](float3 input) {
    recipe::Variables variables;
    variables["x"] = input.x;
    variables["y"] = input.y;
    variables["z"] = input.z;
    return variables;
  });
  RegisterFunction("BreakVector4", [](float4 input) {
    recipe::Variables variables;
    variables["x"] = input.x;
    variables["y"] = input.y;
    variables["z"] = input.z;
    variables["w"] = input.w;
    return variables;
  });

  RegisterFunction("Delay", [&view](float delay_seconds) {
    DelayedFutureScheduler& scheduler =
        view.GetRegistry().GetOrCreate<DelayedFutureScheduler>(&view);
    return scheduler.ScheduleDelayed<absl::Status>(
        absl::Seconds(delay_seconds), []() { return absl::OkStatus(); });
  });

  RegisterFunction("SetNodeEnabled", [](NodeHandle node, bool enabled) {
    node->SetEnabled(enabled);
  });

  RegisterFunction("IsNodeEnabled",
                   [](NodeHandle node) { return node->IsEnabled(); });

  RegisterFunction("ComposeTransform",
                   [](float3 translation, float4 rotation, float3 scale) {
                     quatf rotation_quat = quatf{rotation};
                     return imp::Compose(translation, rotation_quat, scale);
                   });

  RegisterFunction("DecomposeTransform", [](mat4f transform) {
    float3 translation;
    quatf rotation;
    float3 scale;

    recipe::Variables invalid;
    invalid["translation"] = kZero3;
    invalid["rotation"] = float4(0.f, 0.f, 0.f, 1.f);
    invalid["scale"] = kOne3;
    invalid["isValid"] = false;

    // See if the fourth column is invalid
    float4 last_column{transform[0].w, transform[1].w, transform[2].w,
                       transform[3].w};
    if (last_column != float4(0.f, 0.f, 0.f, 1.f)) {
      return invalid;
    }

    // Make sure the basis vectors are not infinite, NAN, or zero length
    float3 x_basis{transform[0].x, transform[0].y, transform[0].z};
    float3 y_basis{transform[1].x, transform[1].y, transform[1].z};
    float3 z_basis{transform[2].x, transform[2].y, transform[2].z};
    float x_basis_length = length(x_basis);
    float y_basis_length = length(y_basis);
    float z_basis_length = length(z_basis);
    if (std::isnan(x_basis_length) || std::isnan(y_basis_length) ||
        std::isnan(z_basis_length) || AlmostEqual(x_basis_length, 0.f) ||
        AlmostEqual(y_basis_length, 0.f) || AlmostEqual(z_basis_length, 0.f) ||
        x_basis_length == std::numeric_limits<float>::infinity() ||
        y_basis_length == std::numeric_limits<float>::infinity() ||
        z_basis_length == std::numeric_limits<float>::infinity()) {
      return invalid;
    }

    imp::Decompose(transform, &translation, &rotation, &scale);

    // Extract the unscaled rotation matrix so we can check the determinant
    mat3f rotation_matrix;
    rotation_matrix[0] = x_basis / scale.x;
    rotation_matrix[1] = y_basis / scale.y;
    rotation_matrix[2] = z_basis / scale.z;

    float determinant = det(rotation_matrix);
    if (!AlmostEqual(abs(determinant), 1.f)) {
      return invalid;
    }

    if (determinant < 0.f) {
      scale = -scale;
    }

    float4 rotation_float4 = {rotation.x, rotation.y, rotation.z, rotation.w};

    recipe::Variables variables;
    variables["translation"] = translation;
    variables["rotation"] = rotation_float4;
    variables["scale"] = scale;
    variables["isValid"] = true;
    return variables;
  });

  RegisterFunction(
      "PlayGltfAudio",
      [](NodeHandle gltf_root,
         int emitter_index) -> absl::StatusOr<recipe::ReturnValueDeclaration> {
        recipe::ReturnValueDeclaration return_values;
        MP_RETURN_IF_ERROR(CheckValidity(gltf_root));

        ComponentHandle<GltfAudioExtension> gltf_audio_extension =
            gltf_root->GetComponent<GltfAudioExtension>();

        MP_RETURN_IF_ERROR(CheckValidity(gltf_audio_extension));

        MP_ASSIGN_OR_RETURN(GltfAudioExtension::EmitterPlayerHandle handle,
                         gltf_audio_extension->PlayEmitter(emitter_index));

        return_values.socket_values["handle"] = static_cast<int>(handle);

        return return_values;
      });

  RegisterFunction(
      "StopGltfAudio",
      [](NodeHandle gltf_root, int emitter_player_handle) -> absl::Status {
        ComponentHandle<GltfAudioExtension> gltf_audio_extension =
            gltf_root->GetComponent<GltfAudioExtension>();

        MP_RETURN_IF_ERROR(CheckValidity(gltf_audio_extension));

        gltf_audio_extension->RemoveEmitterPlayer(emitter_player_handle);

        return absl::OkStatus();
      });

  RegisterFunction("GetCameraNode", [&view]() {
    return view.GetCameraManager().GetCamera()->GetNode();
  });

  recipe::RegisterMathAngleFunctions(this);
  recipe::RegisterMathArithmeticFunctions(this);
  recipe::RegisterMathBitWiseFunctions(this);
  recipe::RegisterMathConstants(this);
  recipe::RegisterMathExponentialFunctions(this);
  recipe::RegisterMathHyperbolicFunctions(this);
  recipe::RegisterMathMatrixFunctions(this);
  recipe::RegisterMathUtilityFunctions(this);
  recipe::RegisterMathVectorFunctions(this);
  recipe::RegisterMathQuaternionFunctions(this);
}

std::unique_ptr<RecipeCustomStatement> RecipeSystem::CreateCustomStatement(
    absl::string_view name) const {
  auto it = custom_statement_creators_.find(name);
  if (it == custom_statement_creators_.end()) {
    return {};
  }
  return it->second();
}

void RecipeSystem::RegisterFunctionImpl(
    std::unique_ptr<recipe::RegisteredFunction> function) {
  if (function == nullptr) {
    IMP_LOG(imp::ERROR) << "Recipe Function was null!";
    return;
  }

  registered_functions_[std::string(function->GetName().data(),
                                    function->GetName().length())] =
      std::move(function);
}

void RecipeSystem::RegisterCustomStatementTypeImpl(
    absl::string_view name,
    Invocable<std::unique_ptr<RecipeCustomStatement>()> creation_fn) {
  auto [_, inserted] = custom_statement_creators_.insert(
      {std::string(name.data(), name.length()), std::move(creation_fn)});

  if (!inserted) {
    IMP_LOG(imp::WARNING) << "Custom statement " << name
                 << " already registered. Overwriting.";
  }
}

}  // namespace imp
