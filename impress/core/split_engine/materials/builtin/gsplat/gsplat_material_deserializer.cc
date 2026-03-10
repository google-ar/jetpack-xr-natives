// Copyright 2025 Google LLC
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

#include "core/split_engine/materials/builtin/gsplat/gsplat_material_deserializer.h"

#include <cstdint>
#include <functional>
#include <optional>
#include <utility>

#include "core/common/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "filament/filament/include/filament/Options.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "flatbuffers/verifier.h"
#include "core/assets/asset_ptr.h"
#include "core/assets/material/material_asset.h"
#include "core/assets/material/material_load_options.proto.imp.h"
#include "core/async/future.h"
#include "core/common/registry.h"
#include "core/common/small_source_location.h"
#include "core/material_library/flatbuffer_utils.h"
#include "core/material_library/material_package.h"
#include "core/materials/material.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/render/texture.h"
#include "core/resources/resource_definition.h"
#include "core/split_engine/flatbuffer_utils.h"
#include "core/split_engine/materials/builtin/builtin_custom_material.h"
#include "core/split_engine/materials/builtin/builtin_material.h"
#include "core/split_engine/materials/builtin/builtin_material_registry.h"
#include "core/split_engine/materials/builtin/gsplat/gsplat_material_deserializer_assets.h"
#include "core/split_engine/materials/builtin/gsplat/precompute_texture_pipeline.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/split_engine_renderer.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/material_factory.h"
#include "core/window/filament_host.h"
#include "split_engine/schemas/split_engine_material_generated.h"
#include "split_engine/schemas/split_engine_primitive_generated.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::split_engine {
namespace {
// LINT.IfChange
constexpr char kOpacityScaleParameter[] = "opacityScale";
constexpr char kMinScreenSizeParameter[] = "minScreenSize";
constexpr char kMaxScreenSizeParameter[] = "maxScreenSize";
constexpr char kWindowDimensionInMagicWindowParameter[] =
    "windowDimensionInMagicWindow";
constexpr char kMagicWindowFromUserWorldMatrixParameter[] =
    "magicWindowFromUserWorldMatrix";
constexpr char kPositionDataTexture[] = "splatDataPosition";
constexpr char kCov3dDataTexture[] = "splatDataCov3d";
constexpr char kColorDataTexture[] = "splatDataColor";
constexpr char kSortedIndicesTexture[] = "sortedIndices";
constexpr char kSplatScaleParameter[] = "splatScale";
constexpr char kUseTrianglesForSplatsParameter[] = "useTrianglesForSplats";
constexpr char kSplatDataPrecomputed[] = "splatDataPrecomputed";
constexpr char kMainViewResolution[] = "mainViewResolution";
// LINT.ThenChange(
//   builtin_gsplat.mat,
//   builtin_gsplat_data_precompute.mat,
//   builtin_magic_window.mat,
//   //depot/google3/third_party/split_engine/schemas/split_engine_material.fbs,
// )

// Helper to get the size of the data textures.
absl::StatusOr<imp::uint2> GetTextureSizeFromFlatbuffer(
    const TextureBorrower& texture_borrower,
    const android_xr::schemas::BuiltInMaterialGsplatParameters&
        serialized_parameters) {
  const android_xr::schemas::BuiltInTextureParameter* serialized_texture =
      serialized_parameters.position_data_texture();
  if (!serialized_texture) {
    return absl::NotFoundError(
        absl::StrCat("Texture not set yet for ", kPositionDataTexture));
  }
  const BorrowedTexturePtr texture = texture_borrower(
      serialized_texture->texture_id(), SmallSourceLocation::Current());
  if (!texture) {
    return absl::InternalError(absl::StrFormat("Texture not found: %d, %s",
                                               serialized_texture->texture_id(),
                                               kPositionDataTexture));
  }
  return texture->GetSize();
}

// Helper function to read texture from flatbuffer and set it on a material.
absl::Status SetMaterialParameterFromFlatbuffer(
    const TextureBorrower& texture_borrower,
    const BorrowedMaterialPtr& material,
    const android_xr::schemas::BuiltInTextureParameter* serialized_texture,
    absl::string_view material_parameter_name) {
  if (!serialized_texture) {
    return absl::NotFoundError(
        absl::StrCat("Texture not set yet for ", material_parameter_name));
  }
  if (!material->HasParameter(material_parameter_name)) {
    return absl::OkStatus();
  }

  const BorrowedTexturePtr texture = texture_borrower(
      serialized_texture->texture_id(), SmallSourceLocation::Current());
  if (!texture) {
    return absl::InternalError(absl::StrFormat("Texture not found: %d, %s",
                                               serialized_texture->texture_id(),
                                               material_parameter_name));
  }
  material->SetParameter(material_parameter_name, texture,
                         ConvertSampler(serialized_texture->sampler()));
  return absl::OkStatus();
}

resources::ResourceDefinition GetResourceDefinition(
    BaseView& view, android_xr::schemas::GsplatMode material_mode) {
  // In multiview mode, materials must accept texture arrays.
  bool is_multiview =
      view.GetHost()->GetEngine()->getConfig().stereoscopicType ==
      filament::backend::StereoscopicType::MULTIVIEW;

  switch (material_mode) {
    case android_xr::schemas::GsplatMode::UNSPECIFIED:
    case android_xr::schemas::GsplatMode::GSPLAT:
      return is_multiview ? kBuiltinGsplatStereoMatCmat
                          : kBuiltinGsplatMonoMatCmat;
    case android_xr::schemas::GsplatMode::MAGIC_WINDOW:
      return is_multiview ? kBuiltinMagicWindowStereoMatCmat
                          : kBuiltinMagicWindowMonoMatCmat;
  }
}

absl::Status HasParameter(const BorrowedMaterialPtr& material,
                          absl::string_view parameter_name) {
  if (material->HasParameter(parameter_name)) {
    return absl::OkStatus();
  }
  return absl::UnimplementedError(
      absl::StrFormat("%s is not supported in this material.", parameter_name));
}

// Extracts main view resolution and applies it as material setting.
absl::Status UpdateMainViewResolution(BaseView& view,
                                      BorrowedMaterialPtr precompute_material) {
  window::FilamentHost& host = *view.GetHost();
  const filament::DynamicResolutionOptions drs_options =
      host.GetView()->getDynamicResolutionOptions();
  const imp::float2 scale =
      drs_options.enabled ? drs_options.maxScale : imp::kOne2;
  precompute_material->SetParameter(
      kMainViewResolution, imp::uint2(host.GetPixelDimensions() * scale));
  return absl::OkStatus();
}

// Utility function convert a node id to a node handle.
absl::StatusOr<NodeHandle> DeserializeNodeId(BaseView& view,
                                             uint32_t gsplat_node_id) {
  absl::StatusOr<NodeHandle> gsplat_node;
  if (view.GetSplitEngineSerializer() == nullptr) {
    // If not in split engine, use the node id directly.
    gsplat_node = NodeHandle(utils::Entity::import(gsplat_node_id));
  } else {
    auto renderer =
        view.GetRegistry().Get<imp::split_engine::SplitEngineRenderer>();
    if (!renderer.ok()) {
      return absl::InvalidArgumentError(
          absl::StrCat("Invalid split engine renderer: ", renderer.status()));
    }

    gsplat_node = renderer->get().GetNodeForCurrentApp(gsplat_node_id);
  }
  return gsplat_node;
}

Future<ComponentHandle<PrecomputeTexturePipeline>>
BuildPrecomputeTexturePipeline(NodeHandle gsplat_node,
                               android_xr::schemas::GsplatMode material_mode) {
  NodeHandle precompute_node = gsplat_node->CreateChildNode();
  precompute_node->SetName("BuiltinGSplatPrecomputeTPR");
  // precompute_node is created disabled and will be enabled after the pipeline
  // is created.
  precompute_node->SetEnabled(false);

  return precompute_node
      ->AddComponent<PrecomputeTexturePipeline>(
          kBuiltinGsplatDataPrecomputeMatCmat)
      .Then([](ComponentHandle<PrecomputeTexturePipeline> pipeline) mutable
                -> absl::StatusOr<ComponentHandle<PrecomputeTexturePipeline>> {
        MP_RETURN_IF_ERROR(UpdateMainViewResolution(pipeline->GetView(),
                                                 pipeline->BorrowMaterial()));
        return pipeline;
      });
}

}  // namespace

Future<BuiltInMaterialPtr> GsplatMaterialDeserializer::Create(
    BaseView& view, BridgeId bridge_id,
    const android_xr::schemas::BuiltInMaterialGsplatSpec& spec) {
  android_xr::schemas::GsplatMode material_mode = spec.material_mode();

  absl::StatusOr<NodeHandle> gsplat_node =
      DeserializeNodeId(view, spec.gsplat_node_id());
  // A valid gsplat renderer is required to correctly provide and update the
  // transforms for the filament material.
  if (!gsplat_node.ok() || !gsplat_node->IsValid()) {
    IMP_LOG(imp::WARNING) << "Invalid gsplat renderer entity ";
    // TODO: Refactor to support default construction of
    // GsplatMaterialDeserializer, without a valid gsplat node.
    gsplat_node = view.CreateNode();
  }
  NodeHandle gsplat_node_value = gsplat_node.value();
  ::imp::resources::ResourceDefinition source =
      GetResourceDefinition(view, material_mode);
  bool use_triangles = false;
  if (spec.use_triangles_for_splats()) {
    use_triangles = spec.use_triangles_for_splats()->value();
  }
  bool has_precomputed_texture = false;
  if (spec.has_precomputed_data_texture()) {
    has_precomputed_texture = spec.has_precomputed_data_texture()->value();
  }

  return view.GetAssetManager()
      .LoadMaterial(source,
                    MaterialPreCompileOptions{
                        .constants = {{.name = kUseTrianglesForSplatsParameter,
                                       .value = use_triangles}}})
      .Then([bridge_id, material_mode, gsplat_node_value,
             has_precomputed_texture](
                AssetPtr<MaterialAsset> material_asset) mutable
                -> Future<BuiltInMaterialPtr> {
        // When the spec provides precomputed texture, we create the builtin
        // material without a PrecomputeTexturePipeline. Otherwise, the pipeline
        // will be created and used by the builtin material.
        // TODO: Temporarily disable precompute with magic window.
        if (has_precomputed_texture ||
            material_mode == android_xr::schemas::GsplatMode::MAGIC_WINDOW) {
          return Future<BuiltInMaterialPtr>(Create(
              gsplat_node_value, bridge_id, material_mode, material_asset,
              ComponentHandle<PrecomputeTexturePipeline>()));
        }
        // TODO: Pass in aabb of the gsplat scene so that the precompute pass
        // doesn't run if the scene is frustum culled.
        return BuildPrecomputeTexturePipeline(gsplat_node_value, material_mode)
            .Then(
                [gsplat_node_value, bridge_id, material_mode, material_asset](
                    ComponentHandle<PrecomputeTexturePipeline> pipeline) mutable
                    -> BuiltInMaterialPtr {
                  return Create(gsplat_node_value, bridge_id, material_mode,
                                material_asset, pipeline);
                });
      });
}

BuiltInMaterialPtr GsplatMaterialDeserializer::Create(
    NodeHandle gsplat_node, BridgeId bridge_id,
    android_xr::schemas::GsplatMode material_mode,
    AssetPtr<MaterialAsset> material_asset,
    ComponentHandle<PrecomputeTexturePipeline> pipeline) {
  return BuiltInMaterialPtr(new GsplatMaterialDeserializer(
      gsplat_node, bridge_id, material_mode,
      gsplat_node->GetView().GetMaterialFactory().CreateMaterial(
          material_asset),
      pipeline));
}

GsplatMaterialDeserializer::GsplatMaterialDeserializer(
    NodeHandle gsplat_node, BridgeId bridge_id,
    android_xr::schemas::GsplatMode material_mode, OwnedMaterialPtr material,
    ComponentHandle<PrecomputeTexturePipeline> precompute_texture_pipeline)
    : BuiltInCustomMaterial(bridge_id, std::move(material)),
      view_(gsplat_node->GetView()),
      gsplat_node_(gsplat_node),
      material_mode_(material_mode) {
  precompute_texture_pipeline_ = precompute_texture_pipeline;
}

GsplatMaterialDeserializer::~GsplatMaterialDeserializer() {
  if (precompute_texture_pipeline_) {
    // Release the pass texture before destroying the pass.
    BorrowedTexturePtr placeholder_texture =
        view_.GetTextureFactory().BorrowRGBA32FPlaceholderTexture();
    GetRenderMaterial()->SetParameter(kSplatDataPrecomputed,
                                      placeholder_texture,
                                      placeholder_texture->GetSampler());
    // A node was added for the precompute texture pipeline.
    // If it hasn't already been destroyed, destroy it now.
    view_.DestroyNode(precompute_texture_pipeline_->GetNode());
  }
}

absl::Status GsplatMaterialDeserializer::SetParameters(
    flatbuffers::Verifier& verifier,
    const android_xr::schemas::BuiltInMaterialInstanceParameters& parameters,
    const TextureBorrower& texture_borrower) {
  if (parameters.data_type() != android_xr::schemas::BuiltInMaterialParameters::
                                    BuiltInMaterialGsplatParameters) {
    return absl::InvalidArgumentError(
        "This material requires BuiltInMaterialGsplatParameters");
  }

  if (!VerifyBuiltInMaterialParameters(
          verifier, parameters.data(),
          android_xr::schemas::BuiltInMaterialParameters::
              BuiltInMaterialGsplatParameters)) {
    return absl::InvalidArgumentError("Invalid parameters");
  }

  const android_xr::schemas::BuiltInMaterialGsplatParameters*
      serialized_parameters =
          parameters
              .data_as<android_xr::schemas::BuiltInMaterialGsplatParameters>();

  MP_RETURN_IF_ERROR(
      SetRenderMaterialParameters(texture_borrower, *serialized_parameters));

  if (material_mode_ == android_xr::schemas::GsplatMode::MAGIC_WINDOW) {
    MP_RETURN_IF_ERROR(SetMagicWindowMaterialParameters(texture_borrower,
                                                     *serialized_parameters));
  }

  // TODO: (broken link) - Move this into if (precompute_texture_pipeline_)
  // once fixed, and magic window material no longer relies on these parameters.
  MP_RETURN_IF_ERROR(SetPrecomputedDataParameters(
      texture_borrower, *serialized_parameters, GetPrecomputedDataMaterial()));

  if (precompute_texture_pipeline_) {
    MP_RETURN_IF_ERROR(UpdatePrecomputeTexturePipeline(texture_borrower,
                                                    *serialized_parameters));
  } else if (serialized_parameters->precomputed_data_texture()) {
    MP_RETURN_IF_ERROR(SetMaterialParameterFromFlatbuffer(
        texture_borrower, GetRenderMaterial(),
        serialized_parameters->precomputed_data_texture(),
        kSplatDataPrecomputed));
  }

  return absl::OkStatus();
}

split_engine::BuiltInMaterialPtr GsplatMaterialDeserializer::Duplicate() const {
  ComponentHandle<PrecomputeTexturePipeline> pipeline;
  if (precompute_texture_pipeline_) {
    Future<ComponentHandle<PrecomputeTexturePipeline>> pipeline_future =
        BuildPrecomputeTexturePipeline(gsplat_node_, material_mode_);
    if (pipeline_future.Ready() && pipeline_future.Get().ok()) {
      pipeline = pipeline_future.Get().value();
    } else {
      pipeline_future.Cancel();
      // Does not exit if NDEBUG is defined.
      IMP_LOG(imp::FATAL) << "Failed to duplicate precompute texture pipeline.";
    }
  }

  OwnedMaterialPtr material = view_.GetMaterialFactory().WrapMaterial(
      filament::MaterialInstance::duplicate(
          GetMaterial()->GetFilamentMaterialInstance()));
  split_engine::BuiltInMaterialPtr result =
      absl::WrapUnique(new GsplatMaterialDeserializer(
          gsplat_node_, GetBridgeId(), material_mode_, std::move(material),
          pipeline));
  return result;
}

absl::Status GsplatMaterialDeserializer::SetRenderMaterialParameters(
    const TextureBorrower& texture_borrower,
    const android_xr::schemas::BuiltInMaterialGsplatParameters&
        serialized_parameters) {
  BorrowedMaterialPtr render_material = GetRenderMaterial();

  if (const android_xr::schemas::Float* splat_scale =
          serialized_parameters.splat_scale()) {
    MP_RETURN_IF_ERROR(HasParameter(render_material, kSplatScaleParameter));
    render_material->SetParameter(kSplatScaleParameter, UnPack(*splat_scale));
  }

  if (const android_xr::schemas::BuiltInTextureParameter*
          sorted_indices_texture =
              serialized_parameters.sorted_indices_texture()) {
    MP_RETURN_IF_ERROR(HasParameter(render_material, kSortedIndicesTexture));
    MP_RETURN_IF_ERROR(SetMaterialParameterFromFlatbuffer(
        texture_borrower, render_material, sorted_indices_texture,
        kSortedIndicesTexture));
  }
  return absl::OkStatus();
}

absl::Status GsplatMaterialDeserializer::SetMagicWindowMaterialParameters(
    const TextureBorrower& texture_borrower,
    const android_xr::schemas::BuiltInMaterialGsplatParameters&
        serialized_parameters) {
  BorrowedMaterialPtr magic_window_material = GetRenderMaterial();
  if (const android_xr::schemas::Float2* window_dimension_in_magic_window =
          serialized_parameters.window_dimension_in_magic_window()) {
    MP_RETURN_IF_ERROR(HasParameter(magic_window_material,
                                 kWindowDimensionInMagicWindowParameter));
    magic_window_material->SetParameter(
        kWindowDimensionInMagicWindowParameter,
        UnPack(*window_dimension_in_magic_window));
  }
  if (const android_xr::schemas::Mat4f* magic_window_from_user_world_matrix =
          serialized_parameters.magic_window_from_user_world_matrix()) {
    MP_RETURN_IF_ERROR(HasParameter(magic_window_material,
                                 kMagicWindowFromUserWorldMatrixParameter));
    magic_window_material->SetParameter(
        kMagicWindowFromUserWorldMatrixParameter,
        UnPack(*magic_window_from_user_world_matrix));
  }

  return absl::OkStatus();
}

absl::Status GsplatMaterialDeserializer::SetPrecomputedDataParameters(
    const TextureBorrower& texture_borrower,
    const android_xr::schemas::BuiltInMaterialGsplatParameters&
        serialized_parameters,
    BorrowedMaterialPtr precompute_material) {
  MP_RETURN_IF_ERROR(SetMaterialParameterFromFlatbuffer(
      texture_borrower, precompute_material,
      serialized_parameters.position_data_texture(), kPositionDataTexture));
  MP_RETURN_IF_ERROR(SetMaterialParameterFromFlatbuffer(
      texture_borrower, precompute_material,
      serialized_parameters.cov3d_data_texture(), kCov3dDataTexture));
  MP_RETURN_IF_ERROR(SetMaterialParameterFromFlatbuffer(
      texture_borrower, precompute_material,
      serialized_parameters.color_data_texture(), kColorDataTexture));

  const android_xr::schemas::Float2* min_screen_size =
      serialized_parameters.min_screen_size();
  if (min_screen_size &&
      precompute_material->HasParameter(kMinScreenSizeParameter)) {
    precompute_material->SetParameter(kMinScreenSizeParameter,
                                      UnPack(*min_screen_size));
  }

  const android_xr::schemas::Float2* max_screen_size =
      serialized_parameters.max_screen_size();
  if (max_screen_size &&
      precompute_material->HasParameter(kMaxScreenSizeParameter)) {
    precompute_material->SetParameter(kMaxScreenSizeParameter,
                                      UnPack(*max_screen_size));
  }

  const android_xr::schemas::Float* opacity_scale =
      serialized_parameters.opacity_scale();
  if (opacity_scale &&
      precompute_material->HasParameter(kOpacityScaleParameter)) {
    precompute_material->SetParameter(kOpacityScaleParameter,
                                      UnPack(*opacity_scale));
  }

  return absl::OkStatus();
}

absl::Status GsplatMaterialDeserializer::UpdatePrecomputeTexturePipeline(
    const TextureBorrower& texture_borrower,
    const android_xr::schemas::BuiltInMaterialGsplatParameters&
        serialized_parameters) {
  if (!precompute_texture_pipeline_) {
    return absl::OkStatus();
  }
  // Resize the pass texture based on the size provided in data textures.
  MP_ASSIGN_OR_RETURN(
      imp::uint2 position_data_texture_size,
      GetTextureSizeFromFlatbuffer(texture_borrower, serialized_parameters));
  MP_RETURN_IF_ERROR(precompute_texture_pipeline_->ResizePassTexture(
      0, position_data_texture_size));

  // Set the main view resolution
  MP_RETURN_IF_ERROR(UpdateMainViewResolution(
      view_, precompute_texture_pipeline_->BorrowMaterial()));

  if (!precompute_texture_pipeline_->IsRunningAsyncSetup() &&
      !precompute_texture_pipeline_->GetNode()->IsEnabled()) {
    precompute_texture_pipeline_->GetNode()->SetEnabled(true);
  }
  // Set the precompute texture on the render material.
  BorrowedTexturePtr precompute_texture =
      precompute_texture_pipeline_->BorrowTexture();
  if (!precompute_texture) {
    return absl::InternalError("Failed to borrow precompute texture");
  }
  GetRenderMaterial()->SetParameter(kSplatDataPrecomputed, precompute_texture,
                                    precompute_texture->GetSampler());
  return absl::OkStatus();
}

BorrowedMaterialPtr GsplatMaterialDeserializer::GetPrecomputedDataMaterial(
    SmallSourceLocation loc) const {
  if (precompute_texture_pipeline_) {
    return precompute_texture_pipeline_->BorrowMaterial(loc);
  }
  return GetMaterial(loc);
}

BorrowedMaterialPtr GsplatMaterialDeserializer::GetRenderMaterial(
    SmallSourceLocation loc) const {
  return GetMaterial(loc);
}

// Registers the built-in material factory.
const bool kRegisterMaterial = BuiltinMaterialRegistry::RegisterOrDie(
    android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterialGsplatSpec,
    [](BaseView& view, BridgeId bridge_id,
       const android_xr::schemas::BuiltInMaterialRequest& request,
       std::optional<
           std::reference_wrapper<const MaterialPackage::MaterialCache>>
           cache) -> Future<BuiltInMaterialPtr> {
      const android_xr::schemas::BuiltInMaterialGsplatSpec* spec =
          request.data_as_BuiltInMaterialGsplatSpec();
      if (spec == nullptr) {
        return Future<BuiltInMaterialPtr>(absl::InvalidArgumentError(
            "Failed to get the BuiltInMaterialGsplatSpec from the request."));
      }
      return GsplatMaterialDeserializer::Create(view, bridge_id, *spec);
    });

}  // namespace imp::split_engine
