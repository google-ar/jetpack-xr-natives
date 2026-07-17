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
#include <string>
#include <tuple>
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
#include "core/gsplat/gsplat_material_params.h"
#include "core/material_library/flatbuffer_utils.h"
#include "core/material_library/material_package.h"
#include "core/materials/material.h"
#include "core/math/quat.h"
#include "core/math/transform.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/render/texture.h"
#include "core/render_passes/group_to_projection_quad_texture_renderer/group_to_projection_quad_texture_renderer.h"
#include "core/render_passes/group_to_projection_quad_texture_renderer/group_to_projection_quad_texture_renderer_state.proto.imp.h"
#include "core/render_passes/texture_pipeline_renderer_helper.h"
#include "core/render_passes/texture_pipeline_renderer_projection_quad.h"
#include "core/resources/resource_definition.h"
#include "core/split_engine/flatbuffer_utils.h"
#include "core/split_engine/materials/builtin/builtin_custom_material.h"
#include "core/split_engine/materials/builtin/builtin_material.h"
#include "core/split_engine/materials/builtin/builtin_material_registry.h"
#include "core/split_engine/materials/builtin/gsplat/gsplat_material_helpers.h"
#include "core/split_engine/materials/builtin/gsplat/precompute_material_assets.h"
#include "core/split_engine/materials/builtin/gsplat/precompute_texture_pipeline.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/split_engine_renderer.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/material_factory.h"
#include "core/view/view_events.h"
#include "core/window/filament_host.h"
#include "split_engine/schemas/split_engine_material_generated.h"
#include "split_engine/schemas/split_engine_primitive_generated.h"
#include "split_engine/schemas/split_engine_render_passes_generated.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::split_engine {
namespace {

constexpr char kMagicWindowTextureNamePrefix[] = "magic_window_texture_";

// TODO: Remove this wait once the serializer can guarantee the
// node is going to be created before this material.
constexpr int kMaxFramesToWaitForGsplatNode = 30;

// Helper to get the size of the data textures.
absl::StatusOr<imp::uint2> GetTextureSizeFromFlatbuffer(
    const TextureBorrower& texture_borrower,
    const android_xr::schemas::GsplatParameters& serialized_parameters) {
  const android_xr::schemas::BuiltInTextureParameter* serialized_texture =
      serialized_parameters.position_data_texture();
  if (!serialized_texture) {
    return absl::NotFoundError(absl::StrCat("Texture not set yet for ",
                                            kPositionDataTextureParameter));
  }
  const BorrowedTexturePtr texture = texture_borrower(
      serialized_texture->texture_id(), SmallSourceLocation::Current());
  if (!texture) {
    return absl::InternalError(absl::StrFormat("Texture not found: %d, %s",
                                               serialized_texture->texture_id(),
                                               kPositionDataTextureParameter));
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

absl::Status MakeParameterNotImplementedError(
    absl::string_view parameter_name) {
  return absl::UnimplementedError(
      absl::StrFormat("Parameter %s not found", parameter_name));
}

// Updates the precompute material target view resolution to either the main
// view resolution, or the given offscreen texture resolution.
absl::Status UpdatePrecomputeViewResolution(
    BaseView& view, std::optional<imp::uint2> offscreen_texture_resolution,
    BorrowedMaterialPtr precompute_material) {
  if (offscreen_texture_resolution.has_value()) {
    precompute_material->SetParameter(kMainViewResolutionParameter,
                                      *offscreen_texture_resolution);
  } else {
    window::FilamentHost& host = *view.GetHost();
    const filament::DynamicResolutionOptions drs_options =
        host.GetView()->getDynamicResolutionOptions();
    const imp::float2 scale =
        drs_options.enabled ? drs_options.maxScale : imp::kOne2;

    precompute_material->SetParameter(
        kMainViewResolutionParameter,
        imp::uint2(host.GetPixelDimensions() * scale));
  }

  return absl::OkStatus();
}

// Utility function convert a node id to a node handle.
absl::StatusOr<NodeHandle> DeserializeNodeId(BaseView& view,
                                             uint32_t gsplat_node_id) {
  auto renderer = view.GetRegistry().Get<SplitEngineRenderer>();
  if (renderer.ok()) {
    // Using Split Engine: return the system side node ID based on the given
    // client side node ID.
    MP_ASSIGN_OR_RETURN(
        auto node_handle, renderer->get().GetNodeForCurrentApp(gsplat_node_id),
        _ << "Failed to deserialize client side node id: " << gsplat_node_id);
    return node_handle;
  } else {
    // Not using Split Engine: use the node id directly.
    return NodeHandle(utils::Entity::import(gsplat_node_id));
  }
}

// Builds a precompute texture pipeline for a gsplat scene.
// The pipeline will be created assuming it is using the main view resolution.
// If the Gsplat is instead rendered to an offscreen texture, it is expected
// that the correct offscreen resolution will be updated later through
// BuiltInMaterialGsplatParameters::magic_window_offscreen_resolution or
// view_resolution.
Future<ComponentHandle<PrecomputeTexturePipeline>>
BuildPrecomputeTexturePipeline(NodeHandle gsplat_node) {
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
        MP_RETURN_IF_ERROR(UpdatePrecomputeViewResolution(
            pipeline->GetView(), /*offscreen_texture_resolution=*/std::nullopt,
            pipeline->BorrowMaterial()));
        return pipeline;
      });
}

// Returns a projection quad in the World Space from the given
// `magic_window_projection_quad` flatbuffer, and the `node` it is associated
// with.
imp::TexturePipelineRendererProjectionQuad GetWorldProjectionQuad(
    const android_xr::schemas::ProjectionQuad& magic_window_projection_quad,
    NodeHandle node) {
  const float2 local_size = UnPack(*magic_window_projection_quad.size());
  const float3 local_center = UnPack(*magic_window_projection_quad.center());
  const quatf local_rotation = UnPack(*magic_window_projection_quad.rotation());

  const Transform<float> world_from_local(node->GetWorldTrs());
  return imp::TexturePipelineRendererProjectionQuad{
      .size = local_size * world_from_local.scale.xy,
      .center =
          world_from_local.rotation * (world_from_local.scale * local_center) +
          world_from_local.translation,
      .rotation = world_from_local.rotation * local_rotation,
  };
}

// Utility method to return a future waiting up to N frames for the gsplat node
// to be available on the system side. This can occur when the request to create
// the GsplatMaterialDeserializer is received before the Split Engine message to
// create the gsplat node it depends upon.
Future<NodeHandle> GetOrWaitForGsplatNode(BaseView& view,
                                          uint32_t gsplat_node_id,
                                          int frames_to_wait) {
  if (absl::StatusOr<NodeHandle> node = DeserializeNodeId(view, gsplat_node_id);
      node.ok() && node->IsValid()) {
    return Future<NodeHandle>(node.value());
  }

  Future<NodeHandle> wait_for_node_future;
  view.GetDispatcher().Connect(
      [wait_for_node_future, &view, gsplat_node_id,
       remaining_frames = frames_to_wait,
       original_frames_to_wait =
           frames_to_wait](const ViewPostFrameUpdateEvent& event) mutable {
        if (absl::StatusOr<NodeHandle> node =
                DeserializeNodeId(view, gsplat_node_id);
            node.ok() && node->IsValid()) {
          event.Disconnect();
          wait_for_node_future.Return(node.value());
        } else if (--remaining_frames <= 0) {
          event.Disconnect();
          wait_for_node_future.Return(absl::FailedPreconditionError(
              absl::StrFormat("Failed to get gsplat node after %d frames",
                              original_frames_to_wait)));
        }
      },
      &view);

  return wait_for_node_future;
}

}  // namespace

Future<BuiltInMaterialPtr> GsplatMaterialDeserializer::Create(
    BaseView& view, BridgeId bridge_id,
    const android_xr::schemas::BuiltInMaterialGsplatSpec& spec) {
  android_xr::schemas::GsplatMode material_mode = spec.material_mode();

  Future<NodeHandle> gsplat_node_future = GetOrWaitForGsplatNode(
      view, spec.entity(), kMaxFramesToWaitForGsplatNode);
  ::imp::resources::ResourceDefinition source =
      GsplatDefaultRenderResource(view, material_mode);

  if (material_mode == android_xr::schemas::GsplatMode::MAGIC_WINDOW) {
    // MAGIC_WINDOW mode.

    const android_xr::schemas::MagicWindowSpec* magic_window_spec = nullptr;
    if (spec.mode_spec_type() ==
        android_xr::schemas::GsplatModeSpec::MagicWindowSpec) {
      magic_window_spec = spec.mode_spec_as_MagicWindowSpec();
    }

    if (magic_window_spec == nullptr) {
      return absl::InternalError(
          "failed to create GsplatMaterialDeserializer; missing "
          "MagicWindowSpec for MAGIC_WINDOW mode!");
    }

    if (magic_window_spec->magic_window_offscreen_resolution() == nullptr) {
      return absl::InternalError(
          "failed to create GsplatMaterialDeserializer; "
          "magic_window_offscreen_resolution was not specified!");
    }

    if (magic_window_spec->render_group() == nullptr) {
      return absl::InternalError(
          "failed to create GsplatMaterialDeserializer; render_group was not "
          "specified!");
    }

    const imp::uint2 offscreen_texture_resolution =
        UnPack(*magic_window_spec->magic_window_offscreen_resolution());
    const std::string render_group_name =
        magic_window_spec->render_group()->str();

    Future<AssetPtr<MaterialAsset>> material_asset_future =
        view.GetAssetManager().LoadMaterial(source);

    return gsplat_node_future.Merge(material_asset_future)
        .Then([bridge_id, offscreen_texture_resolution, render_group_name](
                  std::tuple<NodeHandle, AssetPtr<MaterialAsset>> result)
                  -> Future<BuiltInMaterialPtr> {
          NodeHandle gsplat_node = std::get<0>(result);
          AssetPtr<MaterialAsset> material_asset = std::get<1>(result);
          return gsplat_node
              ->AddComponentWithState<GroupToProjectionQuadTextureRenderer>(
                  GroupToProjectionQuadTextureRendererState{
                      .texture_name =
                          absl::StrCat(kMagicWindowTextureNamePrefix,
                                       gsplat_node.GetEntity().getId()),
                      .render_group_name = render_group_name,
                      .texture_size = offscreen_texture_resolution,
                  })
              .Then([gsplat_node, bridge_id, material_asset,
                     offscreen_texture_resolution](
                        ComponentHandle<GroupToProjectionQuadTextureRenderer>
                            texture_renderer) -> BuiltInMaterialPtr {
                return Create(gsplat_node, bridge_id,
                              android_xr::schemas::GsplatMode::MAGIC_WINDOW,
                              material_asset, offscreen_texture_resolution,
                              ComponentHandle<PrecomputeTexturePipeline>(),
                              texture_renderer);
              });
        });

  } else {
    // GSPLAT mode.

    bool use_triangles = false;
    if (spec.use_triangles_for_splats()) {
      use_triangles = spec.use_triangles_for_splats()->value();
    }
    bool has_precomputed_texture = false;
    if (spec.has_precomputed_data_texture()) {
      has_precomputed_texture = spec.has_precomputed_data_texture()->value();
    }

    // Check if the optional render group is given.
    std::optional<std::string> render_group;
    if (spec.mode_spec_type() ==
        android_xr::schemas::GsplatModeSpec::GsplatSpec) {
      const android_xr::schemas::GsplatSpec* gsplat_spec =
          spec.mode_spec_as_GsplatSpec();
      if (gsplat_spec != nullptr && gsplat_spec->render_group() != nullptr &&
          !gsplat_spec->render_group()->empty()) {
        render_group = gsplat_spec->render_group()->str();
      }
    }

    Future<AssetPtr<MaterialAsset>> material_asset_future =
        view.GetAssetManager().LoadMaterial(
            source, MaterialPreCompileOptions{
                        .constants = {{.name = std::string(
                                           kUseTrianglesForSplatsConstant),
                                       .value = use_triangles}}});

    return gsplat_node_future.Merge(material_asset_future)
        .Then([bridge_id, material_mode, has_precomputed_texture, render_group](
                  std::tuple<NodeHandle, AssetPtr<MaterialAsset>> result)
                  -> Future<BuiltInMaterialPtr> {
          NodeHandle gsplat_node = std::get<0>(result);
          AssetPtr<MaterialAsset> material_asset = std::get<1>(result);
          if (render_group.has_value()) {
            gsplat_node->SetGroups({*render_group});
          }
          // When the spec provides precomputed texture, we create the builtin
          // material without a PrecomputeTexturePipeline. Otherwise, the
          // pipeline will be created and used by the builtin material.
          if (has_precomputed_texture) {
            return Future<BuiltInMaterialPtr>(Create(
                gsplat_node, bridge_id, material_mode, material_asset,
                /*offscreen_texture_resolution=*/std::nullopt,
                ComponentHandle<PrecomputeTexturePipeline>(),
                ComponentHandle<GroupToProjectionQuadTextureRenderer>()));
          }
          // TODO: Pass in aabb of the gsplat scene so that the precompute pass
          // doesn't run if the scene is frustum culled.
          return BuildPrecomputeTexturePipeline(gsplat_node)
              .Then([gsplat_node, bridge_id, material_mode, material_asset](
                        ComponentHandle<PrecomputeTexturePipeline>
                            pipeline) mutable -> BuiltInMaterialPtr {
                return Create(
                    gsplat_node, bridge_id, material_mode, material_asset,
                    /*offscreen_texture_resolution=*/std::nullopt, pipeline,
                    ComponentHandle<GroupToProjectionQuadTextureRenderer>());
              });
        });
  }
}

BuiltInMaterialPtr GsplatMaterialDeserializer::Create(
    NodeHandle gsplat_node, BridgeId bridge_id,
    android_xr::schemas::GsplatMode material_mode,
    AssetPtr<MaterialAsset> material_asset,
    std::optional<imp::uint2> offscreen_texture_resolution,
    ComponentHandle<PrecomputeTexturePipeline> precompute_texture_pipeline,
    ComponentHandle<GroupToProjectionQuadTextureRenderer>
        group_to_projection_quad_texture_renderer) {
  return BuiltInMaterialPtr(new GsplatMaterialDeserializer(
      gsplat_node, bridge_id, material_mode,
      gsplat_node->GetView().GetMaterialFactory().CreateMaterial(
          material_asset),
      offscreen_texture_resolution, precompute_texture_pipeline,
      group_to_projection_quad_texture_renderer));
}

GsplatMaterialDeserializer::GsplatMaterialDeserializer(
    NodeHandle gsplat_node, BridgeId bridge_id,
    android_xr::schemas::GsplatMode material_mode, OwnedMaterialPtr material,
    std::optional<imp::uint2> offscreen_texture_resolution,
    ComponentHandle<PrecomputeTexturePipeline> precompute_texture_pipeline,
    ComponentHandle<GroupToProjectionQuadTextureRenderer>
        group_to_projection_quad_texture_renderer)
    : BuiltInCustomMaterial(bridge_id, std::move(material)),
      view_(gsplat_node->GetView()),
      gsplat_node_(gsplat_node),
      material_mode_(material_mode),
      offscreen_texture_resolution_(offscreen_texture_resolution) {
  precompute_texture_pipeline_ = precompute_texture_pipeline;
  group_to_projection_quad_texture_renderer_ =
      group_to_projection_quad_texture_renderer;

  if (material_mode == android_xr::schemas::GsplatMode::MAGIC_WINDOW) {
    GetRenderMaterial()->SetParameter(
        kBaseColorParameter,
        view_.GetTextureRegistry().BorrowTexture(
            group_to_projection_quad_texture_renderer_->GetState()
                .texture_name.Value()));
  }
}

GsplatMaterialDeserializer::~GsplatMaterialDeserializer() {
  if (precompute_texture_pipeline_) {
    // Release the pass texture before destroying the pass.
    BorrowedTexturePtr placeholder_texture =
        view_.GetTextureFactory().BorrowRGBA32UIPlaceholderTexture();
    GetRenderMaterial()->SetParameter(kSplatDataPrecomputedParameter,
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

  if (material_mode_ == android_xr::schemas::GsplatMode::MAGIC_WINDOW) {
    // Mode 2: Magic Window.
    const android_xr::schemas::MagicWindowParameters* magic_window_params =
        serialized_parameters->mode_parameters_as_MagicWindowParameters();
    if (magic_window_params == nullptr) {
      return absl::InvalidArgumentError(
          "Missing MagicWindowParameters for MAGIC_WINDOW mode");
    }
    MP_RETURN_IF_ERROR(SetMagicWindowMaterialParameters(texture_borrower,
                                                     *magic_window_params));
  } else {
    // Mode 1: Standard Gsplat rendering.
    const android_xr::schemas::GsplatParameters* gsplat_params =
        serialized_parameters->mode_parameters_as_GsplatParameters();
    if (gsplat_params == nullptr) {
      return absl::InvalidArgumentError(
          "Missing GsplatParameters for GSPLAT mode");
    }
    MP_RETURN_IF_ERROR(
        SetRenderMaterialParameters(texture_borrower, *gsplat_params));

    if (precompute_texture_pipeline_) {
      // Mode 1A: With internal precompute data pipeline.
      MP_RETURN_IF_ERROR(SetRawGsplatDataParameters(
          texture_borrower, *gsplat_params, GetPrecomputedDataMaterial()));
      MP_RETURN_IF_ERROR(
          UpdatePrecomputeTexturePipeline(texture_borrower, *gsplat_params));
    } else if (gsplat_params->precomputed_data_texture()) {
      // Mode 1B: With client provided precompute data texture.
      MP_RETURN_IF_ERROR(SetMaterialParameterFromFlatbuffer(
          texture_borrower, GetRenderMaterial(),
          gsplat_params->precomputed_data_texture(),
          kSplatDataPrecomputedParameter));
    }
  }

  return absl::OkStatus();
}

split_engine::BuiltInMaterialPtr GsplatMaterialDeserializer::Duplicate() const {
  ComponentHandle<PrecomputeTexturePipeline> pipeline;
  if (precompute_texture_pipeline_) {
    Future<ComponentHandle<PrecomputeTexturePipeline>> pipeline_future =
        BuildPrecomputeTexturePipeline(gsplat_node_);
    if (pipeline_future.Ready() && pipeline_future.Get().ok()) {
      pipeline = pipeline_future.Get().value();
    } else {
      pipeline_future.Cancel();
      // Does not exit if NDEBUG is defined.
      IMP_LOG(imp::FATAL) << "Failed to duplicate precompute texture pipeline.";
    }
  }

  ComponentHandle<GroupToProjectionQuadTextureRenderer>
      group_to_projection_quad_texture_renderer;
  if (group_to_projection_quad_texture_renderer_) {
    Future<ComponentHandle<GroupToProjectionQuadTextureRenderer>>
        group_to_projection_quad_future =
            group_to_projection_quad_texture_renderer_->GetNode()
                ->AddComponentWithState<GroupToProjectionQuadTextureRenderer>(
                    group_to_projection_quad_texture_renderer_->GetState());
    if (group_to_projection_quad_future.Ready() &&
        group_to_projection_quad_future.Get().ok()) {
      group_to_projection_quad_texture_renderer =
          group_to_projection_quad_future.Get().value();
    } else {
      group_to_projection_quad_future.Cancel();
      // Does not exit if NDEBUG is defined.
      IMP_LOG(imp::FATAL) << "Failed to duplicate "
                     "GroupToProjectionQuadTextureRenderer.";
    }
  }

  OwnedMaterialPtr material = view_.GetMaterialFactory().WrapMaterial(
      filament::MaterialInstance::duplicate(GetFilamentMaterialInstance()));
  split_engine::BuiltInMaterialPtr result =
      absl::WrapUnique(new GsplatMaterialDeserializer(
          gsplat_node_, GetBridgeId(), material_mode_, std::move(material),
          offscreen_texture_resolution_, pipeline,
          group_to_projection_quad_texture_renderer));
  return result;
}

absl::Status GsplatMaterialDeserializer::SetRenderMaterialParameters(
    const TextureBorrower& texture_borrower,
    const android_xr::schemas::GsplatParameters& serialized_parameters) {
  BorrowedMaterialPtr render_material = GetRenderMaterial();

  if (const android_xr::schemas::Float* splat_scale =
          serialized_parameters.splat_scale()) {
    if (!HasParameter(kSplatScaleParameter)) {
      return MakeParameterNotImplementedError(kSplatScaleParameter);
    }
    render_material->SetParameter(kSplatScaleParameter, UnPack(*splat_scale));
  }

  if (const android_xr::schemas::BuiltInTextureParameter*
          sorted_indices_texture =
              serialized_parameters.sorted_indices_texture()) {
    if (!HasParameter(kSortedIndicesParameter)) {
      return MakeParameterNotImplementedError(kSortedIndicesParameter);
    }
    MP_RETURN_IF_ERROR(SetMaterialParameterFromFlatbuffer(
        texture_borrower, render_material, sorted_indices_texture,
        kSortedIndicesParameter));
  }
  return absl::OkStatus();
}

absl::Status GsplatMaterialDeserializer::SetMagicWindowMaterialParameters(
    const TextureBorrower& texture_borrower,
    const android_xr::schemas::MagicWindowParameters& serialized_parameters) {
  BorrowedMaterialPtr magic_window_material = GetRenderMaterial();

  if (group_to_projection_quad_texture_renderer_.IsValid()) {
    if (const android_xr::schemas::ProjectionQuad*
            magic_window_projection_quad =
                serialized_parameters.magic_window_projection_quad()) {
      const imp::TexturePipelineRendererProjectionQuad projection_quad =
          GetWorldProjectionQuad(*magic_window_projection_quad, gsplat_node_);
      MP_RETURN_IF_ERROR(group_to_projection_quad_texture_renderer_
                          ->SetProjectionQuadStateInfo(imp::ProjectionQuadState{
                              .size = projection_quad.size,
                              .center = projection_quad.center,
                              .rotation = projection_quad.rotation,
                          }));
    } else {
      MP_RETURN_IF_ERROR(group_to_projection_quad_texture_renderer_
                          ->SetProjectionQuadStateInfo(std::nullopt));
    }
  }

  return absl::OkStatus();
}

absl::Status GsplatMaterialDeserializer::SetRawGsplatDataParameters(
    const TextureBorrower& texture_borrower,
    const android_xr::schemas::GsplatParameters& serialized_parameters,
    BorrowedMaterialPtr material) {
  if (!material) {
    return absl::InvalidArgumentError(
        "Cannot set raw Gsplat data parameters: material is null");
  }

  MP_RETURN_IF_ERROR(SetMaterialParameterFromFlatbuffer(
      texture_borrower, material, serialized_parameters.position_data_texture(),
      kPositionDataTextureParameter));
  MP_RETURN_IF_ERROR(SetMaterialParameterFromFlatbuffer(
      texture_borrower, material, serialized_parameters.cov3d_data_texture(),
      kCov3dDataTextureParameter));
  MP_RETURN_IF_ERROR(SetMaterialParameterFromFlatbuffer(
      texture_borrower, material, serialized_parameters.color_data_texture(),
      kColorDataTextureParameter));

  const android_xr::schemas::Float2* min_screen_size =
      serialized_parameters.min_screen_size();
  if (min_screen_size && material->HasParameter(kMinScreenSizeParameter)) {
    material->SetParameter(kMinScreenSizeParameter, UnPack(*min_screen_size));
  }

  const android_xr::schemas::Float2* max_screen_size =
      serialized_parameters.max_screen_size();
  if (max_screen_size && material->HasParameter(kMaxScreenSizeParameter)) {
    material->SetParameter(kMaxScreenSizeParameter, UnPack(*max_screen_size));
  }

  const android_xr::schemas::Float* opacity_scale =
      serialized_parameters.opacity_scale();
  if (opacity_scale && material->HasParameter(kOpacityScaleParameter)) {
    material->SetParameter(kOpacityScaleParameter, UnPack(*opacity_scale));
  }

  return absl::OkStatus();
}

absl::Status GsplatMaterialDeserializer::UpdatePrecomputeTexturePipeline(
    const TextureBorrower& texture_borrower,
    const android_xr::schemas::GsplatParameters& serialized_parameters) {
  if (!precompute_texture_pipeline_) {
    return absl::OkStatus();
  }
  // Resize the pass texture based on the size provided in data textures.
  MP_ASSIGN_OR_RETURN(
      imp::uint2 position_data_texture_size,
      GetTextureSizeFromFlatbuffer(texture_borrower, serialized_parameters));
  MP_RETURN_IF_ERROR(precompute_texture_pipeline_->ResizePassTexture(
      0, position_data_texture_size));

  // Update the view resolution used for PrecomputeTexturePipeline:
  // - When `view_resolution` is specified, use the provided resolution.
  //   This take precedence over `magic_window_offscreen_resolution`.
  // - When `magic_window_offscreen_resolution` is specified, the Gsplat scene
  //   is meant to be rendered to an offscreen texture, and we should use the
  //   provided resolution.
  // - Otherwise, `offscreen_texture_resolution_` remains unchanged.
  if (const android_xr::schemas::Float2* view_resolution =
          serialized_parameters.view_resolution()) {
    offscreen_texture_resolution_ = uint2(UnPack(*view_resolution));
  } else if (const android_xr::schemas::Uint2*
                 magic_window_offscreen_resolution =
                     serialized_parameters
                         .magic_window_offscreen_resolution()) {
    offscreen_texture_resolution_ = UnPack(*magic_window_offscreen_resolution);
  }

  MP_RETURN_IF_ERROR(UpdatePrecomputeViewResolution(
      view_, offscreen_texture_resolution_,
      precompute_texture_pipeline_->BorrowMaterial()));

  // TODO: (broken link) - Delay initial use of TPR to workaround black screen.
  static int count = GetFrameDelayForTPR(view_);
  if (!precompute_texture_pipeline_->IsRunningAsyncSetup() &&
      !precompute_texture_pipeline_->GetNode()->IsEnabled()) {
    if (count == 0) {
      precompute_texture_pipeline_->GetNode()->SetEnabled(true);
    } else {
      --count;
    }
  }

  if (const android_xr::schemas::ProjectionQuad* magic_window_projection_quad =
          serialized_parameters.magic_window_projection_quad()) {
    precompute_texture_pipeline_->SetProjectionQuad(
        GetWorldProjectionQuad(*magic_window_projection_quad, gsplat_node_));
  } else {
    precompute_texture_pipeline_->SetProjectionQuad(std::nullopt);
  }

  // Set the precompute texture on the render material.
  BorrowedTexturePtr precompute_texture =
      precompute_texture_pipeline_->BorrowTexture();
  if (!precompute_texture) {
    return absl::InternalError("Failed to borrow precompute texture");
  }
  GetRenderMaterial()->SetParameter(kSplatDataPrecomputedParameter,
                                    precompute_texture,
                                    precompute_texture->GetSampler());
  return absl::OkStatus();
}

BorrowedMaterialPtr GsplatMaterialDeserializer::GetPrecomputedDataMaterial(
    SmallSourceLocation loc) const {
  if (!precompute_texture_pipeline_) {
    IMP_LOG(imp::ERROR) << "Cannot get precomputed material: precompute texture "
                  "pipeline is not available";
    return nullptr;
  }
  return precompute_texture_pipeline_->BorrowMaterial(loc);
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
