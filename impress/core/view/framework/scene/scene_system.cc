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

#include "core/view/framework/scene/scene_system.h"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_split.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "absl/types/span.h"
#include "absl/types/variant.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/async/future_status_utils.h"
#include "core/common/filament_helpers.h"
#include "core/common/hash.h"
#include "core/config.h"
#include "core/graph/dependency_graph.h"
#include "core/math/almost_equal.h"
#include "core/math/quat.h"
#include "core/math/transform.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/isf_info.h"
#include "core/ncsb/node_data.proto.imp.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/scene_metadata.h"
#include "core/ncsb/system.h"
#include "core/proto/any.proto.imp.h"
#include "core/proto/parse_message_visitor.h"
#include "core/proto/proto_reader.h"
#include "core/proto/proto_writer.h"
#include "core/proto/textproto_reader.h"
#include "core/resources/resource_manager.h"
#include "core/scene_handles/scene_handle_interface.h"
#include "core/view/base_view.h"
#include "core/view/framework/animation/gltf_animator.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/framework/assets/proto_asset.h"
#include "core/view/framework/camera/camera_component.h"
#include "core/view/framework/collision/box_collider.h"
#include "core/view/framework/collision/capsule_collider.h"
#include "core/view/framework/collision/cone_collider.h"
#include "core/view/framework/collision/cylinder_collider.h"
#include "core/view/framework/collision/mesh_collider.h"
#include "core/view/framework/collision/sphere_collider.h"
#include "core/view/framework/lighting/light_component.h"
#include "core/view/framework/scene/load_scene_visitor.h"
#include "core/view/framework/scene/scene_component_deserializer.h"
#include "core/view/framework/scene/scene_identifier.h"
#include "core/view/framework/scene/scene_reference.h"
#include "core/view/utils/asset.h"
#include "robin_map/include/tsl/robin_map.h"
#include "robin_map/include/tsl/robin_set.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {
namespace {

// Empty string view that is guaranteed to stay in memory.
constexpr absl::string_view kEmptyStringview = "";

void ApplyTransform(ComponentHandle<SceneMetadata> scene_metadata,
                    NodeHandle node, const TransformData& data, bool is_base) {
  float3 scale;
  float3 position;
  quatf rotation;
  Decompose(node->GetLocalTrs(), &position, &rotation, &scale);

  position = data.position.value_or(position);

  if (scene_metadata && is_base && data.position) {
    scene_metadata->SetBaseLocalPosition(position);
  }

  // Default to a scale of one unless the scale field was assigned in the proto.
  scale = data.scale.value_or(scale);

  if (scene_metadata && is_base && data.scale) {
    scene_metadata->SetBaseLocalScale(scale);
  }

  // Default to an identity quatf unless the rotation was assigned in the proto.
  if (data.rotation()) {
    rotation = *data.rotation();
  } else if (data.euler_angles()) {
    rotation = QuatFromEuler(*data.euler_angles());
  }

  if (scene_metadata && is_base &&
      !absl::holds_alternative<absl::monostate>(data.rotation_oneof)) {
    scene_metadata->SetBaseLocalRotation(rotation);
  }

  node->SetLocalTrs(Compose(position, rotation, scale));
}

bool SaveTransform(ComponentHandle<SceneMetadata> scene_metadata,
                   NodeHandle node, TransformData* data,
                   SceneSystem::SaveMode mode) {
  bool result = false;

  Transform<float> transform(node->GetLocalTrs());

  if (mode == SceneSystem::SaveMode::kAuthoredContent) {
    std::optional<float3> base_position =
        scene_metadata->GetBaseLocalPosition();
    bool should_save =
        (!base_position && !AlmostEqual(transform.translation, kZero3)) ||
        (base_position &&
         !AlmostEqual(base_position.value(), transform.translation));
    if (should_save) {
      data->position = transform.translation;
      result = true;
    }
  } else if (!AlmostEqual(transform.translation, kZero3)) {
    data->position = transform.translation;
    result = true;
  }

  if (mode == SceneSystem::SaveMode::kAuthoredContent) {
    std::optional<quatf> base_rotation = scene_metadata->GetBaseLocalRotation();
    bool should_save =
        (!base_rotation && !AlmostEqual(transform.rotation, kIdentityQuatf)) ||
        (base_rotation &&
         !AlmostEqual(base_rotation.value(), transform.rotation));
    if (should_save) {
      data->rotation_oneof = EulerFromQuatClamped(transform.rotation);
      result = true;
    }
  } else if (!AlmostEqual(transform.rotation, kIdentityQuatf)) {
    // Save as euler for human readability.
    data->rotation_oneof = EulerFromQuatClamped(transform.rotation);
    result = true;
  }

  if (mode == SceneSystem::SaveMode::kAuthoredContent) {
    std::optional<float3> base_scale = scene_metadata->GetBaseLocalScale();
    bool should_save =
        (!base_scale && !AlmostEqual(transform.scale, kOne3)) ||
        (base_scale && !AlmostEqual(base_scale.value(), transform.scale));
    if (should_save) {
      data->scale = transform.scale;
      result = true;
    }
  } else if (!AlmostEqual(transform.scale, kOne3)) {
    data->scale = transform.scale;
    result = true;
  }

  return result;
}
}  // namespace

SceneSystem::SceneSystem(BaseView* view) : System(view) {}

Future<NodeHandle> SceneSystem::LoadScene(
    const AssetDefinition& asset_definition, LoadSceneOptions options) {
  return LoadScene(asset_definition.GetUrl(), std::move(options));
}

Future<NodeHandle> SceneSystem::LoadScene(absl::string_view asset_url,
                                          LoadSceneOptions options) {
  return GetView()
      .GetAssetManager()
      // Pass retain_resource = true when loading ProtoAssets that directly
      // reference underlying memory, which is required here because the
      // "components" field of NodeData is a repeated Any, which directly
      // references underlying Cord data until unpacked.
      .LoadProto<NodeData>(asset_url, true)
      .Then([this, asset_url_copy = std::string{asset_url},
             options = std::move(options)](
                const AssetPtr<ProtoAsset<NodeData>>& node_data_asset) mutable
                -> Future<NodeHandle> {
        NodeHandle node = GetView().CreateNode();
        node->SetEnabled(false);
        if (options.parent) {
          node->SetParent(options.parent);
        }

        auto unique_load_scene_visitor = std::make_unique<LoadSceneVisitor>(
            std::move(options.load_scene_visitor));

        const NodeData& node_data = node_data_asset->GetProto();

        Future<NodeHandle> result = LoadIntoNodeFromProto(
            node, node_data, asset_url_copy, unique_load_scene_visitor.get(),
            options.metadata_mode);
        result.DependsOn(std::move(unique_load_scene_visitor));
        // Hold onto the asset to ensure that proto data is valid for the
        // lifetime of the future.
        result.DependsOn(node_data_asset);
        return result;
      });
}

Future<NodeHandle> SceneSystem::LoadScene(const NodeData& node_data,
                                          absl::string_view path,
                                          LoadSceneOptions options) {
  NodeHandle node = GetView().CreateNode();
  node->SetEnabled(false);
  if (options.parent) {
    node->SetParent(options.parent);
  }

  auto unique_load_scene_visitor =
      std::make_unique<LoadSceneVisitor>(std::move(options.load_scene_visitor));

  // Copy the node data because it must be retained until after the load
  // completes.
  std::unique_ptr<NodeData> node_data_copy =
      std::make_unique<NodeData>(node_data);
  Future<NodeHandle> result = LoadIntoNodeFromProto(
      node, *node_data_copy, !path.empty() ? path : kRuntimeNodeDataPath,
      unique_load_scene_visitor.get(), options.metadata_mode);
  result.DependsOn(std::move(unique_load_scene_visitor));
  result.DependsOn(std::move(node_data_copy));

  return result;
}

Future<NodeHandle> SceneSystem::LoadSceneFromTextproto(
    resources::Resource resource, absl::string_view path,
    LoadSceneOptions options) {
  // NOTE: need to retain the NodeData through the duration of the load.
  std::shared_ptr<NodeData> scene_isf = std::make_shared<NodeData>();
  if (absl::Status status = proto::ParseTextproto(
          resource.GetData().StringView(), scene_isf.get());
      !status.ok()) {
    return Future<NodeHandle>(status);
  }
  auto result = LoadScene(*scene_isf, path, std::move(options));
  result.DependsOn(scene_isf);
  result.DependsOn(resource);
  return result;
}

Future<NodeHandle> SceneSystem::LoadScene(
    const AssetDefinition& asset_definition, NodeHandle parent) {
  return LoadScene(asset_definition, LoadSceneOptions{.parent = parent});
}

Future<NodeHandle> SceneSystem::LoadScene(
    const AssetDefinition& asset_definition,
    LoadSceneVisitor load_scene_visitor, NodeHandle parent) {
  return LoadScene(
      asset_definition,
      LoadSceneOptions{.parent = parent,
                       .load_scene_visitor = std::move(load_scene_visitor)});
}

Future<NodeHandle> SceneSystem::LoadScene(absl::string_view asset_url,
                                          NodeHandle parent) {
  return LoadScene(asset_url, LoadSceneOptions{.parent = parent});
}

Future<NodeHandle> SceneSystem::LoadScene(absl::string_view asset_url,
                                          LoadSceneVisitor load_scene_visitor,
                                          NodeHandle parent) {
  return LoadScene(
      asset_url,
      LoadSceneOptions{.parent = parent,
                       .load_scene_visitor = std::move(load_scene_visitor)});
}

Future<NodeHandle> SceneSystem::LoadScene(const NodeData& node_data,
                                          absl::string_view path,
                                          NodeHandle parent) {
  return LoadScene(node_data, path, LoadSceneOptions{.parent = parent});
}

Future<NodeHandle> SceneSystem::LoadScene(const NodeData& node_data,
                                          LoadSceneVisitor load_scene_visitor,
                                          absl::string_view path,
                                          NodeHandle parent) {
  return LoadScene(
      node_data, path,
      LoadSceneOptions{.parent = parent,
                       .load_scene_visitor = std::move(load_scene_visitor)});
}

Future<NodeHandle> SceneSystem::LoadIntoNodeFromProto(
    NodeHandle node, const NodeData& data, absl::string_view asset_url,
    LoadSceneVisitor* load_scene_visitor, MetadataMode metadata_mode) {
  // Need to map from data to created node, since filament children aren't
  // ordered.
  auto unique_load_scene_info = std::make_unique<LoadSceneInfo>();
  LoadSceneInfo* load_scene_info = unique_load_scene_info.get();
  load_scene_info->load_scene_visitor = load_scene_visitor;
  if (data.disabled.has_value()) {
    load_scene_info->is_root_disabled = *data.disabled;
  }
  load_scene_info->metadata_mode = metadata_mode;

  node->AddComponent<SceneReference>(asset_url);
  Future<absl::Status> create_hierarchy_future = CreateSceneHierarchy(
      data, node, load_scene_info, load_scene_visitor, /*is_base=*/false);

  // Finish importing children before setting up components.
  // This is necessary to hook up ComponentSceneHandle references from
  // imported children, and so that components can access the contents of
  // imported children during setup.
  Future<NodeHandle> result =
      create_hierarchy_future
          .Then([this, load_scene_info]() mutable {
            return SetupComponents(load_scene_info);
          })
          .Then([this, asset_url_copy = std::string{asset_url}, node,
                 load_scene_info](
                    absl::Status status) -> absl::StatusOr<NodeHandle> {
            // Convert from Future<Status> to Future<NodeHandle> and
            // delete the node if loading failed.
            if (status.ok()) {
              // The root node is disabled while the load is occurring. This
              // line re-enables the root node if it hasn't been explicitly
              // disabled in the actual .isf file.
              node->SetEnabled(!load_scene_info->is_root_disabled);
              if (node->GetName().empty()) {
                // Split the path on '/' characters to extract the filename.
                std::vector<std::string> path =
                    absl::StrSplit(asset_url_copy, '/');
                // Remove the file extension.
                std::vector<std::string> file =
                    absl::StrSplit(path.back(), '.');
                node->SetName(file.front());
              }
              return node;
            } else {
              IMP_LOG(imp::WARNING) << "[SceneComponent] Failure loading file: "
                           << asset_url_copy << "\nStatus: " << status;
              GetView().DestroyNode(node);
              return status;
            }
          });
  result.DependsOn(std::move(unique_load_scene_info));

  return result;
}

absl::StatusOr<std::string> SceneSystem::SaveToString(NodeHandle node,
                                                      SaveMode mode) {
  NodeData state;
  absl::StatusOr<SaveResult> result =
      SaveImpl(node, &state, mode, /*is_root_node=*/true);
  std::string data;
  if (result.ok()) {
    if (!proto::SerializeTo(&state, &data)) {
      return absl::InternalError("[SceneSystem] Failed to write NodeData");
    }
  }
  return data;
}

absl::StatusOr<NodeData> SceneSystem::SaveToData(NodeHandle node,
                                                 SaveMode mode) {
  NodeData state;
  absl::StatusOr<SaveResult> result =
      SaveImpl(node, &state, mode, /*is_root_node=*/true);
  if (result.ok()) {
    return state;
  }
  return result.status();
}

template <typename T>
absl::StatusOr<absl::string_view> GetAnyValueAsStringView(const T& any_value) {
  return absl::StatusOr<absl::string_view>(any_value);
}

template <>
absl::StatusOr<absl::string_view> GetAnyValueAsStringView(
    const absl::Cord& any_value) {
  absl::optional<absl::string_view> flat = any_value.TryFlat();
  if (!flat.has_value()) {
    return absl::InvalidArgumentError("Given cord is not flat.");
  }
  return *flat;
}

template <typename T>
void TryFlattenCord(T& any_value) {
  // No-op.
}

void TryFlattenCord(absl::Cord& cord) { cord.Flatten(); }

absl::StatusOr<SceneSystem::ExtractedComponentAny>
SceneSystem::VerifyAndExtractComponentAny(
    const google::protobuf::imp_proto::Any& component_any) {
  SceneSystem::ExtractedComponentAny extracted_component_any;
  extracted_component_any.type_url = std::string(component_any.type_url);
  extracted_component_any.type = Hash(extracted_component_any.type_url);

  absl::StatusOr<absl::string_view> any_value =
      GetAnyValueAsStringView(component_any.value);
  if (!any_value.ok()) {
    // Note: this may happen when the value of an Any is created dynamically.
    // To fix this, call any->value.Flatten() before storing it.
    return absl::InvalidArgumentError(absl::StrFormat(
        "Any::value cord wasn't flat: %s", extracted_component_any.type_url));
  }
  extracted_component_any.component_data = *any_value;

  // For stateless components, need to grab the actual type:
  if (extracted_component_any.type == StatelessComponent::kTypeUrlHash) {
    StatelessComponent comp;
    if (proto::ParseMessage(extracted_component_any.component_data, &comp)) {
      extracted_component_any.type_url = comp.type;
      extracted_component_any.type = Hash(extracted_component_any.type_url);
      extracted_component_any.component_data = kEmptyStringview;
    } else {
      return absl::InvalidArgumentError(
          absl::StrFormat("Failed to parse stateless component: %s",
                          extracted_component_any.component_data));
    }
  }

  const SceneComponentDeserializer::Handler* handler =
      scene_component_deserializer_.GetHandler(extracted_component_any.type);

  if (!handler) {
    return absl::UnavailableError(absl::StrFormat(
        "[SceneComponent] Skipping unknown component type: %s. Was the "
        "component registered by calling "
        "SceneSystem::RegisterComponentIsfInfo?",
        extracted_component_any.type_url));
  }

  return extracted_component_any;
}

Future<absl::Status> SceneSystem::AddComponent(
    NodeHandle node, const google::protobuf::imp_proto::Any& component_any) {
  absl::StatusOr<SceneSystem::ExtractedComponentAny> extracted_component_any =
      VerifyAndExtractComponentAny(component_any);
  if (!extracted_component_any.ok()) {
    return Future<absl::Status>(extracted_component_any.status());
  }
  const SceneComponentDeserializer::Handler* handler =
      scene_component_deserializer_.GetHandler(extracted_component_any->type);

  auto& cm = GetView().GetComponentManager();

  handler->remove_component(node->GetEntity(), &cm);

  if (!handler->parse(node, &cm, {extracted_component_any->component_data},
                      nullptr, true)) {
    absl::Status status(
        absl::StatusCode::kInvalidArgument,
        absl::StrCat("Failed to parse component: ", handler->type_url,
                     " on entity ", node->GetName()));
    return Future<absl::Status>(status);
  }
  return handler->default_setup(node->GetEntity(), &cm, true);
}

Future<absl::Status> SceneSystem::UpdateComponent(
    NodeHandle node, const google::protobuf::imp_proto::Any& component_any) {
  absl::StatusOr<SceneSystem::ExtractedComponentAny> extracted_component_any =
      VerifyAndExtractComponentAny(component_any);
  if (!extracted_component_any.ok()) {
    return Future<absl::Status>(extracted_component_any.status());
  }
  const SceneComponentDeserializer::Handler* handler =
      scene_component_deserializer_.GetHandler(extracted_component_any->type);

  auto& cm = GetView().GetComponentManager();

  if (!handler->has_component(node->GetEntity(), &cm)) {
    return Future<absl::Status>(absl::NotFoundError(
        absl::StrCat("Component not found: ", handler->type_url, " on entity ",
                     node->GetName())));
  }

  if (!handler->parse(node, &cm, {extracted_component_any->component_data},
                      nullptr, false)) {
    absl::Status status(
        absl::StatusCode::kInvalidArgument,
        absl::StrCat("Failed to parse component: ", handler->type_url,
                     " on entity ", node->GetName()));
    return Future<absl::Status>(status);
  }
  return handler->on_isf_state_changed(node->GetEntity(), &cm);
}

Future<absl::Status> SceneSystem::CreateSceneHierarchy(
    const NodeData& data, NodeHandle node, LoadSceneInfo* load_scene_info,
    LoadSceneVisitor* load_scene_visitor, bool is_base) {
  if (data.base.empty()) {
    // This node has no base, so apply the data directly.
    return ApplyData(data, node, load_scene_info, load_scene_visitor, is_base);
  } else {
    // This node has a base, so start by loading the hierarchy of the base .isf
    // file before we finish loading the data within this .isf file.
    return GetView()
        .GetAssetManager()
        .LoadProto<NodeData>(data.base)
        .Then([this, node, &data, load_scene_info,
               load_scene_visitor](AssetPtr<ProtoAsset<NodeData>> scene_asset) {
          load_scene_info->assets.push_back(scene_asset);

          if (load_scene_info->metadata_mode == MetadataMode::kInclude) {
            node->GetOrAddComponent<SceneMetadata>()->SetBaseUrl(data.base);
          }
          if (data.unique_id) {
            node->AddComponent<SceneIdentifier>(*data.unique_id);
          }

          return CreateSceneHierarchy(scene_asset->GetProto(), node,
                                      load_scene_info, load_scene_visitor,
                                      /*is_base=*/true);
        })
        .Then([this, node, &data, load_scene_info, load_scene_visitor,
               is_base]() mutable {
          return ApplyData(data, node, load_scene_info, load_scene_visitor,
                           is_base);
        });
  }
}

Future<absl::Status> SceneSystem::ApplyData(
    const NodeData& data, NodeHandle node, LoadSceneInfo* load_scene_info,
    LoadSceneVisitor* load_scene_visitor, bool is_base) {
  // The node has a unique_id. Therefore, we need to do two things:
  // 1. Add the node to ids_to_nodes map so that NodeSceneHandle fields can
  //    be hooked up by the id.
  // 2. Add the id to the node so that it can be saved back out later if the
  //    node is saved out to NodeData.
  if (data.unique_id) {
    const NodeUniqueId& unique_id = *data.unique_id;
    node->AddComponent<SceneIdentifier>(unique_id);

    // Check to make sure the unique_id hasn't appeared multiple times.
    auto ids_to_nodes_itr = load_scene_info->ids_to_nodes.find(unique_id);
    if (ids_to_nodes_itr != load_scene_info->ids_to_nodes.end() &&
        ids_to_nodes_itr->second != node) {
      return ReturnFuture(absl::InvalidArgumentError(absl::StrFormat(
          "unique id %i was duplicated on both %s and %s.", unique_id,
          ToString(node), ToString(ids_to_nodes_itr->second))));
    }

    // Assign it to the map so that NodeSceneHandle references can be hooked up
    // using it.
    load_scene_info->ids_to_nodes[unique_id] = node;
  }

  ComponentHandle<SceneMetadata> scene_metadata;
  if (load_scene_info->metadata_mode == MetadataMode::kInclude) {
    scene_metadata = node->GetOrAddComponent<SceneMetadata>();
  }

  // Set the node's name.
  if (!data.name.empty()) {
    node->SetName(data.name);
    load_scene_info->names_to_nodes[std::string(data.name)] = node;
  }

  // TODO: Add Support for saving groups.
  if (data.groups.has_value()) {
    node->SetGroupsFromVector(data.groups->groups);
  }

  // Copy the local transform data into filament
  ApplyTransform(scene_metadata, node, data.transform, is_base);

  if (data.disabled.has_value()) {
    if (scene_metadata && is_base) {
      scene_metadata->SetBaseDisabled(*data.disabled);
    }

    if (!node->GetParent()) {
      // If this is the root node, don't actually change the node's enabled
      // status. Just track it for later.
      load_scene_info->is_root_disabled = *data.disabled;
    } else {
      node->SetEnabled(!*data.disabled);
    }
  }

  // For each component on the node in the asset, add the necessary information
  // to attach the component to the node to handler_hash_to_node_and_comp_data.
  // Actually deserializing and attaching the component is deferred until later
  // in the loading process. This is so that all nodes are created before any
  // component is added, allowing NodeSceneHandle references to be hooked up so
  // that components can reference other nodes in the scene.
  tsl::robin_map<uint32_t, tsl::robin_set<HashValue>> seen_components_for_node;
  uint32_t node_entity_id = node->GetEntity().getId();
  for (const google::protobuf::imp_proto::Any& component_any :
       data.components) {
    absl::StatusOr<SceneSystem::ExtractedComponentAny> extracted_component_any =
        VerifyAndExtractComponentAny(component_any);
    if (!extracted_component_any.ok()) {
      return Future<absl::Status>(extracted_component_any.status());
    }

    if (seen_components_for_node[node_entity_id].count(
            extracted_component_any->type)) {
      return ReturnFuture(absl::AlreadyExistsError(
          absl::StrFormat("Component of type %s already exists on node.",
                          extracted_component_any->type_url)));
    }
    seen_components_for_node[node_entity_id].insert(
        extracted_component_any->type);

    LoadSceneInfo::ComponentInfo& component_info =
        load_scene_info
            ->handler_hash_to_node_and_comp_data[extracted_component_any->type]
                                                [node];
    component_info.component_data.push_back(
        extracted_component_any->component_data);

    if (scene_metadata) {
      scene_metadata->SetComponentAuthored(extracted_component_any->type,
                                           /*is_authored=*/true);
      if (is_base) {
        scene_metadata->PushBaseComponentSources(
            extracted_component_any->type, /*disabled=*/false,
            extracted_component_any->component_data);
      }
    }
  }

  // Do the same as above for components that include metadata.
  for (const ComponentWithMetadata& component_with_metadata :
       data.components_with_metadata) {
    absl::StatusOr<SceneSystem::ExtractedComponentAny> extracted_component_any =
        VerifyAndExtractComponentAny(component_with_metadata.component);
    if (!extracted_component_any.ok()) {
      return Future<absl::Status>(extracted_component_any.status());
    }

    if (seen_components_for_node[node_entity_id].count(
            extracted_component_any->type)) {
      return ReturnFuture(absl::AlreadyExistsError(
          absl::StrFormat("Component of type %s already exists on node.",
                          extracted_component_any->type_url)));
    }
    seen_components_for_node[node_entity_id].insert(
        extracted_component_any->type);

    LoadSceneInfo::ComponentInfo& component_info =
        load_scene_info
            ->handler_hash_to_node_and_comp_data[extracted_component_any->type]
                                                [node];
    component_info.component_data.push_back(
        extracted_component_any->component_data);
    component_info.component_with_metadata = &component_with_metadata;

    if (scene_metadata) {
      scene_metadata->SetComponentAuthored(extracted_component_any->type,
                                           /*is_authored=*/true);
      if (is_base) {
        scene_metadata->PushBaseComponentSources(
            extracted_component_any->type, component_with_metadata.disabled,
            extracted_component_any->component_data);
      }
    }
  }

  // Create child nodes and distribute the data to them.
  Future<absl::Status> result(absl::OkStatus());

  // If there are preexisting children, then they come from a base .isf file.
  std::vector<NodeHandle> preexisting_children = node->GetChildren();

  for (const NodeData& child_data : data.children) {
    NodeHandle child;

    // Try to find pre-existing child by name that this one maps to.
    if (!child_data.name.empty()) {
      for (NodeHandle preexisting_child : preexisting_children) {
        if (preexisting_child->GetName() == child_data.name) {
          child = preexisting_child;
          break;
        }
      }
    }

    if (!child) {
      child = GetView().CreateNode();
      child->SetParent(node);
    }

    result = result.Combine(CreateSceneHierarchy(
        child_data, child, load_scene_info, load_scene_visitor, is_base));
  }

  return result;
}

Future<absl::Status> SceneSystem::SetupComponents(
    LoadSceneInfo* load_scene_info) {
  // Build the dependency graph of components based on the components in the
  // .isf file being loaded. Alternatively, SceneSystem could cache a single
  // dependency graph that includes all components that exist (either adding
  // lazily or during initialization) that is reused. Though in that case, we
  // would likely have to traverse a larger graph for each load and may make the
  // implementation more complex. That is something we can explore optimizing
  // when it becomes necessary.
  DependencyGraph<HashValue> graph;
  for (auto& pair : load_scene_info->handler_hash_to_node_and_comp_data) {
    auto& deps = scene_component_deserializer_.GetDeps(pair.first);
    if (deps.empty()) {
      graph.AddNode(pair.first);
    } else {
      // Make sure to add the node even if there are no deps.
      graph.AddNode(pair.first);

      for (auto dep : deps) {
        // TODO: There is a potential bug here where we can lose
        // transitive dependencies. If 'A' -> 'B' -> 'C', there could be a .isf
        // file that includes only 'A' and 'C', in that case, 'C' can be setup
        // before 'A'. That could be a problem in particular if 'A' dynamically
        // adds B if it doesn't already exist.

        // Don't add to the dep graph if this dependency is not actually in
        // this .isf file.
        if (load_scene_info->handler_hash_to_node_and_comp_data.find(dep) !=
            load_scene_info->handler_hash_to_node_and_comp_data.end()) {
          graph.AddDependency(pair.first, dep);
        }
      }
    }
  }

  // Traverse the dependency graph and call Setup for each component instance
  // in the order of component dependencies. ParallelTraverse automatically
  // handles parallel execution of async work from async Setups. We do this
  // by component type and not by node, so all components of type 'A' on every
  // node in the .isf will be setup before any components of type 'B'. This
  // allows us to support dependencies where component A needs to be setup
  // before component B even if component B is on a different node in the .isf.
  Future<absl::Status> result = graph.ParallelTraverse(
      [this, load_scene_info = std::move(*load_scene_info),
       &view = GetView()](HashValue type) mutable {
        Future<absl::Status> inner_result(absl::OkStatus());

        const SceneComponentDeserializer::Handler* handler =
            scene_component_deserializer_.GetHandler(type);
        auto& node_and_component_data_list =
            load_scene_info.handler_hash_to_node_and_comp_data[type];
        auto& cm = view.GetComponentManager();

        // Helper for tracking information shared between each visit using
        // ParseMessageVisitor.
        struct VisitorInfo {
          // Out status, if the visit method assigns this to an error then the
          // load has failed.
          absl::Status status = absl::OkStatus();
          // Current node containing the component being visited.
          NodeHandle current_node;
        };
        VisitorInfo visitor_info;

        // This visitor is used to hook up scene handle references when the
        // component's state is deserialized by mapping the scene handle
        // identifiers to the actual instantiated nodes when deserialization
        // occurs. This is used for scene handles.
        proto::ParseMessageVisitor visitor;
        visitor.OnVisit([&load_scene_info, &visitor_info](
                            SceneHandleInterface& scene_handle_interface) {
          // Map the NodeSceneHandle's identifier to the actual node created
          // from the .isf file.
          NodeHandle identified_node;
          const SceneHandleInterface::Identifier& identifier =
              scene_handle_interface.GetIdentifier();

          if (absl::holds_alternative<std::string>(identifier)) {
            auto itr = load_scene_info.names_to_nodes.find(
                absl::get<std::string>(identifier));
            if (itr != load_scene_info.names_to_nodes.end()) {
              identified_node = itr->second;
            }
          } else if (absl::holds_alternative<int32_t>(identifier)) {
            auto itr = load_scene_info.ids_to_nodes.find(
                absl::get<int32_t>(identifier));
            if (itr != load_scene_info.ids_to_nodes.end()) {
              identified_node = itr->second;
            }
          }

          visitor_info.status.Update(
              scene_handle_interface.AssignSceneHandleForIdentifier(
                  identified_node, visitor_info.current_node));
        });

        for (const auto& pair : node_and_component_data_list) {
          NodeHandle node = pair.first;
          const std::vector<absl::string_view>& component_data =
              pair.second.component_data;
          const ComponentWithMetadata* metadata =
              pair.second.component_with_metadata;

          visitor_info.current_node = node;

          // This adds the component to the node and deserializes the
          // component_data proto into the component.
          if (!handler->parse(node, &cm, component_data, &visitor, true)) {
            absl::Status status(
                absl::StatusCode::kInvalidArgument,
                absl::StrCat("Failed to parse component: ", handler->type_url,
                             " on entity ", node->GetName()));
            return Future<absl::Status>(status);
          }

          // Check if the visitor reported a failure.
          if (!visitor_info.status.ok()) {
            return Future<absl::Status>(visitor_info.status);
          }
          visitor_info.status = absl::OkStatus();

          bool should_enable_component = !metadata || !metadata->disabled;

          // This visits the LoadSceneVisitor passed in to LoadScene.
          absl::optional<Future<absl::Status>> visit_result =
              handler->visit(node, &cm, load_scene_info.load_scene_visitor,
                             should_enable_component);

          // Finally, this calls the Setup method of the component if there is
          // one specified.
          Future<absl::Status> setup_future;
          if (visit_result) {
            setup_future = *visit_result;
          } else {
            setup_future = handler->default_setup(node->GetEntity(), &cm,
                                                  should_enable_component);
          }
          if (setup_future.Ready()) {
            // If the future is already ready and has an ok status, then we
            // don't need to bother combining the futures. If the status is not
            // ok, then we can combine the future and break immediately since we
            // know the load has failed.
            if (!setup_future.Get().ok()) {
              inner_result = inner_result.Combine(setup_future);
              break;
            }
          } else {
            // Combine the future, the future isn't ready yet and we don't know
            // the result.
            inner_result = inner_result.Combine(setup_future);
          }
        }

        return inner_result;
      });

  return result;
}

absl::StatusOr<SceneSystem::SaveResult> SceneSystem::SaveImpl(
    NodeHandle node, NodeData* data, SaveMode mode, bool is_root_node) {
  SaveResult save_result = SaveResult::kNoAuthoredContent;

  auto scene_metadata = node->GetComponent<SceneMetadata>();
  if (mode == SaveMode::kAuthoredContent && !scene_metadata) {
    if (is_root_node) {
      return absl::InvalidArgumentError(
          "Can only save node in kAuthoredContent mode if it has a "
          "scene metadata (i.e. created from the editor or a .isf file with "
          "metadata included).");
    }
  }

  // Save the node's name.
  if (!is_root_node || data->base.empty()) {
    data->name = std::string(node->GetName());
  }

  // If we are saving authored content & this node has no metadata, it gets
  // skipped but it's children don't because the children may have authored
  // content.
  if ((scene_metadata && mode == SaveMode::kAuthoredContent) ||
      mode == SaveMode::kFull) {
    if (mode == SaveMode::kAuthoredContent) {
      data->base = scene_metadata->GetBaseUrl();
    }

    if (!data->base.empty()) {
      save_result = SaveResult::kIncludesAuthoredContent;
    }

    auto& handlers = scene_component_deserializer_.GetHandlers();
    auto& cm = GetView().GetComponentManager();

    // Save the local transform data
    bool was_transform_saved =
        SaveTransform(scene_metadata, node, &data->transform, mode);
    if (was_transform_saved) {
      save_result = SaveResult::kIncludesAuthoredContent;
    }

    // Save the enabled state of the node.
    if (mode == SaveMode::kAuthoredContent) {
      std::optional<bool> is_base_disabled = scene_metadata->IsBaseDisabled();
      bool should_save =
          (!is_base_disabled && !node->IsEnabled()) ||
          (is_base_disabled && node->IsEnabled() == *is_base_disabled);
      if (should_save) {
        data->disabled = !node->IsEnabled();
        save_result = SaveResult::kIncludesAuthoredContent;
      }
    } else if (!node->IsEnabled()) {
      data->disabled = true;
      save_result = SaveResult::kIncludesAuthoredContent;
    }

    if (auto scene_id = node->GetComponent<SceneIdentifier>()) {
      data->unique_id = scene_id->GetId();
    }

    // Save all the components
    for (auto& [type_hash, handler] : handlers) {
      if (mode == SaveMode::kAuthoredContent &&
          !scene_metadata->IsComponentAuthored(type_hash)) {
        continue;
      }

      // TODO Currently this iterates through all savable
      // components, checks if node has that component, then saves it.  It would
      // be more efficient to directly get a list of all components a node has,
      // then save those components.
      if (handler.has_component(node->GetEntity(), &cm)) {
        if (!handler.save) {
          return absl::FailedPreconditionError(absl::StrFormat(
              "Component %s is not registered with the Saving feature enabled. "
              "Either build in dev mode or call "
              "SceneSystem::RegisterComponentIsfInfo with the "
              "IsfOptionalFeature::kSaving flag.",
              handler.type_url));
        }

        absl::Span<const std::string> base_component_bytes;
        const SceneMetadata::ComponentSource* component_source =
            scene_metadata ? scene_metadata->GetBaseComponentSource(type_hash)
                           : nullptr;
        if (mode == SaveMode::kAuthoredContent && component_source) {
          base_component_bytes = component_source->component_bytes;
        }

        google::protobuf::imp_proto::Any component_any;
        bool is_enabled;
        SaveIsfStateResult save_state_result =
            handler.save(node->GetEntity(), cm, component_any, is_enabled,
                         base_component_bytes);

        if (save_state_result == SaveIsfStateResult::kFailed) {
          // This should only happen if there's an allocation failure, or if
          // there's a bug in the proto serialization code.
          return absl::InternalError(absl::StrFormat(
              "[SceneSystem] error serializing node %i's component of type: "
              "%.*s",
              node->GetEntity().getId(), handler.type_url.size(),
              handler.type_url.data()));
        }
        TryFlattenCord(component_any.value);

        if (mode == SaveMode::kAuthoredContent) {
          bool should_save_disabled =
              (!component_source && !is_enabled) ||
              (component_source && is_enabled == component_source->disabled);
          if (should_save_disabled) {
            data->components_with_metadata.push_back(
                ComponentWithMetadata{.disabled = !is_enabled,
                                      .component = std::move(component_any)});
            save_result = SaveResult::kIncludesAuthoredContent;
          } else if (save_state_result != SaveIsfStateResult::kEmpty) {
            data->components.push_back(std::move(component_any));
            save_result = SaveResult::kIncludesAuthoredContent;
          }
        } else {
          if (is_enabled) {
            data->components.push_back(std::move(component_any));
          } else {
            data->components_with_metadata.push_back(ComponentWithMetadata{
                .disabled = true, .component = std::move(component_any)});
          }
          save_result = SaveResult::kIncludesAuthoredContent;
        }
      }
    }
  }

  for (NodeHandle child : node->GetChildren()) {
    NodeData child_data;
    MP_ASSIGN_OR_RETURN(SaveResult child_save_result,
                     SaveImpl(child, &child_data, mode, false));

    if (child_save_result == SaveResult::kIncludesAuthoredContent) {
      save_result = SaveResult::kIncludesAuthoredContent;
      data->children.push_back(std::move(child_data));
    } else if (mode == SaveMode::kFull) {
      data->children.push_back(std::move(child_data));
    }
  }

  // Before returning, sort the children & components by name.
  // This is helpful so that when saving the file, the children & components
  // are in a consistent order. This improves the readability & reviewability
  // of the .isf file.
  if (data->children.size() > 1) {
    std::sort(data->children.begin(), data->children.end(),
              [](const NodeData& a, const NodeData& b) {
                // First ,sort by name.
                if (a.name != b.name) {
                  return a.name < b.name;
                }

                // Second, sort by unique_id.
                if (a.unique_id != b.unique_id) {
                  return a.unique_id < b.unique_id;
                }

                // If the above fail, sort by the contents of the proto.
                std::string a_serialized;
                proto::SerializeTo(&a, &a_serialized);
                std::string b_serialized;
                proto::SerializeTo(&b, &b_serialized);
                return a_serialized < b_serialized;
              });
  }
  std::sort(data->components.begin(), data->components.end(),
            [](const google::protobuf::imp_proto::Any& a,
               const google::protobuf::imp_proto::Any& b) {
              return a.type_url < b.type_url;
            });
  std::sort(data->components_with_metadata.begin(),
            data->components_with_metadata.end(),
            [](const ComponentWithMetadata& a, const ComponentWithMetadata& b) {
              return a.component.type_url < b.component.type_url;
            });

  return save_result;
}

void SceneSystem::RegisterDefaultComponentsIsfInfo() {
  RegisterComponentIsfInfo<GltfRenderer>();
  RegisterComponentIsfInfo<GltfAnimator>();
  RegisterComponentIsfInfo<LightComponent>();
  RegisterComponentIsfInfo<CameraComponent>();
  RegisterComponentIsfInfo<BoxCollider>();
  RegisterComponentIsfInfo<SphereCollider>();
  RegisterComponentIsfInfo<CapsuleCollider>();
  RegisterComponentIsfInfo<CylinderCollider>();
  RegisterComponentIsfInfo<ConeCollider>();
  RegisterComponentIsfInfo<MeshCollider>();
}

#if IMP_RUNTIME(DEV)
std::unique_ptr<editor::Widget> SceneSystem::CreateComponentWidget(
    HashValue component_state_type_url_hash, NodeHandle node,
    Dispatcher& editor_dispatcher) {
  return scene_component_deserializer_.CreateComponentWidget(
      component_state_type_url_hash, node, editor_dispatcher);
}

void SceneSystem::ShowAddComponentUi(NodeHandle node) {
  scene_component_deserializer_.ShowAddComponentUi(node);
}
#endif

}  // namespace imp
