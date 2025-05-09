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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_SCENE_SCENE_SYSTEM_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_SCENE_SCENE_SYSTEM_H_

#include <cstdint>
#include <string>
#include <vector>

#include "absl/base/attributes.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "core/assets/asset_cache.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/common/hash.h"
#include "core/common/pass_key.h"
#include "core/common/platform_helpers.h"
#include "core/common/robin_map.h"
#include "core/config.h"
#include "core/ncsb/component.h"
#include "core/ncsb/isf_info.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_data.proto.imp.h"
#include "core/ncsb/system.h"
#include "core/proto/any.proto.imp.h"
#include "core/proto/proto_reader.h"
#include "core/resources/resource_manager.h"
#include "core/view/async/future.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/proto_asset.h"
#include "core/view/framework/scene/load_scene_visitor.h"
#include "core/view/framework/scene/scene_component_deserializer.h"
#include "core/view/utils/asset.h"

namespace imp {

// System for saving and loading scenes from .isf files.
//
// A .isf file is a NodeData protobuffer containing a declarative description of
// a scene. A scene includes a hierarchy of nodes, the transforms of those
// nodes, and the components of those nodes. Additionally, it also supports
// importing another .isf file as a child, and referencing nodes from
// components.
//
// See:
// third_party/impress/core/proto/imp_isf_data.bzl
// third_party/impress/core/ncsb/node_data.proto
class SceneSystem : public System {
 public:
  // Represents a unique id of a node within a .isf file.
  // Correlates exactly to the unique_id fields within the following file:
  // google3/third_party/impress/core/ncsb/node_data.proto
  using NodeUniqueId = int32_t;

  enum class MetadataMode {
    // Exclude metadata from the loaded scene.
    kExclude,
    // Include metadata from the loaded scene. This means that each loaded node
    // will have a SceneMetadata component that contains tracking information
    // about the how the node was loaded.
    //
    // This metadata is required to Save the scene back out using
    // SaveMode::kAuthoredContent. However, it comes at a
    // non-trivial memory cost, because it retains the original bytes for all
    // the base component protos among other data.
    //
    // In general, this is only needed if you are planning on saving the scene
    // back out using SaveMode::kAuthoredContent and is primarily needed for the
    // editor.
    kInclude
  };

  struct LoadSceneOptions {
    // If specified, then the loaded scene is parented to this node.
    // Otherwise, the loaded scene has no parent.
    NodeHandle parent;
    // LoadSceneVisitor can be used to modify & inspect the deserialized  state
    // of components during the loading process before the component's are
    // Setup. This makes it possible to override fields dynamically.
    LoadSceneVisitor load_scene_visitor;
    // Determines if the scene is loaded with or without metadata.
    MetadataMode metadata_mode;
  };

  enum class SaveMode {
    // This mode excludes data that isn't considered authored content.
    //
    // That means the following data is included:
    //   - Nodes created or modified directly by the editor
    //     - For example, glTF children are excluded unless they are modified by
    //       the editor.
    //   - Nodes originally loaded from a .isf file.
    //   - Data that is overidden from a base .isf file.
    //     - Data originating from a base .isf file is stripped out if it isn't
    //       modified.
    //
    // IMPORTANT NOTE: This mode requires that the scene was loaded with
    // MetadataMode::kInclude to function.
    kAuthoredContent,
    // Include all nodes.
    //
    // This ignores base .isf files and also includes all dynamically created
    // nodes.
    kFull
  };

  // Placeholder Url to use for loading NodeData with no specified url.
  static constexpr absl::string_view kRuntimeNodeDataPath =
      "runtime_node_proto";

  explicit SceneSystem(BaseView* view);

  // Loads an Impress Scene File (.isf) asynchronously and then parses the
  // contents into a node hierarchy.
  //
  // Setup is called on each component based on the dependency ordering
  // specified in the Savable for each component. See Savable::Dependency for
  // details.
  //
  // Setup is called by component type and not in order of the node hierarchy.
  // This means that all components of type 'A' on every node in the .isf will
  // be setup before any components of type 'B'. This allows us to support
  // dependencies where component A needs to be setup before component B even if
  // component B is on a different node in the .isf.
  Future<NodeHandle> LoadScene(const AssetDefinition& asset_definition,
                               LoadSceneOptions options = {});
  Future<NodeHandle> LoadScene(absl::string_view asset_url,
                               LoadSceneOptions options = {});
  Future<NodeHandle> LoadScene(const NodeData& node_data,
                               absl::string_view path = "",
                               LoadSceneOptions options = {});
  Future<NodeHandle> LoadSceneFromTextproto(resources::Resource resource,
                                            absl::string_view path = "",
                                            LoadSceneOptions options = {});

  ABSL_DEPRECATED("Use the LoadSceneOptions version instead.")
  Future<NodeHandle> LoadScene(const AssetDefinition& asset_definition,
                               NodeHandle parent);
  ABSL_DEPRECATED("Use the LoadSceneOptions version instead.")
  Future<NodeHandle> LoadScene(absl::string_view asset_url, NodeHandle parent);
  ABSL_DEPRECATED("Use the LoadSceneOptions version instead.")
  Future<NodeHandle> LoadScene(const NodeData& node_data,
                               absl::string_view path, NodeHandle parent);
  ABSL_DEPRECATED("Use the LoadSceneOptions version instead.")
  Future<NodeHandle> LoadScene(const AssetDefinition& asset_definition,
                               LoadSceneVisitor load_scene_visitor,
                               NodeHandle parent = NodeHandle());
  ABSL_DEPRECATED("Use the LoadSceneOptions version instead.")
  Future<NodeHandle> LoadScene(absl::string_view asset_url,
                               LoadSceneVisitor load_scene_visitor,
                               NodeHandle parent = NodeHandle());
  ABSL_DEPRECATED("Use the LoadSceneOptions version instead.")
  Future<NodeHandle> LoadScene(const NodeData& node_data,
                               LoadSceneVisitor load_scene_visitor,
                               absl::string_view path = "",
                               NodeHandle parent = NodeHandle());

  // Save a node to a binary string.
  absl::StatusOr<std::string> SaveToString(NodeHandle node, SaveMode mode);

  // Save a node to data.
  absl::StatusOr<NodeData> SaveToData(NodeHandle node, SaveMode mode);

  // Adds a component to the given node from an Any state proto data.
  // If the component already exists, it is removed before a new one is added.
  Future<absl::Status> AddComponent(
      NodeHandle node, const google::protobuf::imp_proto::Any& component_any);
  // Applies the component state data top of an existing component data.
  // Set fields will be updated and any unset fields remain at current values.
  Future<absl::Status> UpdateComponent(
      NodeHandle node, const google::protobuf::imp_proto::Any& component_any);

#if IMP_RUNTIME(DEV)
  // Save the node to json isf format.
  absl::StatusOr<std::string> SaveToJson(NodeHandle node, SaveMode mode);
#endif

  // Registers component types with the SceneSystem so that they can be
  // deserialized from a .isf file. This must be called for all component types
  // within a .isf file at some point before the .isf file is loaded via
  // SceneSystem::LoadScene.
  //
  // To successfully register a component type, the type must declare a using
  // statement for IsfInfo or StatelessIsfInfo. The IsfInfo is used to provide
  // information to the SceneSystem about how it should be deserialized.
  //
  // Ex:
  //   using IsfInfo = IsfInfo<&FooComponent::state_>;
  //   using IsfInfo = StatelessIsfInfo<FooComponent, kFooTypeUrl>;
  //
  // RegisterComponentIsfInfo can also take a flags parameter to
  // register optional features (i.e. saving). By default, the optional features
  // are enabled in dev mode but disabled otherwise.
  //
  // See:
  //   third_party/impress/core/view/framework/tests/scene_system_test.cc
  template <typename ComponentT,
            IsfOptionalFeature::Flags flags = IsfOptionalFeature::kAutomatic>
  void RegisterComponentIsfInfo();

  template <typename... ComponentTypes>
  ABSL_DEPRECATED("Use the RegisterComponentIsfInfo version instead.")
  void RegisterComponentsIsfInfo();

  // Registers default components with the SceneSystem so that they can be
  // deserialized without clients needing to manually register them. Called by
  // View automatically before View::Setup is called.
  void RegisterDefaultComponentsIsfInfo();

#if IMP_RUNTIME(DEV)
  std::unique_ptr<editor::Widget> CreateComponentWidget(
      HashValue component_state_type_url_hash, NodeHandle node,
      Dispatcher& editor_dispatcher);

  // Displays an ImGui UI for editing component state and adding that component.
  void ShowAddComponentUi(NodeHandle node);
#endif

 private:
  enum class SaveResult { kNoAuthoredContent, kIncludesAuthoredContent };

  Future<NodeHandle> LoadIntoNodeFromProto(NodeHandle node,
                                           const NodeData& data,
                                           absl::string_view asset_url,
                                           LoadSceneVisitor* load_scene_visitor,
                                           MetadataMode metadata_mode);

  // Struct of temporary data used during the process of loading a scene
  // and then discarded afterwards.
  struct LoadSceneInfo {
    // Holds a node and the data required to add a component to that node
    // for a specific component type.
    struct ComponentInfo {
      // Bytes for the components state proto. Empty if it's a stateless
      // component. There can be multiple binary protos for a single node
      // because a node can inherit from an external .isf file. This vector is
      // used to merge the binary protos together.
      std::vector<absl::string_view> component_data;

      // If the component was added in the .isf using the
      // components_with_metadata field, then this field will contain all the
      // metadata. The component_data field should still be used to add the
      // component instead of the Any within the metadata, because the
      // component_data field already handled extracting the Stateless
      // component.
      const ComponentWithMetadata* component_with_metadata = nullptr;
    };

    using NodeToComponentInfo = RobinMap<NodeHandle, ComponentInfo>;

    using HandlerHashToNodesAndComponentData =
        RobinMap<HashValue, NodeToComponentInfo>;

    using UniqueIdsToNodes = RobinMap<int32_t, NodeHandle>;
    using NamesToNodes = RobinMap<std::string, NodeHandle>;

    // Maps the unique ids from nodes within a .isf file to the actual created
    // nodes. Used to assign NodeSceneHandle references to the actual nodes.
    UniqueIdsToNodes ids_to_nodes;
    // Maps names from nodes within a .isf file to the actual created nodes.
    // Used to assign NodeSceneHandle references to the actual nodes.
    NamesToNodes names_to_nodes;

    // Maps the hash of a Savable::Handler to a list of nodes and the serialized
    // data for the savable component. Used to build a dependency graph of
    // components in the .isf file to deserialize and then Setup the components
    // on the nodes in the correct order. This is also used to defer the
    // deserialization of the components until after all nodes in the node
    // hierarchy have been created so that NodeSceneHandle's can be hooked up to
    // reference other nodes within the scene.
    HandlerHashToNodesAndComponentData handler_hash_to_node_and_comp_data;

    // Store the ProtoAsset<NodeData> in LoadSceneInfo to guarantee that the
    // memory for the asset stays alive until the load has completely finished.
    std::vector<AssetPtr<ProtoAsset<NodeData>>> assets;

    LoadSceneVisitor* load_scene_visitor;

    // Tracks if the root node is disabled by the .isf file. We need to track
    // this so that we can keep the root node disabled while everything is
    // loading and only enable it at the end. We track this within LoadSceneInfo
    // so that we can merge together the disabled setting from base .isf files.
    bool is_root_disabled = false;

    // Determines if the nodes are loaded with or without metadata.
    MetadataMode metadata_mode;
  };

  // Helper containing the data from a component's proto Any.
  // this is used to unify Stateless components and components that have their
  // own state into one data structure.
  struct ExtractedComponentAny {
    // Human readable string identifying the type of component.
    // This is the proto's type url for components that have a state proto.
    // Otherwise, it's the string passed into StatelessIsfInfo.
    std::string type_url;
    // Hash for the type of component used to access the handler.
    HashValue type;
    // Bytes for the components state proto. Empty if it's a stateless
    // component.
    absl::string_view component_data;
  };

  // Extracts the real data for parsing a component from the raw proto Any if
  // it's a stateless component or a component that has its own state proto.
  // Also performs some data validation on the types.
  absl::StatusOr<ExtractedComponentAny> VerifyAndExtractComponentAny(
      const google::protobuf::imp_proto::Any& component_any);

  Future<absl::Status> CreateSceneHierarchy(
      const NodeData& data, NodeHandle node, LoadSceneInfo* load_scene_info,
      LoadSceneVisitor* load_scene_visitor, bool is_base);

  Future<absl::Status> ApplyData(const NodeData& data, NodeHandle node,
                                 LoadSceneInfo* load_scene_info,
                                 LoadSceneVisitor* load_scene_visitor,
                                 bool is_base = false);

  // Sets up all the components in the .isf file based on the dependency
  // order of the components, allowing parallel execution of async work.
  Future<absl::Status> SetupComponents(LoadSceneInfo* load_scene_info);

  absl::StatusOr<SaveResult> SaveImpl(NodeHandle node, NodeData* data,
                                      SaveMode mode, bool is_root_node);

  SceneComponentDeserializer scene_component_deserializer_;
};

template <typename ComponentT, IsfOptionalFeature::Flags flags>
void SceneSystem::RegisterComponentIsfInfo() {
  scene_component_deserializer_.RegisterComponentIsfInfo<ComponentT, flags>();
}

template <typename... ComponentTypes>
void SceneSystem::RegisterComponentsIsfInfo() {
  (scene_component_deserializer_.RegisterComponentIsfInfo<
       ComponentTypes, IsfOptionalFeature::kAutomatic>(),
   ...);
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_SCENE_SCENE_SYSTEM_H_
