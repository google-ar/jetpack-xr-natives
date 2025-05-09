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

#ifndef THIRD_PARTY_IMPRESS_CORE_NCSB_BASE_SCENE_HANDLE_H_
#define THIRD_PARTY_IMPRESS_CORE_NCSB_BASE_SCENE_HANDLE_H_

#include <cstdint>
#include <string>

#include "absl/status/status.h"
#include "absl/types/variant.h"
#include "core/common/hash.h"
#include "core/common/type_traits.h"
#include "core/ncsb/node_handle.h"

namespace imp {

// Pure virtual interface for defining a SceneHandle. SceneHandle's are used to
// hook up references to nodes & components within a .isf file. For each type of
// SceneHandle, there is a proto message and a corresponding native type.
struct SceneHandleInterface {
  // Identifier that maps to either a unique_id or a name from NodeData or
  // NodeReferenceData within the NodeData proto message.
  //
  // The identifier maps to a node within a .isf file, which is then used to
  // assign this scene handle.
  //
  // See the NodeData proto:
  //   depot/google3/third_party/impress/core/ncsb/node_data.proto
  using Identifier = absl::variant<absl::monostate, std::string, int32_t>;

  // Defines a TypeUrlHash shared by implementors of SceneHandleInterface used
  // to visit them during deserialization and assign the scene handle. This
  // allows scene handles to be visited in a type-erased way which is necessary
  // since we don't know the concrete type of the handle during deserialization.
  static constexpr imp::HashValue kTypeUrlHash =
      type_traits::kTypeHash<SceneHandleInterface>;

  virtual ~SceneHandleInterface() = default;

  // Called by SceneSystem when loading a .isf if the scene handle has a valid
  // assigned identifier. The node for that identifier is passed in. This is
  // used to map the scene handle to the actual node/component when
  // deserialization occurs.
  //
  // Returns a status indicating if assigning succeeded or failed.
  virtual absl::Status AssignSceneHandleForIdentifier(
      NodeHandle identified_node, NodeHandle attached_node) = 0;

  // Called by the editor when assigning the scene handle to a node via drag &
  // drop. Updates the identifier to match the node being assigned.
  //
  // This is different from AssignSceneHandleForIdentifier, which is used to map
  // the identifiers to real nodes when loading a .isf file.
  virtual void AssignSceneHandleForNode(NodeHandle node) = 0;

  // Returns true if the scene handle can be assigned to the given node.
  virtual bool CanAssignSceneHandleForNode(NodeHandle node) const = 0;

  // Returns this scene handles identifier.
  //
  // This will also update the identifier if necessary if the scene handle is
  // referencing a node whose name or SceneIdentifier has changed.
  virtual const Identifier& GetIdentifier() = 0;

  // Returns a string that represents debugging information about the scene
  // handle's string.
  virtual std::string ToSceneHandleString() const = 0;

  // Returns the type that this scene handle represents (i.e. "Node", "glTF
  // Node").
  //
  // this is used for debugging and for displaying the type in the editor.
  virtual std::string GetTypeName() const = 0;

  // Returns the node that this scene handle is referencing.
  virtual NodeHandle GetSceneNode() const = 0;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_BASE_SCENE_HANDLE_H_
