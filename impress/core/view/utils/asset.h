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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_UTILS_ASSET_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_UTILS_ASSET_H_

#include "core/resources/resource_manager.h"

namespace imp {

// Defines an asset to be loaded by the asset manager.  Embedded resources
// should be generated using the imp_assets rule in
// third_party/impress/core/resources/resources.bzl but network-only
// assets can be created directly with the AssetDefinition url constructor.
//
// AssetDefinition will always be an alias to a resource definition and clients
// may assume this in their implementations.
//
// Example:
// AssetDefinition remote_glb("https://www.google.com/model.glb");
// asset_manager.LoadModel(remote_glb, [](NodeHandle node_handle) {});
using AssetDefinition = imp::resources::ResourceDefinition;

// Defines a stable identifier that is associated with a particular defined
// asset.
// AssetId may change from std::string to a different type and users should
// only assume that it is a type hashable with std::hash and comparable with
// std::equal_to.
using AssetId = std::string;

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_UTILS_ASSET_H_
