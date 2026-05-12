// Copyright 2026 Google LLC
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

#include "apibindings/testing/marshalling/test_node_manager.h"

#include <cstdint>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "apibindings/impress_api_view.h"
#include "core/math/transform.h"

namespace imp {

TestNodeManager::TestNodeManager(ImpressApiView& view) {}

int32_t TestNodeManager::CreateImpressNode() {
  IMP_LOG(imp::FATAL) << "TestNodeManager::CreateImpressNode unimplemented";
  return -1;
}

absl::Status TestNodeManager::DestroyImpressNode(int32_t node) {
  return absl::UnimplementedError(
      "TestNodeManager::DestroyImpressNode unimplemented");
}

absl::Status TestNodeManager::SetImpressNodeParent(int32_t child,
                                                   int32_t parent) {
  return absl::UnimplementedError(
      "TestNodeManager::SetImpressNodeParent unimplemented");
}

absl::StatusOr<int32_t> TestNodeManager::GetImpressNodeParent(int32_t node_id) {
  return absl::UnimplementedError(
      "TestNodeManager::GetImpressNodeParent unimplemented");
}

absl::StatusOr<int32_t> TestNodeManager::GetImpressNodeChildCount(
    int32_t node_id) {
  return absl::UnimplementedError(
      "TestNodeManager::GetImpressNodeChildCount unimplemented");
}

absl::StatusOr<int32_t> TestNodeManager::GetImpressNodeChildAt(int32_t node_id,
                                                               int32_t index) {
  return absl::UnimplementedError(
      "TestNodeManager::GetImpressNodeChildAt unimplemented");
}

absl::StatusOr<absl::string_view> TestNodeManager::GetImpressNodeName(
    int32_t node_id) {
  return absl::UnimplementedError(
      "TestNodeManager::GetImpressNodeName unimplemented");
}

absl::StatusOr<imp::Transform<float>>
TestNodeManager::GetImpressNodeLocalTransform(int32_t node_id) {
  return absl::UnimplementedError(
      "TestNodeManager::GetImpressNodeLocalTransform unimplemented");
}

absl::Status TestNodeManager::SetImpressNodeLocalTransform(
    int32_t node_id, const imp::Transform<float>& transform) {
  return absl::UnimplementedError(
      "TestNodeManager::SetImpressNodeLocalTransform unimplemented");
}

}  // namespace imp
