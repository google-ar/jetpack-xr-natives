/*
 * Copyright 2026 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_MARSHALLING_TEST_NODE_MANAGER_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_MARSHALLING_TEST_NODE_MANAGER_H_

#include <cstdint>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "apibindings/impress_api_view.h"
#include "apibindings/node_manager.h"
#include "core/math/transform.h"

namespace imp {

// Inherits from the real NodeManager for testing purposes.
class TestNodeManager : public NodeManager {
 public:
  explicit TestNodeManager(ImpressApiView& view);
  ~TestNodeManager() override = default;

  int32_t CreateImpressNode() override;
  absl::Status DestroyImpressNode(int32_t node) override;
  absl::Status SetImpressNodeParent(int32_t child, int32_t parent) override;
  absl::StatusOr<int32_t> GetImpressNodeParent(int32_t node_id) override;
  absl::StatusOr<int32_t> GetImpressNodeChildCount(int32_t node_id) override;
  absl::StatusOr<int32_t> GetImpressNodeChildAt(int32_t node_id,
                                                int32_t index) override;
  absl::StatusOr<absl::string_view> GetImpressNodeName(
      int32_t node_id) override;
  absl::StatusOr<imp::Transform<float>> GetImpressNodeLocalTransform(
      int32_t node_id) override;
  absl::Status SetImpressNodeLocalTransform(
      int32_t node_id, const imp::Transform<float>& transform) override;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_MARSHALLING_TEST_NODE_MANAGER_H_
