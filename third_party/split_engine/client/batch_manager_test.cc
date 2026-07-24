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

#include "split_engine/client/batch_manager.h"

#include <vector>

#include "gtest/gtest.h"
#include "mediapipe/framework/port/status_matchers.h"
#include "split_engine/client/batch.h"
#include "split_engine/client/types.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"

namespace split_engine {
namespace {

using android_xr::schemas::CommandTypes;

class SplitEngineSerializerBatchManagerTest : public ::testing::Test {
 protected:
  struct BatchInfo {
    CommandTypes type;
  };
  std::vector<BatchInfo> GetBatches() {
    std::vector<BatchInfo> result;
    batch_manager_.ForEachBatchRunAndConsume(
        [&](BatchBase& batch) { result.push_back({batch.type}); });
    return result;
  }

  SplitEngineSerializerBatchManager batch_manager_;
};

TEST_F(SplitEngineSerializerBatchManagerTest,
       CommandsOfSameTypeWithoutDependencyAreBatched) {
  auto& batch1 = batch_manager_.GetOrCreateBatch<CommandTypes::AddNodes>();
  auto& batch2 = batch_manager_.GetOrCreateBatch<CommandTypes::AddNodes>();

  EXPECT_EQ(&batch1, &batch2);

  auto batches = GetBatches();
  EXPECT_EQ(batches.size(), 1);
  EXPECT_EQ(batches[0].type, CommandTypes::AddNodes);
}

TEST_F(SplitEngineSerializerBatchManagerTest,
       CommandsOfSameTypeWithInterleavingCommandAreBatched) {
  auto& batch_a1 = batch_manager_.GetOrCreateBatch<CommandTypes::AddNodes>();
  batch_manager_.GetOrCreateBatch<CommandTypes::UpdateNodes>();
  auto& batch_a2 = batch_manager_.GetOrCreateBatch<CommandTypes::AddNodes>();

  EXPECT_EQ(&batch_a1, &batch_a2);

  auto batches = GetBatches();
  EXPECT_EQ(batches.size(), 2);
  EXPECT_EQ(batches[0].type, CommandTypes::AddNodes);
  EXPECT_EQ(batches[1].type, CommandTypes::UpdateNodes);
}

// NOTE: This test should fail when the data structure for the batch does not
// keep order.
TEST_F(SplitEngineSerializerBatchManagerTest,
       CommandsOfSameTypeWithSameDependencyAreBatched) {
  EntityId entity = 0;
  auto& batch_a1 =
      batch_manager_.GetOrCreateBatch<CommandTypes::AddNodes>({entity});
  batch_manager_.GetOrCreateBatch<CommandTypes::UpdateNodes>();
  auto& batch_a2 =
      batch_manager_.GetOrCreateBatch<CommandTypes::AddNodes>({entity});

  EXPECT_EQ(&batch_a1, &batch_a2);

  auto batches = GetBatches();
  EXPECT_EQ(batches.size(), 2);
  EXPECT_EQ(batches[0].type, CommandTypes::AddNodes);
  EXPECT_EQ(batches[1].type, CommandTypes::UpdateNodes);
}

TEST_F(SplitEngineSerializerBatchManagerTest,
       CommandsOfSameTypeWithDifferentDependencyAreBatched) {
  EntityId entity1 = 0;
  EntityId entity2 = 1;
  auto& batch_a1 =
      batch_manager_.GetOrCreateBatch<CommandTypes::AddNodes>({entity1});
  batch_manager_.GetOrCreateBatch<CommandTypes::UpdateNodes>();
  auto& batch_a2 =
      batch_manager_.GetOrCreateBatch<CommandTypes::AddNodes>({entity2});

  EXPECT_EQ(&batch_a1, &batch_a2);

  auto batches = GetBatches();
  EXPECT_EQ(batches.size(), 2);
  EXPECT_EQ(batches[0].type, CommandTypes::AddNodes);
  EXPECT_EQ(batches[1].type, CommandTypes::UpdateNodes);
}

TEST_F(SplitEngineSerializerBatchManagerTest,
       InterleavingCommandWithSameEntityDependencyPreventsBatching) {
  EntityId entity = 0;

  auto& batch_a1 = batch_manager_.GetOrCreateBatch<CommandTypes::AddNodes>();
  batch_manager_.GetOrCreateBatch<CommandTypes::UpdateNodes>({entity});
  auto& batch_a2 =
      batch_manager_.GetOrCreateBatch<CommandTypes::AddNodes>({entity});

  EXPECT_NE(&batch_a1, &batch_a2);

  auto batches = GetBatches();
  EXPECT_EQ(batches.size(), 3);
  EXPECT_EQ(batches[0].type, CommandTypes::AddNodes);
  EXPECT_EQ(batches[1].type, CommandTypes::UpdateNodes);
  EXPECT_EQ(batches[2].type, CommandTypes::AddNodes);
}

TEST_F(SplitEngineSerializerBatchManagerTest,
       InterleavingCommandWithSameResourceDependencyPreventsBatching) {
  ResourceId resource{1};

  auto& batch_a1 =
      batch_manager_.GetOrCreateBatch<CommandTypes::AddNodes>({}, {});
  batch_manager_.GetOrCreateBatch<CommandTypes::UpdateNodes>({}, {resource});
  auto& batch_a2 =
      batch_manager_.GetOrCreateBatch<CommandTypes::AddNodes>({}, {resource});

  EXPECT_NE(&batch_a1, &batch_a2);

  auto batches = GetBatches();
  EXPECT_EQ(batches.size(), 3);
  EXPECT_EQ(batches[0].type, CommandTypes::AddNodes);
  EXPECT_EQ(batches[1].type, CommandTypes::UpdateNodes);
  EXPECT_EQ(batches[2].type, CommandTypes::AddNodes);
}

TEST_F(SplitEngineSerializerBatchManagerTest,
       InterleavingCommandWithDifferentDependencyDoesNotPreventBatching) {
  EntityId entity1 = 0;
  EntityId entity2 = 1;

  auto& batch_a1 =
      batch_manager_.GetOrCreateBatch<CommandTypes::AddNodes>({entity1}, {});
  batch_manager_.GetOrCreateBatch<CommandTypes::UpdateNodes>({entity2}, {});
  auto& batch_a2 =
      batch_manager_.GetOrCreateBatch<CommandTypes::AddNodes>({entity1}, {});

  EXPECT_EQ(&batch_a1, &batch_a2);

  auto batches = GetBatches();
  EXPECT_EQ(batches.size(), 2);
  EXPECT_EQ(batches[0].type, CommandTypes::AddNodes);
  EXPECT_EQ(batches[1].type, CommandTypes::UpdateNodes);
}

TEST_F(SplitEngineSerializerBatchManagerTest, BatchExecutionOrderIsCorrect) {
  EntityId entity = 0;
  batch_manager_.GetOrCreateBatch<CommandTypes::AddNodes>({entity}, {});
  batch_manager_.GetOrCreateBatch<CommandTypes::UpdateNodes>({entity}, {});
  batch_manager_.GetOrCreateBatch<CommandTypes::RemoveNodes>({entity}, {});

  auto batches = GetBatches();
  ASSERT_EQ(batches.size(), 3);
  EXPECT_EQ(batches[0].type, CommandTypes::AddNodes);
  EXPECT_EQ(batches[1].type, CommandTypes::UpdateNodes);
  EXPECT_EQ(batches[2].type, CommandTypes::RemoveNodes);
}

TEST_F(SplitEngineSerializerBatchManagerTest,
       EndOfFrameCommandsAreExecutedLastAndInOrder) {
  batch_manager_.GetOrCreateBatch<CommandTypes::RemoveTextures>();
  batch_manager_
      .GetOrCreateBatch<CommandTypes::RemoveImageBasedLightingAssets>();
  batch_manager_.GetOrCreateBatch<CommandTypes::RemoveMaterials>();
  batch_manager_.GetOrCreateBatch<CommandTypes::RemoveMaterialInstances>();
  batch_manager_.GetOrCreateBatch<CommandTypes::RemoveMeshData>();
  batch_manager_.GetOrCreateBatch<CommandTypes::RemoveMorphTargetBuffers>();

  // Non-remove commands should execute first even if sent last.
  batch_manager_.GetOrCreateBatch<CommandTypes::AddNodes>();
  batch_manager_.GetOrCreateBatch<CommandTypes::UpdateNodes>();

  auto batches = GetBatches();
  ASSERT_EQ(batches.size(), 8);
  EXPECT_EQ(batches[0].type, CommandTypes::AddNodes);
  EXPECT_EQ(batches[1].type, CommandTypes::UpdateNodes);
  EXPECT_EQ(batches[2].type, CommandTypes::RemoveMorphTargetBuffers);
  EXPECT_EQ(batches[3].type, CommandTypes::RemoveMeshData);
  EXPECT_EQ(batches[4].type, CommandTypes::RemoveMaterialInstances);
  EXPECT_EQ(batches[5].type, CommandTypes::RemoveMaterials);
  EXPECT_EQ(batches[6].type, CommandTypes::RemoveImageBasedLightingAssets);
  EXPECT_EQ(batches[7].type, CommandTypes::RemoveTextures);
}

}  // namespace
}  // namespace split_engine
