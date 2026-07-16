/*
 * Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_SERIALIZER_BATCH_MANAGER_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_SERIALIZER_BATCH_MANAGER_H_

#include <array>
#include <memory>
#include <queue>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/base/nullability.h"
#include "absl/container/flat_hash_map.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "core/common/invocable.h"
#include "core/split_engine/split_engine_serializer_data_types.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"

namespace imp::split_engine {

// System to combine multiple serialization commands into batches to reduce the
// number of messages sent to the renderer.
// This specific implementation provides the option to define entities and
// resources as dependencies for each command. It batches commands into the
// least number of batches possible while respecting the dependency chains
// between commands.
// Commands that remove resources are handled separately from other commands,
// and are serialized at the end of the frame in a predefined order, ignoring
// any dependencies.
class SplitEngineSerializerBatchManager {
 public:
  SplitEngineSerializerBatchManager();

  // Returns either an existing batch that keeps dependent commands in order, or
  // a new batch if no such batch exists.
  template <android_xr::schemas::CommandTypes CommandT>
  SerializerDataTypes::Batch<CommandT>& GetOrCreateBatch(
      const std::vector<utils::Entity>& entity_dependencies = {},
      const std::vector<SerializerDataTypes::ResourceId>&
          resource_dependencies = {}) {
    // Handle commands that are executed at the end of frame separately.
    if (absl::c_linear_search(kRemoveResourceCommandOrder, CommandT)) {
      if (!end_of_frame_batches_.contains(CommandT)) {
        // The batch index is not used for end of frame batches so we pass 0.
        end_of_frame_batches_[CommandT] =
            std::make_unique<SerializerDataTypes::Batch<CommandT>>(0);
      }
      return *static_cast<SerializerDataTypes::Batch<CommandT>*>(
          end_of_frame_batches_[CommandT].get());
    }

    SerializerDataTypes::CommandBatchBase* batch =
        FindBatch(CommandT, entity_dependencies, resource_dependencies);

    if (batch == nullptr) {
      // The position of the batch in the queue is its index. This is required
      // to keep dependent batches in order.
      batch = AddBatch(std::make_unique<SerializerDataTypes::Batch<CommandT>>(
                           batch_queue_.size()),
                       entity_dependencies, resource_dependencies);
    }

    return *static_cast<SerializerDataTypes::Batch<CommandT>*>(batch);
  }

  // Calls the given function for each batch in the queue.
  // This method resets the manager by clearing all batches from the queue.
  void ForEachBatchRunAndConsume(
      imp::Invocable<void(SerializerDataTypes::CommandBatchBase*)> func);

 private:
  // When a command is added to a batch, we use this helper method to store that
  // this is the last batch that affected the given entities and resources.
  void StoreAffectedDependenciesBatch(
      const std::vector<utils::Entity>& entity_dependencies,
      const std::vector<SerializerDataTypes::ResourceId>& resource_dependencies,
      SerializerDataTypes::CommandBatchBase* /*absl_nonnull*/  batch);

  // Returns the first batch of the given type that runs after the last batch
  // that affected the given dependencies. Returns nullptr if no such batch
  // exists.
  SerializerDataTypes::CommandBatchBase* /*absl_nullable*/  FindBatch(
      android_xr::schemas::CommandTypes command_type,
      const std::vector<utils::Entity>& entity_dependencies = {},
      const std::vector<SerializerDataTypes::ResourceId>&
          resource_dependencies = {});

  // Adds the given batch to the queue and returns a pointer to it.
  SerializerDataTypes::CommandBatchBase* /*absl_nonnull*/  AddBatch(
      /*absl_nonnull*/  std::unique_ptr<SerializerDataTypes::CommandBatchBase> batch,
      const std::vector<utils::Entity>& entity_dependencies = {},
      const std::vector<SerializerDataTypes::ResourceId>&
          resource_dependencies = {});

  void Reset();

  // Stores batches of commands in the order they were created. This ensures
  // that commands are executed in the general order intended by the app.
  // NOTE: In theory, we don't need to execute commands in the order they came
  // in, as long as we run commands affecting the same dependency in the correct
  // order. However, running commands in this order makes the implementation
  // deterministic.
  std::queue<
      /*absl_nonnull*/  std::unique_ptr<SerializerDataTypes::CommandBatchBase>>
      batch_queue_;

  // Queue of batches that are executed at the end of the frame. This is used
  // for removing resources, as it is unsafe to remove resources while they are
  // potentially still in use.
  static constexpr std::array<android_xr::schemas::CommandTypes, 6>
      kRemoveResourceCommandOrder = {
          android_xr::schemas::CommandTypes::RemoveMorphTargetBuffers,
          android_xr::schemas::CommandTypes::RemoveMeshData,
          android_xr::schemas::CommandTypes::RemoveMaterialInstances,
          android_xr::schemas::CommandTypes::RemoveMaterials,
          android_xr::schemas::CommandTypes::RemoveImageBasedLightingAssets,
          android_xr::schemas::CommandTypes::RemoveTextures,
  };
  absl::flat_hash_map<android_xr::schemas::CommandTypes,
                      std::unique_ptr<SerializerDataTypes::CommandBatchBase>>
      end_of_frame_batches_;

  // This data structure allows for quick lookup of batches by type.
  absl::flat_hash_map<
      android_xr::schemas::CommandTypes,
      std::vector<SerializerDataTypes::CommandBatchBase* /*absl_nonnull*/ >>
      batches_;

  // Stores a pointer to the last batch that affected a given entity.
  // This is used to ensure the order of dependent commands.
  SerializerDataTypes::EntityMap<SerializerDataTypes::CommandBatchBase*>
      last_batch_affecting_entity_;

  // We need to store resource IDs separately, because they do not use the same
  // ID system as entities. While ID collisions do not break our algorithm, they
  // do lead to the creation of more batches than necessary.
  absl::flat_hash_map<SerializerDataTypes::ResourceId,
                      SerializerDataTypes::CommandBatchBase*>
      last_batch_affecting_resource_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_SERIALIZER_BATCH_MANAGER_H_
