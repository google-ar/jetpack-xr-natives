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
#include "core/split_engine/split_engine_serializer_batch_manager.h"

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include "absl/base/nullability.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "core/common/invocable.h"
#include "core/split_engine/split_engine_serializer_data_types.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"

namespace imp::split_engine {

using android_xr::schemas::CommandTypes;

SplitEngineSerializerBatchManager::SplitEngineSerializerBatchManager() {}

void SplitEngineSerializerBatchManager::ForEachBatchRunAndConsume(
    imp::Invocable<void(SerializerDataTypes::CommandBatchBase*)> func) {
  // First go through the queued batches.
  while (!batch_queue_.empty()) {
    std::unique_ptr<SerializerDataTypes::CommandBatchBase> batch =
        std::move(batch_queue_.front());
    batch_queue_.pop();
    func(batch.get());
  }

  // Then go through the end of frame batches.
  for (auto command_type : kRemoveResourceCommandOrder) {
    if (end_of_frame_batches_.contains(command_type)) {
      func(end_of_frame_batches_[command_type].get());
      end_of_frame_batches_.erase(command_type);
    }
  }

  Reset();
}

void SplitEngineSerializerBatchManager::Reset() {
  batch_queue_ = {};
  end_of_frame_batches_ = {};
  batches_.clear();
  last_batch_affecting_entity_.clear();
  last_batch_affecting_resource_.clear();
}

void SplitEngineSerializerBatchManager::StoreAffectedDependenciesBatch(
    const std::vector<utils::Entity>& entity_dependencies,
    const std::vector<SerializerDataTypes::ResourceId>& resource_dependencies,
    SerializerDataTypes::CommandBatchBase* /*absl_nonnull*/  batch) {
  // Store for each entity and resource that the provided CommandBatch contains
  // the last command that affected this entity or resource.
  // The next command that affects the same entity or resource will have to be
  // executed in a later batch.
  for (const utils::Entity& entity : entity_dependencies) {
    last_batch_affecting_entity_[entity] = batch;
  }
  for (const auto& resource_id : resource_dependencies) {
    last_batch_affecting_resource_[resource_id] = batch;
  }
}

SerializerDataTypes::CommandBatchBase* /*absl_nullable*/ 
SplitEngineSerializerBatchManager::FindBatch(
    CommandTypes command_type,
    const std::vector<utils::Entity>& entity_dependencies,
    const std::vector<SerializerDataTypes::ResourceId>& resource_dependencies) {
  // For each dependency, find the last batch that affected it. If no prior
  // batch affected this dependency, we can add this command to the first batch
  // we find that has the same type.
  const SerializerDataTypes::CommandBatchBase* /*absl_nullable*/  last_batch =
      nullptr;
  for (const auto& dependency : entity_dependencies) {
    auto it = last_batch_affecting_entity_.find(dependency);
    if (it != last_batch_affecting_entity_.end()) {
      if (last_batch == nullptr || it->second->index > last_batch->index) {
        last_batch = it->second;
      }
    }
  }
  for (const auto& dependency : resource_dependencies) {
    auto it = last_batch_affecting_resource_.find(dependency);
    if (it != last_batch_affecting_resource_.end()) {
      if (last_batch == nullptr || it->second->index > last_batch->index) {
        last_batch = it->second;
      }
    }
  }

  // Go through batches of the same command type.
  auto it = batches_.find(command_type);
  if (it != batches_.end()) {
    const std::vector<SerializerDataTypes::CommandBatchBase*>& batches =
        it->second;
    // Find the first batch that does not disrupt the dependency chain (meaning
    // it is executed after the last batch that affected the dependencies).
    // Since batches are sorted by index, we use binary search to find the
    // last_batch that affected the provided dependencies.
    // NOTE: An argument could be made that we should use upper_bound instead of
    // lower_bound here. The difference is that if we allow for lower_bound,
    // and the last batch is of the same type, we just append the current
    // command to the batch. This keeps correct order if the data is stored in
    // a format that by itself keeps order, like a vector, but order may be
    // broken if the data structure does not keep order, like a map. In this
    // case, using upper_bound would create a new batch even if the last batch
    // is of the same type and enforce order independent of the data type.
    // We could also add a boolean to each command type that indicates if the
    // data is stored in order or not and allow for the last batch to be used
    // or not based on this.
    auto batch_it = batches.begin();
    if (last_batch != nullptr) {
      batch_it = std::lower_bound(
          batches.begin(), batches.end(), last_batch,
          [](const auto* a, const auto* b) { return a->index < b->index; });
    }

    if (batch_it != batches.end()) {
      StoreAffectedDependenciesBatch(entity_dependencies, resource_dependencies,
                                     *batch_it);
      return *batch_it;
    }
  }

  return nullptr;
}

SerializerDataTypes::CommandBatchBase* /*absl_nonnull*/ 
SplitEngineSerializerBatchManager::AddBatch(
    std::unique_ptr<SerializerDataTypes::CommandBatchBase> batch,
    const std::vector<utils::Entity>& entity_dependencies,
    const std::vector<SerializerDataTypes::ResourceId>& resource_dependencies) {
  batch_queue_.push(std::move(batch));
  SerializerDataTypes::CommandBatchBase* batch_ptr = batch_queue_.back().get();
  batches_[batch_ptr->type].push_back(batch_ptr);
  StoreAffectedDependenciesBatch(entity_dependencies, resource_dependencies,
                                 batch_ptr);
  return batch_ptr;
}

}  // namespace imp::split_engine
