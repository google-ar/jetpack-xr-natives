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

#include "core/view/framework/scene/scene_component_deserializer.h"

#include "absl/container/btree_map.h"
#include "absl/container/flat_hash_set.h"
#include "core/common/hash.h"
#include "core/config.h"

namespace imp {

#if IMP_RUNTIME(DEV)
SceneComponentDeserializer::SceneComponentDeserializer()
    : add_component_ui_("add component...") {
  proto::TextprotoReader::RegisterKnownType<StatelessComponent>();
  proto::TextprotoWriter::RegisterKnownType<StatelessComponent>();
}
#else
SceneComponentDeserializer::SceneComponentDeserializer() = default;
#endif

const SceneComponentDeserializer::Handler*
SceneComponentDeserializer::GetHandler(HashValue type_hash) const {
  auto iter = handlers_.find(type_hash);
  if (iter != handlers_.end()) {
    return &iter->second;
  }
  return nullptr;
}

// The hash is based on component type URL and guaranteed to be consistent.
const absl::btree_map<HashValue, SceneComponentDeserializer::Handler>&
SceneComponentDeserializer::GetHandlers() const {
  return handlers_;
}

const absl::flat_hash_set<HashValue>& SceneComponentDeserializer::GetDeps(
    HashValue type_hash) {
  return deps_[type_hash];
}

#if IMP_RUNTIME(DEV)
void SceneComponentDeserializer::ShowAddComponentUi(
    const absl::flat_hash_set<NodeHandle>& nodes) {
  selected_nodes_ = nodes;
  if (selected_nodes_.empty()) return;
  command_manager_ = &(*selected_nodes_.begin())
                          ->GetView()
                          .GetRegistry()
                          .GetOrCreate<editor::CommandManager>();
  add_component_ui_.DrawImGui();
}

std::unique_ptr<editor::Widget>
SceneComponentDeserializer::CreateComponentWidget(
    HashValue component_state_type_url_hash,
    absl::flat_hash_set<NodeHandle> nodes, Dispatcher& editor_dispatcher) {
  const Handler* handler = GetHandler(component_state_type_url_hash);
  if (handler == nullptr) {
    return {};
  }

  if (handler->create_component_widget == nullptr) {
    return {};
  }

  return handler->create_component_widget(std::move(nodes), editor_dispatcher);
}
#endif

}  // namespace imp
