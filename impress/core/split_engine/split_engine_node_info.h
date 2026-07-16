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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_INPUTS_SPLIT_ENGINE_NODE_INFO_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_INPUTS_SPLIT_ENGINE_NODE_INFO_H_

#include <cstdint>

#include "core/config.h"
#include "core/ncsb/component.h"
#if IMP_RUNTIME(DEV)
#include "dear_imgui/imgui.h"
#endif  // IMP_RUNTIME(DEV)

namespace imp::split_engine {

// Class to store node specific data from the split engine front end instance.
class SplitEngineNodeInfo : public Component {
 public:
  void Setup(uint32_t front_end_entity_id) {
    front_end_entity_id_ = front_end_entity_id;
  };
  uint32_t GetFrontEndEntityId() { return front_end_entity_id_; };

#if IMP_RUNTIME(DEV)
  void DrawEditorUi() {
    if (ImGui::BeginTable("##NodeInfoTable", 2,
                          ImGuiTableFlags_Resizable |
                              ImGuiTableFlags_BordersInnerV |
                              ImGuiTableFlags_SizingStretchProp)) {
      ImGui::TableNextRow();
      ImGui::TableNextColumn();
      ImGui::TextUnformatted("Front End Entity ID");
      ImGui::TableNextColumn();
      ImGui::Text("%u", front_end_entity_id_);
      ImGui::EndTable();
    }
  }
#endif  // IMP_RUNTIME(DEV)

 private:
  uint32_t front_end_entity_id_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_INPUTS_SPLIT_ENGINE_NODE_INFO_H_
