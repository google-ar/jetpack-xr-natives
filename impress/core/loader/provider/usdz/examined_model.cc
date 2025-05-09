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

#include "core/loader/provider/usdz/examined_model.h"

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "core/common/typed_id.h"
#include "third_party/tinyusdz/src/prim-types.hh"
#include "third_party/tinyusdz/src/stage.hh"
#include "third_party/tinyusdz/src/usdGeom.hh"
#include "third_party/tinyusdz/src/usdShade.hh"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::loader::details::provider_usdz {

absl::StatusOr<ExaminedModel> ExaminedModel::FromStage(
    const tinyusdz::Stage &stage) {
  ExaminedModel result;
  for (const auto &root_prim : stage.root_prims()) {
    MP_RETURN_IF_ERROR(ExaminePrim(root_prim, result));
  }
  return result;
}

absl::Status ExaminedModel::ExaminePrim(const tinyusdz::Prim &prim,
                                        ExaminedModel &model,
                                        WeakBoneId parent_bone,
                                        WeakEntityId parent_entity) {
  WeakBoneId bone;
  WeakEntityId entity;

  if (prim.is<tinyusdz::Xform>() || prim.is<tinyusdz::GeomMesh>()) {
    bone = model.bone_prims.Append<BoneId>(&prim);
    model.bone_child_counts.push_back(0);
    if (parent_bone) {
      model.bone_child_counts[parent_bone]++;
    }
  }

  if (prim.is<tinyusdz::Material>()) {
    model.material_prims.push_back(&prim);
  }

  if (prim.is<tinyusdz::GeomMesh>()) {
    entity = model.entity_prims.Append<EntityId>(&prim);
    model.entity_child_counts.push_back(0);
    model.entity_bones.push_back(bone);
    if (parent_entity) {
      model.entity_child_counts[parent_entity]++;
    }
  }

  for (const auto &child : prim.children()) {
    MP_RETURN_IF_ERROR(ExaminePrim(child, model, bone ? bone : parent_bone,
                                entity ? entity : parent_entity));
  }

  return absl::OkStatus();
}

}  // namespace imp::loader::details::provider_usdz
