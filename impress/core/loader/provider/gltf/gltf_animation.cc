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

#include "core/loader/provider/gltf/gltf_animation.h"

#include <algorithm>
#include <iterator>

#include "core/common/optional_error.h"
#include "core/loader/provider/gltf/accessor_reader.h"
#include "core/loader/provider/gltf/dense_data_access.h"
#include "core/loader/provider/gltf/gltf.proto.imp.h"
#include "core/model/model_data.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::loader::details::provider_gltf {
namespace {

using ::filament::math::mat4f;
using ::imp::gltf::imp_proto::Skin;

// LINT.IfChange(matrix_op)
// LINT.ThenChange(
//   //depot/google3/third_party/impress/core/view/framework/\
//       animation/gltf_animator.cc:matrix_op
// )

}  // namespace

OptionalError GetInverseBindPoses(
    const imp::gltf::imp_proto::Gltf& gltf, const Skin& skin,
    model::ModelData::SampledJointLookup<mat4f>* out_poses) {
  if (!skin.inverse_bind_matrices) {
    return Error("Invalid asset; missing bind matrices");
  }

  MP_ASSIGN_OR_RETURN(AccessorReader reader,
                   AccessorReader::Create(gltf, *skin.inverse_bind_matrices));

  if (reader.GetType() != "MAT4" ||
      reader.GetComponentType() != imp::gltf::imp_proto::ComponentType::FLOAT ||
      reader.GetStride() != sizeof(mat4f)) {
    return Error("Invalid asset; expected bind matrices to be mat4f");
  }

  const DenseDataAccess data = reader.GetData();
  std::copy_n(data.ReadRawData<const mat4f>(), reader.GetCount(),
              std::back_inserter(*out_poses));

  return NoError();
}

}  // namespace imp::loader::details::provider_gltf
