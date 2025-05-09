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

#include "core/loader/details/loaded_model_fb.h"

#include "core/loader/details/animation_resources.h"

namespace imp::loader::details {

OptionalError VerifyModelNestedData(const schemas::LoadedModel* model) {
  MP_RETURN_IF_ERROR(details::VerifyAnimations(model)) << "Animations";
  return NoError();
}

OptionalError VerifyAndGetModel(
    BufferAccess&& storage, VerifyOptions options,
    FlatBufferAccess<schemas::LoadedModel>* out_access) {
  auto verifier = flatbuffers::Verifier(storage.Data(), storage.Size());
  if (options == VerifyOptions::All &&
      !verifier.VerifyBuffer<schemas::LoadedModel>()) {
    return Error("Verification failed");
  }

  MP_RETURN_IF_ERROR(CreateFlatBufferAccess(std::move(storage), out_access));
  MP_RETURN_IF_ERROR(VerifyModelNestedData(out_access->Root()));
  return NoError();
}

}  // namespace imp::loader::details
