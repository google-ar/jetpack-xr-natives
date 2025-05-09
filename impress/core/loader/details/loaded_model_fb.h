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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_DETAILS_LOADED_MODEL_FB_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_DETAILS_LOADED_MODEL_FB_H_

#include "core/common/buffer_access.h"
#include "core/common/flatbuffer_helpers.h"
#include "core/common/optional_error.h"
#include "core/loader/provider/schemas/loaded_model_generated.h"

namespace imp::loader::details {

enum VerifyOptions {
  NestedData,  // Verify nested data that is not automatically verified by
               // flatbuffers::Verifier.
  All,         // Verify everything, including the LoadedModel.
};

// Verify nested data that is not automatically verified by
// flatbuffers::Verifier.
OptionalError VerifyModelNestedData(const schemas::LoadedModel* model);

// Verify the flatbuffer and get the model from the storage buffer.
OptionalError VerifyAndGetModel(
    BufferAccess&& storage, VerifyOptions options,
    FlatBufferAccess<schemas::LoadedModel>* out_access);

}  // namespace imp::loader::details

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_DETAILS_LOADED_MODEL_FB_H_
