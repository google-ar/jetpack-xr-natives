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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_IPC_LOADER_CLIENT_BASE_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_IPC_LOADER_CLIENT_BASE_H_

#include <string>
#include <vector>

#include "absl/strings/string_view.h"
#include "core/common/buffer_access.h"
#include "core/common/flatbuffer_helpers.h"
#include "core/common/optional_error.h"
#include "core/loader/loader_options.h"
#include "core/loader/provider/schemas/loaded_model_generated.h"

namespace imp::loader::ipc {

enum class ResourceType {
  Default,
  Missing,
};

class LoaderClientBase {
 public:
  virtual ~LoaderClientBase() {}

  virtual OptionalError Start(absl::string_view uri, BufferAccess&& access,
                              LoaderOptions options) = 0;

  virtual OptionalError TryLoad(
      std::vector<std::string>* out_missing_resource_paths,
      bool* out_loaded) = 0;

  virtual OptionalError AddResource(absl::string_view path,
                                    BufferAccess&& access,
                                    ResourceType type) = 0;

  // Gets the results of the load.
  virtual OptionalError GetLoadedModel(
      FlatBufferAccess<imp::schemas::LoadedModel>* out_model) = 0;

  virtual void Close(bool wait_for_done_response) = 0;
  virtual void EnsureClosed() = 0;
};

}  // namespace imp::loader::ipc

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_IPC_LOADER_CLIENT_BASE_H_
