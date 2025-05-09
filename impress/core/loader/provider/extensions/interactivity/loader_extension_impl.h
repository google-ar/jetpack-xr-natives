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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_EXTENSIONS_INTERACTIVITY_LOADER_EXTENSION_IMPL_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_EXTENSIONS_INTERACTIVITY_LOADER_EXTENSION_IMPL_H_

#include "absl/status/statusor.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/loader/provider/extensions/interactivity/interactivity.proto.imp.h"
#include "core/loader/provider/extensions/interactivity/loader_extension.h"

namespace imp::loader::details {

class InteractivityLoaderExtensionImpl : public InteractivityLoaderExtension {
 public:
  InteractivityLoaderExtensionImpl(flatbuffers::FlatBufferBuilder &fbb)
      : fbb_(fbb) {}

  absl::StatusOr<InteractivityOffset> AddInteractivity(
      const imp::gltf::Interactivity &interactivity) override;

 private:
  flatbuffers::FlatBufferBuilder &fbb_;
};

}  // namespace imp::loader::details

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_EXTENSIONS_INTERACTIVITY_LOADER_EXTENSION_IMPL_H_
