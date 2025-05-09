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

#include "core/loader/provider/details/provider_details_common.h"

#include <cctype>

#include "absl/algorithm/container.h"
#include "core/common/file_helpers.h"
#include "core/common/string_helpers.h"

namespace imp::loader::details {
namespace {

void AddResource(absl::string_view path, BufferAccess &&access,
                 ResourceMap *resources) {
  (*resources)[std::string{path}] = std::move(access);
}

}  // namespace

LoaderState::LoaderState(absl::string_view directory,
                         absl::string_view basename,
                         absl::string_view extension,
                         BufferAccess &&primary_resource, LoaderOptions options)
    : directory_(directory),
      basename_(basename),
      extension_(ToLower(extension)),
      primary_resource_(std::move(primary_resource)),
      options_(std::move(options)),
      resources_() {}

void LoaderState::AddResource(absl::string_view path, BufferAccess &&access) {
  ::imp::loader::details::AddResource(path, std::move(access), &resources_);
}

}  // namespace imp::loader::details
