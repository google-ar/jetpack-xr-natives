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

#include "core/common/resource_helpers.h"

#include "sandboxed_api/file_toc.h"
#include "core/common/log.h"
#include "core/common/platform_helpers.h"
#include "core/common/robin_map.h"
#include "core/common/robin_set.h"

namespace imp {
namespace {

struct PackagedResourcesState {
  absl::Mutex mutex;
  RobinMap<absl::string_view, const FileToc*> files;
  RobinSet<const FileToc*> resources;
};

PackagedResourcesState& GetPackagedResourcesState() {
  // Avoid destruction-before-use of by never running the destructor.
  static PackagedResourcesState* result = new PackagedResourcesState();
  return *result;
}

const FileToc* EntryFromFilename(absl::string_view filename) {
  auto& state = GetPackagedResourcesState();
  absl::MutexLock lock(&state.mutex);
  auto iter = state.files.find(filename);
  if (iter == state.files.end()) return nullptr;
  return iter->second;
}

}  // namespace

void RegisterPackagedResources(const FileToc* resources) {
  auto& state = GetPackagedResourcesState();
  absl::MutexLock lock(&state.mutex);
  if (state.resources.find(resources) != state.resources.end()) return;
  state.resources.emplace(resources);
  for (const FileToc* entry = resources; entry->name != nullptr; ++entry) {
    absl::string_view name = entry->name;
    if (state.files.find(name) != state.files.end()) {
      IMP_LOG(imp::ERROR) << "Double-registration of '" << name << "'";
      continue;
    }
    state.files.emplace(name, entry);
  }
}

bool PackagedFileExists(absl::string_view filename) {
  return EntryFromFilename(filename) != nullptr;
}

OptionalError LoadPackagedFile(absl::string_view filename,
                               BufferAccess* access) {
  if (const auto* entry = EntryFromFilename(filename)) {
    *access = BufferAccess::Wrap(reinterpret_cast<const uint8_t*>(entry->data),
                                 entry->size);
    return NoError();
  }
  return Error("Failed to find '%.*s' on disk",
               static_cast<int32_t>(filename.size()), filename.data());
}

absl::StatusOr<absl::Cord> PackagedFileToCord(absl::string_view filename) {
  const FileToc* entry = EntryFromFilename(filename);
  if (!entry) {
    return absl::InvalidArgumentError(
        absl::StrFormat("Failed to find '%s' on disk", filename));
  }

  absl::string_view data = {entry->data, entry->size};
  return absl::MakeCordFromExternal(data, [](absl::string_view) {});
}

}  // namespace imp
