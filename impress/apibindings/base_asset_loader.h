/*
 * Copyright 2025 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_BASE_ASSET_LOADER_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_BASE_ASSET_LOADER_H_

#include <cstdint>
#include <string>

namespace imp {

// Base class for an asset loading callback.
class BaseAssetLoader {
 public:
  virtual ~BaseAssetLoader() = default;

  // Called when the asset load completes successfully.
  virtual void OnSuccess(std::intptr_t value) = 0;
  // Called if the asset load fails.
  virtual void OnFailure(std::string error_message) = 0;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_BASE_ASSET_LOADER_H_
