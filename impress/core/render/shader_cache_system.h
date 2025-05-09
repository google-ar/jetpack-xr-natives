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

#ifndef THIRD_PARTY_IMPRESS_CORE_RENDER_SHADER_CACHE_SYSTEM_H_
#define THIRD_PARTY_IMPRESS_CORE_RENDER_SHADER_CACHE_SYSTEM_H_

#include <string>

#include "core/ncsb/system.h"
#include "core/view/base_view.h"

namespace imp {

// ShaderCacheSystem caches compiled shader programs as binary files and loads
// them back when requested.
//
// Internally, this class utilizes filament's platform APIs.
// filament::backend::Platform::setBlobFunc() allows us to provide custom
// functions for handling shader program binary caching and loading.
//
// TODO: Add testing for ShaderCacheSystem.
// TODO: Add support for iOS.
// TODO: Add support for WASM.
class ShaderCacheSystem : public System {
 public:
  explicit ShaderCacheSystem(BaseView* view);

  // Returns true if shader caching is supported on the current platform.
  constexpr static bool IsSupportedOnCurrentPlatform(const BaseView* view);

  // Sets up the ShaderCacheSystem. Shader caching will only be enabled after
  // calling Setup().
  void Setup();

  // Enables shader caching. Please note that this function should only be
  // called within or after imp::View::Setup().
  void EnableShaderCaching();

  // Disables shader caching.
  void DisableShaderCaching();

 private:
  void WriteCache(const void* key, size_t keySize, const void* value,
                  size_t valueSize);

  size_t ReadCache(const void* key, size_t keySize, void* value,
                   size_t valueSize);

  BaseView* view_;
  std::string cache_dir_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RENDER_SHADER_CACHE_SYSTEM_H_
