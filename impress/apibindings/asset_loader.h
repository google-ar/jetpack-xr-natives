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

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_ASSET_LOADER_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_ASSET_LOADER_H_

#include <jni.h>

#include <cstdint>
#include <string>

#include "apibindings/base_asset_loader.h"
#include "core/common/jni_helpers.h"

namespace imp {

// JNI wrapper for the Java AssetLoader class.
class AssetLoader : public BaseAssetLoader, public JavaWrapper {
 public:
  AssetLoader(JNIEnv* env, jobject j_asset_loader);

  // Native version of the OnSuccess callback from the Java class.
  void OnSuccess(std::intptr_t value) override;
  // Native version of the OnFailure callback from the Java class.
  void OnFailure(std::string error_message) override;

 private:
  JniHandle on_success_;
  JniHandle on_failure_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_ASSET_LOADER_H_
