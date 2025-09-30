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

#include "apibindings/asset_loader.h"

#include <jni.h>

#include <cstdint>
#include <string>

#include "core/common/jni_helpers.h"

namespace imp {

AssetLoader::AssetLoader(JNIEnv* env, jobject j_asset_loader)
    : JavaWrapper(env, j_asset_loader,
                  "androidx/xr/scenecore/impl/impress/AssetLoader") {
  on_success_ = GetMethodHandle("onSuccess", "(J)V");
  on_failure_ = GetMethodHandle("onFailure", "(Ljava/lang/String;)V");
}

void AssetLoader::OnSuccess(std::intptr_t value) {
  CallVoidMethod(on_success_, value);
}

void AssetLoader::OnFailure(std::string error_message) {
  CallVoidMethod(on_failure_, ToString(Env(), error_message));
}

}  // namespace imp
