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

#include "apibindings/asset_animator.h"

#include <jni.h>

#include <string>

#include "core/common/jni_helpers.h"

namespace imp {
namespace {
const char* GetAssetAnimatorPath(JNIEnv* env) {
  static const char* kPath =
      "androidx/xr/scenecore/spatial/rendering/impress/AssetAnimator";
  static const char* kFallbackPath =
      "androidx/xr/scenecore/impl/impress/AssetAnimator";
  if (env->FindClass(kPath) != nullptr) {
    return kPath;
  }
  JavaExceptionPrintClear(env);
  if (env->FindClass(kFallbackPath)) {
    return kFallbackPath;
  }
  return nullptr;
}
}  // namespace

AssetAnimator::AssetAnimator(JNIEnv* env, jobject j_asset_animator)
    : JavaWrapper(env, j_asset_animator, GetAssetAnimatorPath(env)) {
  on_complete_ = GetMethodHandle("onComplete", "()V");
  on_failure_ = GetMethodHandle("onFailure", "(Ljava/lang/String;)V");
}

void AssetAnimator::OnComplete() { CallVoidMethod(on_complete_); }

void AssetAnimator::OnFailure(std::string error_message) {
  CallVoidMethod(on_failure_, ToString(Env(), error_message));
}

}  // namespace imp
