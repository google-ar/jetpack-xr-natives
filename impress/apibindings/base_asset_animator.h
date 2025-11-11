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

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_BASE_ASSET_ANIMATOR_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_BASE_ASSET_ANIMATOR_H_

#include <string>

namespace imp {

// Base class for an animation callback.
class BaseAssetAnimator {
 public:
  virtual ~BaseAssetAnimator() = default;

  // Called when the animation completes successfully.
  virtual void OnComplete() = 0;
  // Called if the animation fails to play.
  virtual void OnFailure(std::string error_message) = 0;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_BASE_ASSET_ANIMATOR_H_
