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

#ifndef THIRD_PARTY_IMPRESS_CORE_RESOURCES_CRONET_URL_LOADER_H_
#define THIRD_PARTY_IMPRESS_CORE_RESOURCES_CRONET_URL_LOADER_H_

#include <jni.h>

#include <memory>

#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/common/context.h"
#include "core/resources/url_loader.h"

namespace imp {
namespace resources {

// Interface for a handle to an ongoing URL request.
class UrlRequestHandle {
 public:
  virtual ~UrlRequestHandle() = default;
  virtual void Start() = 0;
  virtual void Cancel() = 0;
};

class UrlRequestFactory {
 public:
  virtual ~UrlRequestFactory() = default;
  virtual std::unique_ptr<UrlRequestHandle> CreateRequest(
      absl::string_view url, JNIEnv* env, imp::Future<absl::Cord>& future) = 0;
};

// Creates a CronetUrlLoader.
// If `url_request_factory` is not provided, a default factory will be used.
// That factory will use Cronet to perform URL requests. If you need to override
// this behavior for testing purposes, you can provide a custom factory here.
std::unique_ptr<UrlLoader> CreateCronetLoader(
    const Context& context,
    std::unique_ptr<UrlRequestFactory> url_request_factory = nullptr);

}  // namespace resources
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RESOURCES_CRONET_URL_LOADER_H_
