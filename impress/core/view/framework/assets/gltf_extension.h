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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_GLTF_EXTENSION_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_GLTF_EXTENSION_H_

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/isf_info.h"

namespace imp {

class GltfRenderer;

// GltfExtension represents an optional feature for impress gltf libraries.
// Please note that this is mostly for some of the extensions that requires
// runtime setup, which means that not necessarily all extensions need to
// inherit from this base class.
//
// GltfExtensions can be registered to GltfRenderer by calling
// GltfRenderer::RegisterExtension<FooExtension>(). Once registered,
// GltfRenderer will automatically create and setup the registered extensions.
class GltfExtension : public Component {
 public:
  virtual ~GltfExtension() = default;

  // Gets the GltfRenderer on the node and then calls
  // Setup(ComponentHandle<GltfRenderer>).
  virtual Future<absl::Status> Setup();

  // Calls SetupInternal(ComponentHandle<GltfRenderer>).
  //
  // This overload is necessary since GltfRenderer creates GltfExtension during
  // Setup() and GetComponent<GltfRendere> will return an invalid handle as it
  // has not finished setting up. And the reasoning behind making this a wrapper
  // function of SetupInternal() is due to the naming hiding feature. If user
  // only overrides Setup(ComponentHandle<GltfRenderer> renderer), then
  // GltfRenderer::Setup() will not be visible by default.
  virtual Future<absl::Status> Setup(ComponentHandle<GltfRenderer> renderer);

  // This will be called after the GltfRenderer is fully set up.
  virtual absl::Status Start() { return absl::OkStatus(); }

 protected:
  // The actual implementation of Setup().
  // Extensions will need to implement this for setting up things needed.
  // Please note that GltfRenderer will not be fully set up when this is called
  // (i.e. calling GetComponent<GltfRenderer> will return an invalid handle). If
  // any work depends GltfRenderer, it should be done in Start().
  virtual Future<absl::Status> SetupInternal(
      ComponentHandle<GltfRenderer> renderer) = 0;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_GLTF_EXTENSION_H_
