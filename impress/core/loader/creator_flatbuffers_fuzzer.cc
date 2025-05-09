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

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "devtools/build/runtime/get_runfiles_dir.h"
#include "absl/base/log_severity.h"
#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/statusor.h"
#include "absl/strings/cord.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "core/animation/gltf_animation.h"
#include "core/async/future.h"
#include "core/common/buffer_access.h"
#include "core/common/context.h"
#include "core/common/filament_engine_helpers.h"
#include "core/common/flatbuffer_helpers.h"
#include "core/common/resource_helpers.h"
#include "core/loader/creator/creator.h"
#include "core/loader/data/embedded_imp_default_gltf_materials.h"
#include "core/loader/data/embedded_placeholder_textures.h"
#include "core/loader/details/loaded_model_fb.h"
#include "core/loader/provider/schemas/loaded_model_generated.h"
#include "core/material_library/material_package.h"
#include "core/model/model_data.h"
#include "core/resources/resource_manager.h"
#include "core/window/filament_host.h"
#include "testing/test_view.h"
#include "third_party/swiftshader/google/lib_wrapper.h"

namespace imp::loader::details {

using imp::animation::GltfAnimation;
using imp::model::ModelData;

// Since autofuzz runs with the working directory outside of google3,
// swiftshader's LibWrapper fails to find the libs.  Swiftshader provides the
// AddSearchPath helper to manually add the path, use that here.  However, to
// filament uses Swiftshader from within a static initializer in gl_headers.cpp,
// so we need to set the search path before filament does.  Use the non-standard
// GCC attribute "init_priority" so that we run first.
static class InitSwiftshader {
 public:
  InitSwiftshader() {
    const std::string swiftshader_dir =
        devtools_build::GetRunfilesDir() + "/google3/third_party/swiftshader/";
    swiftshader_wrapper::LibWrapper::AddSearchPath(swiftshader_dir);
  }
} init_swiftshader __attribute__((init_priority(101)));

class NoopState : public imp::window::FilamentHost::State {
 public:
  NoopState() = default;
};

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
  imp::testing::TestView test_view(std::make_unique<imp::Context>());
  RegisterPackagedResources(embedded_imp_default_gltf_materials_create());
  RegisterPackagedResources(embedded_placeholder_textures_create());

  absl::SetFlag(&FLAGS_minloglevel, base_logging::FATAL);

  FlatBufferAccess<schemas::LoadedModel> access;
  if (auto status = VerifyAndGetModel(BufferAccess::Wrap(data, size),
                                      VerifyOptions::All, &access);
      !status.ok()) {
    return 0;
  }

  absl::StatusOr<absl::Cord> materials =
      PackagedFileToCord("compiled_imp_default_gltf_materials.zip");
  
  Future<resources::Resource> materials_zip_futures(
      resources::Resource(*std::move(materials)));
  auto material_package =
      std::make_unique<MaterialPackage>(materials_zip_futures);

  Creator creator(*test_view.GetView(), material_package.get(),
                  std::move(access));

  std::unique_ptr<ModelData> model_data;
  auto future = creator.CreateModel(test_view.GetView()->GetSharedEngine());
  while (!future.Ready()) {
    absl::SleepFor(absl::Milliseconds(1));
  }
  if (!future.Get().status().ok()) {
    IMP_LOG(imp::FATAL) << "CreateModel: " << future.Get().status().message();
  }
  model_data = *future.Move();

  for (auto animation : creator.GetAnimationNames()) {
    absl::StatusOr<std::unique_ptr<GltfAnimation>> animation_data_or =
        creator.CreateAnimation(animation);
    if (!animation_data_or.status().ok()) {
      IMP_LOG(imp::INFO) << "CreateAnimation: " << animation_data_or.status().message();
    }
  }

  FlushEngineAndWait(test_view.GetView()->GetSharedEngine());
  model_data.reset();
  material_package.reset();
  FlushEngineAndWait(test_view.GetView()->GetSharedEngine());
  return 0;
}

}  // namespace imp::loader::details
