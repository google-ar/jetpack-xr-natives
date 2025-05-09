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

#include "core/loader/creator/creator.h"

#include <cstddef>
#include <functional>
#include <iterator>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/IndirectLight.h"
#include "filament/libs/math/include/math/mathfwd.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/vector.h"
#include "core/animation/gltf_animation.h"
#include "core/animation/schemas/gltf_animation_generated.h"
#include "core/async/future.h"
#include "core/async/future_group.h"
#include "core/common/filament_engine_helpers.h"
#include "core/common/optional_error.h"
#include "core/common/trace.h"
#include "core/image/image_contents.h"
#include "core/loader/creator/load_image.h"
#include "core/loader/creator/model_creator.h"
#include "core/loader/details/animation_resources.h"
#include "core/loader/loader_options.h"
#include "core/loader/provider/schemas/loaded_model_generated.h"
#include "core/material_library/material_package.h"
#include "core/model/model_data.h"
#include "core/model/skeleton_data.h"
#include "core/view/base_view.h"

namespace imp::loader::details {
namespace {

using ::filament::Engine;
using ::filament::math::float3;
using ::imp::model::BoneId;
using ::imp::model::ModelData;

template <typename E>
using BundleVector = flatbuffers::Vector<flatbuffers::Offset<E>>;

}  // namespace

Creator::Creator(BaseView &view, MaterialPackage *material_package,
                 LoadedModelAccess &&access)
    : view_(view),
      material_package_(material_package),
      access_(std::move(access)) {}

Creator::~Creator() {
  if (model_creator_ && model_creator_->HasPendingWork()) {
    IMP_LOG(imp::ERROR) << "Destroyed a Creator with pending GPU resources.";
  }
}

Future<std::unique_ptr<model::ModelData>> Creator::CreateModel(
    Engine *engine, LoaderOptions options,
    std::optional<FutureGroup> future_group,
    std::optional<absl::string_view> name) {
  IMP_TRACE();
  model_creator_ = std::make_unique<ModelCreator>(engine, options);
  const schemas::LoadedModel *model = *access_;
  if (!model) {
    return Future<std::unique_ptr<model::ModelData>>(
        absl::FailedPreconditionError("Model Not Loaded"));
  }

  Future<absl::Status> load_all_future = model_creator_->LoadAll(
      view_, model, material_package_, std::move(images_), future_group, name);

  return load_all_future.Then(
      [this]() {
        IMP_TRACE_BLOCK("Then");
        return model_creator_->CreateModelData(&view_);
      },
      {.future_group = future_group});
}

absl::StatusOr<std::unique_ptr<animation::GltfAnimation>>
Creator::CreateAnimation(size_t animation_index) {
  const schemas::LoadedModel *model = *access_;

  if (!model)
    return absl::InternalError(
        "Tried to create animation without anything loaded");

  return CreateAnimationResources(model, animation_index);
}

absl::StatusOr<std::unique_ptr<animation::GltfAnimation>>
Creator::CreateAnimation(absl::string_view name) {
  const schemas::LoadedModel *model = *access_;

  if (!model)
    return absl::InternalError(
        "Tried to create animation without anything loaded");
  const auto *animations = model->animations();
  for (size_t anim_index = 0; anim_index < animations->size(); ++anim_index) {
    const schemas::GltfAnimationInfo *gltf_info = animations->Get(anim_index);
    const flatbuffers::String *anim_name =
        gltf_info->buffer_nested_root()->name();
    if (absl::string_view(anim_name->data(), anim_name->size()) != name)
      continue;
    return CreateAnimation(anim_index);
  }
  return Error("Missing Animation");
}

std::vector<absl::string_view> Creator::GetAnimationNames() const {
  std::vector<absl::string_view> result;
  if (const schemas::LoadedModel *model = *access_) {
    if (const BundleVector<schemas::GltfAnimationInfo> *animation_infos =
            model->animations()) {
      absl::c_transform(
          *animation_infos, std::back_inserter(result),
          [](const schemas::GltfAnimationInfo *animation_info) {
            const animation::schemas::GltfAnimation *gltf_animation =
                animation_info->buffer_nested_root();
            const flatbuffers::String &name = *gltf_animation->name();
            return absl::string_view(name.c_str(), name.size());
          });
    }
  }
  return result;
}

bool Creator::IsFullyLoaded() const {
  if (!model_creator_) return true;
  return model_creator_->IsFullyLoaded();
}

void Creator::WhenFullyLoaded(std::function<void()> &&cb) const {
  if (!model_creator_) {
    IMP_LOG(imp::ERROR) << "Model not being created, cannot add callback";
    return;
  }
  model_creator_->WhenFullyLoaded(std::move(cb));
}

void Creator::RemoveWhenFullyLoadedCallback() {
  model_creator_->RemoveWhenFullyLoadedCallback();
}

Future<absl::Status> Creator::LoadImages(
    const imp::Context &context, std::function<void()> callback,
    std::optional<FutureGroup> future_group) {
  const schemas::LoadedModel *model = *access_;
  // Load textures.
  if (model->textures()->size() != model->images()->size()) {
    return Future<absl::Status>(absl::InternalError(
        "texture infos and texture contents did not match"));
  }
  images_.resize(model->textures()->size());
  std::vector<Future<absl::Status>> load_images_futures;
  load_images_futures.reserve(model->textures()->size());
  for (size_t i = 0; i < model->textures()->size(); ++i) {
    const schemas::TextureInfo *texture_info = model->textures()->Get(i);
    auto image_type = model->images_type()->GetEnum<schemas::ImageInfo>(i);
    auto image_info = model->images()->Get(i);
    load_images_futures.push_back(
        LoadImage(context, texture_info, image_type, image_info, callback)
            .Then(
                [this, i](std::unique_ptr<image::ImageContents> image) {
                  images_[i] = std::move(image);
                },
                {.future_group = future_group}));
  }
  return Future<absl::Status>::CombineList(load_images_futures);
}

absl::Status Creator::BlockUntilLoaded(filament::Engine *engine) {
  constexpr int kTryLimit = 2;
  int tries = 0;
  while (!IsFullyLoaded() && ++tries <= kTryLimit) {
    FlushEngineAndWait(engine);
  }
  if (!IsFullyLoaded()) {
    return Error("Failed to flush loader after %d attempts", tries);
  }
  return NoError();
}

}  // namespace imp::loader::details
