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

#include "core/media/media_asset.h"

#include <memory>
#include <utility>

#include "core/async/future.h"
#include "core/common/trace.h"
#include "core/view/base_view.h"

namespace imp::media {

Future<std::unique_ptr<MediaAsset>> MediaAsset::Load(
    BaseView* view, absl::string_view asset_url,
    Future<resources::Resource> resource_future) {
  IMP_TRACE();
  return resource_future.Then([](resources::Resource resource) {
    IMP_TRACE_BLOCK("Then");
    return std::make_unique<MediaAsset>(std::move(resource));
  });
}

MediaAsset::MediaAsset(imp::resources::Resource resource)
    : resource_(std::move(resource)) {}

size_t MediaAsset::GetSize() const { return resource_.GetData().Size(); }

const uint8_t* MediaAsset::GetData() const {
  return resource_.GetData().Data();
}

}  // namespace imp::media
