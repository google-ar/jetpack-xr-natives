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

#include "core/render/texture.h"

#include <cassert>

#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/Stream.h"
#include "filament/filament/include/filament/Texture.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "core/math/vec.h"
#include "core/render/content_security_level.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/view/base_view.h"

namespace imp {

Texture::Texture(BaseView& view, filament::Stream* stream,
                 filament::Texture* texture,
                 const filament::TextureSampler& sampler,
                 ContentSecurityLevel security_level)
    : view_(view),
      stream_(stream),
      texture_(texture),
      sampler_(sampler),
      security_level_(security_level) {}

Texture::~Texture() {
  // Ensure filament resources are destroyed over split engine.
  if (split_engine::SplitEngineSerializer* serializer =
          view_.GetSplitEngineSerializer()) {
    serializer->RemoveTexture(*texture_);
  }

  if (filament::Engine* engine = view_.GetSharedEngine()) {
    if (stream_ && engine->isValid(stream_)) {
      engine->destroy(stream_);
    }
    if (texture_ && engine->isValid(texture_)) {
      engine->destroy(texture_);
    }
  }
  stream_ = nullptr;
  texture_ = nullptr;
}

absl::string_view Texture::GetName() const { return name_; }

void Texture::SetName(absl::string_view name) { name_ = name; }

filament::Texture* Texture::GetTexture() const { return texture_; }

filament::Stream* Texture::GetStream() const { return stream_; }

const filament::TextureSampler& Texture::GetSampler() const { return sampler_; }

uint2 Texture::GetSize() const {
  assert(texture_);
  return {texture_->getWidth(), texture_->getHeight()};
}

bool Texture::IsValid() const {
  return view_.GetSharedEngine() && view_.GetSharedEngine()->isValid(texture_);
}

ContentSecurityLevel Texture::GetContentSecurityLevel() const {
  return security_level_;
}

}  // namespace imp
