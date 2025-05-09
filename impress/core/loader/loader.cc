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

#include "core/loader/loader.h"

#include <functional>
#include <memory>
#include <optional>
#include <utility>

#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/async/future.h"
#include "core/loader/loader_options.h"
#include "core/model/model_data.h"

namespace imp::loader {

LoaderOptions::TextureTranscodeCompressionType
Loader::GetTextureTranscodeCompressionType(filament::Engine& engine) {
  using TextureTranscodeCompressionType =
      loader::LoaderOptions::TextureTranscodeCompressionType;

  using InternalFormat = filament::Texture::InternalFormat;
  if (filament::Texture::isTextureFormatSupported(engine,
                                                  InternalFormat::ETC2_RGB8)) {
    if (filament::Texture::isTextureFormatSupported(
            engine, InternalFormat::RGBA_ASTC_4x4)) {
      return TextureTranscodeCompressionType::AstcAndEtc;
    }
  }
  return TextureTranscodeCompressionType::Unknown;
}

Future<std::unique_ptr<model::ModelData>> Loader::CreateModel(
    filament::Engine* engine) {
  return CreateModel(engine, std::nullopt);
}

Future<std::unique_ptr<model::ModelData>> Loader::CreateModel(
    filament::Engine* engine, std::function<void()>&& callback) {
  return CreateModel(engine, std::move(callback), std::nullopt);
}

}  // namespace imp::loader
