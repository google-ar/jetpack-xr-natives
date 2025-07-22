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

#include "core/lighting/image_based_lighting_asset.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/cord.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_split.h"
#include "absl/strings/string_view.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/common/buffer_access.h"
#include "core/common/context.h"
#include "core/common/file_helpers.h"
#include "core/common/small_source_location.h"
#include "core/common/zip_helpers.h"
#include "core/image/decode_image.h"
#include "core/image/image_contents.h"
#include "core/lighting/image_based_lighting_types.h"
#include "core/render/texture.h"
#include "core/render/texture_factory.h"
#include "core/resources/resource_manager.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/view/base_view.h"
#include "core/view/utils/string_map.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {

namespace {

constexpr absl::string_view kSphericalHarmonicsFileName = "sh.txt";

constexpr absl::string_view kCubemapPrefix = "m";
constexpr std::array<absl::string_view, 6> kCubemapFaceSuffixes = {
    "px", "nx", "py", "ny", "pz", "nz"};
constexpr absl::string_view kCubemapFileExtension = ".rgb32f";

struct Cubemap {
  TexturePtr texture;
  std::vector<CubemapLevel> levels;
};

/**
 * Holds the cubemap and the stitched face images for all cubemap levels.
 */
struct CubemapImageContents {
  std::unique_ptr<Cubemap> cubemap;
  std::vector<std::unique_ptr<image::ImageContents>> stitched_face_images;
};

bool HasMipmapLevel(
    uint8_t level, const StringMap<resources::Resource>& filename_to_resource) {
  std::string level_file_path =
      absl::StrFormat("%s%d_%s%s", kCubemapPrefix, level,
                      kCubemapFaceSuffixes[0], kCubemapFileExtension);
  return filename_to_resource.find(level_file_path) !=
         filename_to_resource.end();
}

void MovePixelsToTexture(
    filament::Engine& engine, Texture* texture, uint8_t level,
    CubemapLevel& cubemap_level,
    std::unique_ptr<image::ImageContents> stitched_face_image) {
  auto mipmap_descriptors =
      stitched_face_image->CreatePixelBufferDescriptorLevels(
          nullptr, /* is_r11_g11_b10 = */ true);
  for (int mipmap = 0; mipmap < mipmap_descriptors.size(); ++mipmap) {
    texture->GetTexture()->setImage(engine, level,
                                    std::move(mipmap_descriptors[mipmap]),
                                    cubemap_level.face_offsets);
  }
}

OwnedTexturePtr CreateIblCubemapTexture(CubemapImageContents& cubemaps,
                                        TextureFactory& texture_factory,
                                        filament::Engine& engine) {
  cubemaps.cubemap->texture =
      texture_factory.CreateTexture(TextureFactory::TextureCreationSettings{
          .width = cubemaps.cubemap->levels.front().face_size,
          .height = cubemaps.cubemap->levels.front().face_size,
          .format = TextureFactory::Format::R11F_G11F_B10F,
          .levels = cubemaps.cubemap->levels.size(),
          .sampler_type = TextureFactory::SamplerType::SAMPLER_CUBEMAP});
  if (!cubemaps.cubemap->texture) {
    return {};
  }

  for (uint8_t level = 0; level < cubemaps.cubemap->levels.size(); level++) {
    CubemapLevel& cubemap_level = cubemaps.cubemap->levels[level];
    std::unique_ptr<image::ImageContents>& stitched_face_image =
        cubemaps.stitched_face_images[level];
    MovePixelsToTexture(engine, cubemaps.cubemap->texture.get(), level,
                        cubemap_level, std::move(stitched_face_image));
  }

  return std::move(cubemaps.cubemap->texture);
}

OwnedTexturePtr CreateSkyboxCubemapTexture(CubemapLevelImageContents& cubemap,
                                           TextureFactory& texture_factory,
                                           filament::Engine& engine) {
  std::unique_ptr<Cubemap> skybox_cubemap = std::make_unique<Cubemap>();
  skybox_cubemap->levels.push_back(std::move(cubemap.cubemap_level));
  skybox_cubemap->texture =
      texture_factory.CreateTexture(TextureFactory::TextureCreationSettings{
          .width = skybox_cubemap->levels.front().face_size,
          .height = skybox_cubemap->levels.front().face_size,
          .format = TextureFactory::Format::R11F_G11F_B10F,
          .sampler_type = TextureFactory::SamplerType::SAMPLER_CUBEMAP});
  if (!skybox_cubemap->texture) {
    return {};
  }

  MovePixelsToTexture(engine, skybox_cubemap->texture.get(), 0,
                      skybox_cubemap->levels.front(),
                      std::move(cubemap.stitched_face_image));

  return std::move(skybox_cubemap->texture);
}

CubemapImageContents CreateCubemapImageContents(
    std::vector<CubemapLevelImageContents>& cubemaps) {
  std::unique_ptr<Cubemap> ibl_cubemap = std::make_unique<Cubemap>();
  std::vector<std::unique_ptr<image::ImageContents>> stitched_face_images;
  for (auto& [cubemap_level, stitched_face_image] : cubemaps) {
    ibl_cubemap->levels.push_back(std::move(cubemap_level));
    stitched_face_images.push_back(std::move(stitched_face_image));
  }

  return CubemapImageContents{std::move(ibl_cubemap),
                              std::move(stitched_face_images)};
}

absl::StatusOr<CubemapLevelImageContents> CreateCubemapLevelImageContents(
    std::vector<std::unique_ptr<image::ImageContents>>& images,
    absl::string_view level_prefix) {
  // Check if each face image has equal width and height.
  for (int i = 0; i < images.size(); ++i) {
    std::string face_file = absl::StrCat(level_prefix, kCubemapFaceSuffixes[i],
                                         kCubemapFileExtension);
    if (images[i]->GetWidth() != images[i]->GetHeight()) {
      return absl::FailedPreconditionError(absl::StrFormat(
          "Cubemap face image %s should have equal height and width. (%d "
          "!= %d)",
          face_file, images[i]->GetWidth(), images[i]->GetHeight()));
    }
  }

  CubemapLevel cubemap_level;
  std::unique_ptr<image::ImageContents> stitched_face_image;
  cubemap_level.face_size = images.front()->GetWidth();

  absl::Status stitch_status = image::ImageContents::CreateStitched(
      std::move(images), cubemap_level.face_offsets.offsets,
      &stitched_face_image);

  if (!stitch_status.ok()) {
    return stitch_status;
  }

  return CubemapLevelImageContents{cubemap_level,
                                   std::move(stitched_face_image)};
}

Future<CubemapLevelImageContents> LoadCubemapLevel(
    const Context& context, std::string level_prefix,
    const StringMap<resources::Resource>& filename_to_resource) {
  std::vector<Future<std::unique_ptr<image::ImageContents>>> face_image_futures;
  for (absl::string_view face_suffix : kCubemapFaceSuffixes) {
    std::string face_file =
        absl::StrCat(level_prefix, face_suffix, kCubemapFileExtension);

    if (filename_to_resource.find(face_file) == filename_to_resource.end()) {
      return Future<CubemapLevelImageContents>(absl::NotFoundError(
          absl::StrFormat("Cubemap face %s not found.", face_file)));
    }

    const resources::Resource& face_resource =
        filename_to_resource.at(absl::string_view(face_file));
    face_image_futures.push_back(
        image::DecodeImage(context, face_file, face_resource));
  }

  // TODO (broken link) Investigate why this needs to be an rvalue
  return Future<std::unique_ptr<image::ImageContents>>::MergeList(
             face_image_futures)
      .Then([level_prefix](
                std::vector<std::unique_ptr<image::ImageContents>>&& images)
                -> absl::StatusOr<CubemapLevelImageContents> {
        return CreateCubemapLevelImageContents(images, level_prefix);
      });
}

// Creates a deep copy of a CubemapLevelImageContents.
absl::StatusOr<CubemapLevelImageContents> DeepCopyCubemapLevelImageContents(
    const CubemapLevelImageContents& cubemap_level_image_contents) {
  CubemapLevelImageContents copy;
  copy.cubemap_level = cubemap_level_image_contents.cubemap_level;
  image::ImageContents& stitched_face_image =
      *cubemap_level_image_contents.stitched_face_image;
  MP_ASSIGN_OR_RETURN(
      copy.stitched_face_image,
      image::ImageContents::CreatePreStitchedImage(
          stitched_face_image.GetWidth(), stitched_face_image.GetHeight(),
          std::vector<uint8_t>(
              stitched_face_image.GetData(),
              stitched_face_image.GetData() + stitched_face_image.GetSize())));
  return copy;
}

// Creates a deep copy of CubemapLevelImageContents required for constructing an
// ImageBasedLightingAsset.
absl::StatusOr<ImageBasedLightingAssetCubemapImages> DeepCopyIblCubemaps(
    const ImageBasedLightingAssetCubemapImages& cubemaps) {
  std::vector<CubemapLevelImageContents> ibl_cubemap_copy;
  ibl_cubemap_copy.reserve(cubemaps.ibl_cubemap_images.size());
  for (const CubemapLevelImageContents& ibl_cubemap_image :
       cubemaps.ibl_cubemap_images) {
    MP_ASSIGN_OR_RETURN(CubemapLevelImageContents ibl_cubemap_image_copy,
                     DeepCopyCubemapLevelImageContents(ibl_cubemap_image));
    ibl_cubemap_copy.push_back(std::move(ibl_cubemap_image_copy));
  }

  std::optional<CubemapLevelImageContents> skybox_cubemap_copy = std::nullopt;
  if (cubemaps.skybox_cubemap_images.has_value()) {
    MP_ASSIGN_OR_RETURN(skybox_cubemap_copy, DeepCopyCubemapLevelImageContents(
                                              *cubemaps.skybox_cubemap_images));
  }
  return ImageBasedLightingAssetCubemapImages{
      .ibl_cubemap_images = std::move(ibl_cubemap_copy),
      .skybox_cubemap_images = std::move(skybox_cubemap_copy)};
}

// TODO Replace this with Future<T>::Merge once vectors of
// move-only types are supported.
Future<ImageBasedLightingAssetCubemapImages> MergeIblCubemapFutures(
    Future<std::vector<CubemapLevelImageContents>> cubemap_vector_future,
    Future<CubemapLevelImageContents> cubemap_future) {
  return cubemap_vector_future.Then(
      [cubemap_future](
          std::vector<CubemapLevelImageContents>&& ibl_cubemap_contents) {
        return cubemap_future.Then([ibl_cubemap_contents =
                                        std::move(ibl_cubemap_contents)](
                                       absl::StatusOr<CubemapLevelImageContents>
                                           skybox_cubemap_contents) mutable {
          std::optional<CubemapLevelImageContents> optional_skybox_contents =
              std::nullopt;
          // TODO (broken link) Determine if the error should be
          // swallowed or propagated.
          if (skybox_cubemap_contents.ok()) {
            optional_skybox_contents = std::move(*skybox_cubemap_contents);
          }
          return ImageBasedLightingAssetCubemapImages{
              .ibl_cubemap_images = std::move(ibl_cubemap_contents),
              .skybox_cubemap_images = std::move(optional_skybox_contents)};
        });
      });
}

}  // namespace

// TODO Split up this function into smaller functions.
Future<std::unique_ptr<ImageBasedLightingAsset>> ImageBasedLightingAsset::Load(
    BaseView* view, absl::string_view asset_url,
    Future<resources::Resource> resource_future) {
  if (view->GetSharedEngine()->getActiveFeatureLevel() ==
      filament::backend::FeatureLevel::FEATURE_LEVEL_0) {
    return Future<std::unique_ptr<ImageBasedLightingAsset>>(
        absl::FailedPreconditionError("FeatureLevel::FEATURE_LEVEL_0 does not "
                                      "support ImageBasedLightingAsset"));
  }
  return resource_future.Then(
      [view, context = view->GetContext(),
       &texture_factory = view->GetTextureFactory(),
       &engine = *view->GetSharedEngine(),
       url = std::string(asset_url)](resources::Resource resource)
          -> Future<std::unique_ptr<ImageBasedLightingAsset>> {
        BufferAccess data = resource.GetData();
        std::vector<std::string> filenames;
        absl::Status get_filenames_status =
            GetFilenamesFromZip(data, &filenames);

        if (!get_filenames_status.ok()) {
          return Future<std::unique_ptr<ImageBasedLightingAsset>>(
              absl::NotFoundError(absl::StrFormat(
                  "Failed to load filenames from ibl asset zip: %s",
                  get_filenames_status.ToString())));
        }

        StringMap<resources::Resource> ibl_resources;
        std::set<std::string> filenames_set;
        for (const auto& filename : filenames) {
          absl::string_view basename = GetLocalFilenameFromFilename(filename);

          if (basename.empty()) {
            continue;
          }
          filenames_set.insert(filename);
        }

        auto file_buffers_or_status = GetFilesFromZip(data, filenames_set);
        if (!file_buffers_or_status.ok()) {
          return Future<std::unique_ptr<ImageBasedLightingAsset>>(
              absl::NotFoundError(
                  absl::StrFormat("Failed to load files from ibl asset zip: %s",
                                  file_buffers_or_status.status().ToString())));
        }
        for (auto& file : *file_buffers_or_status) {
          // Make the cord own the data and destroy it by moving the
          // unique_ptr. The resource will then reference count the cord which
          // will be destroyed once the data is done loading.
          size_t file_size = file.access.Size();
          std::unique_ptr<uint8_t[]> file_data =
              file.access.ReleaseDataOwnership();
          const absl::string_view file_data_view(
              reinterpret_cast<char*>(file_data.get()), file_size);
          resources::Resource resource(absl::MakeCordFromExternal(
              file_data_view, [file_data = std::move(file_data)]() {}));
          ibl_resources.emplace(GetLocalFilenameFromFilename(file.filename),
                                std::move(resource));
        }

        // Load spherical harmonics
        std::unique_ptr<SphericalHarmonics> spherical_harmonics = nullptr;
        auto sh_buffer_it = ibl_resources.find(kSphericalHarmonicsFileName);
        if (sh_buffer_it != ibl_resources.end()) {
          const resources::Resource& sh_resource = sh_buffer_it->second;

          absl::StatusOr<std::unique_ptr<SphericalHarmonics>>
              spherical_harmonics_status =
                  LoadSphericalHarmonics(sh_resource.GetData());
          if (!spherical_harmonics_status.ok()) {
            return Future<std::unique_ptr<ImageBasedLightingAsset>>(
                spherical_harmonics_status.status());
          }

          spherical_harmonics = std::move(*spherical_harmonics_status);

          // Remove the spherical harmonics buffer from ibl_buffers so that we
          // don't have to deal with it when loading cubemaps.
          ibl_resources.erase(kSphericalHarmonicsFileName);
        }

        Future<std::vector<CubemapLevelImageContents>> ibl_future =
            LoadIblCubemapLevelImageContents(context, texture_factory, engine,
                                             ibl_resources);

        Future<CubemapLevelImageContents> skybox_future =
            LoadSkyboxCubemapLevelImageContents(context, texture_factory,
                                                engine, ibl_resources);

        return MergeIblCubemapFutures(std::move(ibl_future),
                                      std::move(skybox_future))
            .Then([spherical_harmonics = std::move(spherical_harmonics), url,
                   view](ImageBasedLightingAssetCubemapImages
                             cubemap_images) mutable {
              return SerializeAndConstructImageBasedLightingAsset(
                  *view, std::move(spherical_harmonics),
                  std::move(cubemap_images), url);
            });
      },
      Executor::Type::kBackground);
}

// Get the cubemap texture for lighting. This could return an invalid pointer
// if IBL cubemap was not built or failed to load.
Texture* ImageBasedLightingAsset::GetLightingCubemap() const {
  if (ibl_cubemap_) {
    return &(*ibl_cubemap_);
  }
  return nullptr;
}

// Get the cubemap texture for Skybox. This could return an invalid pointer if
// skybox cubemap was not built or failed to load.
Texture* ImageBasedLightingAsset::GetSkyboxCubemap() const {
  if (skybox_cubemap_) {
    return &(*skybox_cubemap_);
  }
  return nullptr;
}

// Get the cubemap texture for reflections. This returns the skybox cubemap
// if it exists, otherwise it returns the lighting cubemap. This could return
// an invalid pointer if both cubemaps were not built or failed to load.
Texture* ImageBasedLightingAsset::GetReflectionTexture() const {
  if (GetSkyboxCubemap()) {
    return GetSkyboxCubemap();
  } else if (GetLightingCubemap()) {
    return GetLightingCubemap();
  }
  return nullptr;
}

BorrowedTexturePtr ImageBasedLightingAsset::BorrowLightingCubemap(
    SmallSourceLocation loc) const {
  if (ibl_cubemap_) {
    return ibl_cubemap_.Borrow(loc);
  }
  return BorrowedTexturePtr();
}

BorrowedTexturePtr ImageBasedLightingAsset::BorrowSkyboxCubemap(
    SmallSourceLocation loc) const {
  if (skybox_cubemap_) {
    return skybox_cubemap_.Borrow(loc);
  }
  return BorrowedTexturePtr();
}

BorrowedTexturePtr ImageBasedLightingAsset::BorrowReflectionTexture(
    SmallSourceLocation loc) const {
  if (skybox_cubemap_) {
    return BorrowSkyboxCubemap(loc);
  } else if (ibl_cubemap_) {
    return BorrowLightingCubemap(loc);
  }
  return BorrowedTexturePtr();
}

// Given spherical harmonics, IBL CubemapLevelImageContents, and an optional
// skybox CubemapLevelImageContents, returns an ImageBasedLightingAsset.
// Serializes the resultant ImageasedLightingAsset and adds it to the
// SplitEngineSerializer, if it exists.
absl::StatusOr<std::unique_ptr<ImageBasedLightingAsset>>
ImageBasedLightingAsset::SerializeAndConstructImageBasedLightingAsset(
    BaseView& view, std::unique_ptr<SphericalHarmonics> spherical_harmonics,
    ImageBasedLightingAssetCubemapImages cubemap_images,
    std::optional<std::string_view> asset_url) {
  if (split_engine::SplitEngineSerializer* serializer =
          view.GetSplitEngineSerializer()) {
    // Copy the spherical harmonics and cubemaps so that they can be
    // serialized.
    SphericalHarmonics spherical_harmonics_copy = *spherical_harmonics;
    MP_ASSIGN_OR_RETURN(ImageBasedLightingAssetCubemapImages cubemap_images_copy,
                     DeepCopyIblCubemaps(cubemap_images));
    auto image_based_lighting_asset = std::make_unique<ImageBasedLightingAsset>(
        view, std::move(spherical_harmonics), std::move(cubemap_images),
        asset_url);

    Texture* reflection_texture =
        image_based_lighting_asset->GetReflectionTexture();
    if (!reflection_texture) {
      return absl::InternalError("Failed to get reflections texture.");
    }

    serializer->SerializeImageBasedLightingAsset(
        *reflection_texture->GetTexture(), spherical_harmonics_copy,
        cubemap_images_copy);
    // Remove the IBL asset from SplitEngine when it is destroyed.
    image_based_lighting_asset->on_destroy_callback_ = [serializer,
                                                        reflection_texture] {
      serializer->RemoveImageBasedLightingAsset(
          *reflection_texture->GetTexture());
    };

    return image_based_lighting_asset;
  }

  return std::make_unique<ImageBasedLightingAsset>(
      view, std::move(spherical_harmonics), std::move(cubemap_images),
      asset_url);
}

Future<std::unique_ptr<ImageBasedLightingAsset>> ImageBasedLightingAsset::Load(
    BaseView* view, absl::string_view asset_url,
    Future<resources::Resource> resource_future, CustomLoader& custom_loader) {
  if (view->GetSharedEngine()->getActiveFeatureLevel() ==
      filament::backend::FeatureLevel::FEATURE_LEVEL_0) {
    return Future<std::unique_ptr<ImageBasedLightingAsset>>(
        absl::FailedPreconditionError("FeatureLevel::FEATURE_LEVEL_0 does not "
                                      "support ImageBasedLightingAsset"));
  }
  return custom_loader.Load(view, asset_url, resource_future);
}

Future<std::vector<CubemapLevelImageContents>>
ImageBasedLightingAsset::LoadIblCubemapLevelImageContents(
    const Context& context, TextureFactory& texture_factory,
    filament::Engine& engine,
    const StringMap<resources::Resource>& filename_to_resource) {
  if (!HasMipmapLevel(0, filename_to_resource)) {
    return Future<std::vector<CubemapLevelImageContents>>(
        std::vector<CubemapLevelImageContents>());
  }

  std::vector<Future<CubemapLevelImageContents>> load_cubemap_futures;
  for (uint8_t level = 0; HasMipmapLevel(level, filename_to_resource);
       level++) {
    std::string level_prefix = absl::StrFormat("%s%d_", kCubemapPrefix, level);
    load_cubemap_futures.push_back(LoadCubemapLevel(
        context, std::string(level_prefix), filename_to_resource));
  }

  return Future<CubemapLevelImageContents>::MergeList(load_cubemap_futures);
}

Future<CubemapLevelImageContents>
ImageBasedLightingAsset::LoadSkyboxCubemapLevelImageContents(
    const Context& context, TextureFactory& texture_factory,
    filament::Engine& engine,
    const StringMap<resources::Resource>& filename_to_resource) {
  return LoadCubemapLevel(context, "", filename_to_resource)
      .Then(
          [](absl::StatusOr<CubemapLevelImageContents> cubemap)
              -> absl::StatusOr<CubemapLevelImageContents> {
            if (!cubemap.ok()) {
              return absl::NotFoundError(
                  absl::StrFormat("Failed to load skybox cubemap level: %s",
                                  cubemap.status().ToString()));
            }
            return std::move(*cubemap);
          },
          Executor::Type::kForeground);
}

ImageBasedLightingAsset::ImageBasedLightingAsset(
    std::unique_ptr<SphericalHarmonics> spherical_harmonics,
    OwnedTexturePtr ibl_cubemap, OwnedTexturePtr skybox_cubemap,
    std::optional<std::string_view> asset_url)
    : spherical_harmonics_(std::move(spherical_harmonics)),
      ibl_cubemap_(std::move(ibl_cubemap)),
      skybox_cubemap_(std::move(skybox_cubemap)),
      asset_url_(asset_url) {}

ImageBasedLightingAsset::ImageBasedLightingAsset(
    BaseView& view, std::unique_ptr<SphericalHarmonics> spherical_harmonics,
    ImageBasedLightingAssetCubemapImages ibl_cubemap_images,
    std::optional<std::string_view> asset_url)
    : asset_url_(asset_url) {
  Context context = view.GetContext();
  TextureFactory& texture_factory = view.GetTextureFactory();
  filament::Engine& engine = *view.GetSharedEngine();

  OwnedTexturePtr ibl_cubemap_texture =
      ibl_cubemap_images.ibl_cubemap_images.empty()
          ? OwnedTexturePtr()
          : LoadIblCubemap(context, texture_factory,
                           std::move(ibl_cubemap_images.ibl_cubemap_images),
                           engine);
  OwnedTexturePtr skybox_cubemap_texture =
      ibl_cubemap_images.skybox_cubemap_images.has_value()
          ? LoadSkyboxCubemap(
                context, texture_factory, engine,
                std::move(*ibl_cubemap_images.skybox_cubemap_images))
          : OwnedTexturePtr();

  spherical_harmonics_ = std::move(spherical_harmonics);
  ibl_cubemap_ = std::move(ibl_cubemap_texture);
  skybox_cubemap_ = std::move(skybox_cubemap_texture);
};

absl::StatusOr<std::unique_ptr<SphericalHarmonics>>
ImageBasedLightingAsset::LoadSphericalHarmonics(const BufferAccess& data) {
  auto spherical_harmonics = std::make_unique<SphericalHarmonics>();

  std::vector<absl::string_view> lines =
      absl::StrSplit(data.StringView(), '\n');
  for (absl::string_view line : lines) {
    if (line.empty()) {
      continue;
    }

    // Extract float3 values from the line.
    float r, g, b;
    std::string line_str(line);
    if (sscanf(line_str.c_str(), "(%f, %f, %f);", &r, &g, &b) != 3) {
      return absl::FailedPreconditionError(
          absl::StrFormat("Malformed spherical harmonics file read. Discarded "
                          "SphericalHarmonics object."));
    }

    spherical_harmonics->coefficients.push_back({r, g, b});
  }

  size_t num_coefficients = spherical_harmonics->coefficients.size();
  if (num_coefficients == 0) {
    return std::unique_ptr<SphericalHarmonics>();
  }

  spherical_harmonics->num_bands = sqrt(num_coefficients);
  // Check if the number of coefficients are sqaure number. Non square number of
  // coefficients might suggest a malformed file.
  // Each spherical harmonics band has `2 * sh_index - 1` coffeicients, which
  // means the total number of coefficients is:
  // 1 + 3 + ... + 2 * num_bands - 1 = num_bands ^ 2.
  if (spherical_harmonics->num_bands * spherical_harmonics->num_bands !=
      num_coefficients) {
    return absl::FailedPreconditionError(absl::StrFormat(
        "The number of spherical harmonics coefficients is not a square number "
        "(loaded %d coefficients). Discarded "
        "SphericalHarmonics object.",
        num_coefficients));
  }
  return spherical_harmonics;
}

OwnedTexturePtr ImageBasedLightingAsset::LoadIblCubemap(
    const Context& context, TextureFactory& texture_factory,
    std::vector<CubemapLevelImageContents> ibl_cubemap_level_image_contents,
    filament::Engine& engine) {
  // TODO (broken link) Investigate why this needs to be an rvalue
  absl::StatusOr<CubemapImageContents> cubemap_image_contents =
      CreateCubemapImageContents(ibl_cubemap_level_image_contents);
  // TODO (broken link) Determine if the error should be swallowed or
  // propagated.
  if (!cubemap_image_contents.ok()) {
    return OwnedTexturePtr();
  }
  return CreateIblCubemapTexture(*cubemap_image_contents, texture_factory,
                                 engine);
}

OwnedTexturePtr ImageBasedLightingAsset::LoadSkyboxCubemap(
    const Context& context, TextureFactory& texture_factory,
    filament::Engine& engine,
    CubemapLevelImageContents skybox_cubemap_level_image_contents) {
  return CreateSkyboxCubemapTexture(skybox_cubemap_level_image_contents,
                                    texture_factory, engine);
}

}  // namespace imp
