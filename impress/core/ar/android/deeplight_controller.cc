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

#include "core/ar/android/deeplight_controller.h"

#include <memory>

#include "core/common/log.h"
#include "third_party/arcore/ar/core/c_api/arcore_c_api.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/IndirectLight.h"
#include "filament/filament/include/filament/Texture.h"
#include "filament/libs/math/include/math/vec4.h"
#include "core/ar/ar_hdr_lighting.h"
#include "core/common/trace.h"
#include "core/lighting/environment_light_factory.h"

namespace imp::ar {

namespace {
/**
 * Convert Environmental HDR's spherical harmonics to Filament spherical
 * harmonics.
 *
 * <p>This conversion is calculated to include the following:
 *
 * <ul>
 *   <li>pre-scaling by SH basis normalization factor [shader optimization]
 *   <li>sqrt(2) factor coming from keeping only the real part of the basis
 * [shader optimization] <li>1/pi factor for the diffuse lambert BRDF [shader
 * optimization] <li>|dot(n,l)| spherical harmonics [irradiance] <li>scaling for
 * convolution of SH function by radially symmetrical SH function [irradiance]
 * </ul>
 *
 * <p>kEnvironmentalHDRToFilamentSHIndexMap must be applied change ordering
 * of coeffients from Environmental HDR to filament.
 */
constexpr float kEnvironmentalHDRToFilamentSHCoefficients[] = {
    0.282095f,  -0.325735f, 0.325735f,  -0.325735f, 0.273137f,
    -0.273137f, 0.078848f,  -0.273137f, 0.136569f};

// SH coefficients are not in the same order in Filament and Environmental HDR.
// SH coefficients at indices 6 and 7 are swapped between the two
// implementations.
constexpr int kEnvironmentalHDRToFilamentSHIndexMap[] = {0, 1, 2, 3, 4,
                                                         5, 7, 6, 8};

constexpr size_t kCubemapFaceSize = 16u;
constexpr size_t kCubemapFaceCount = 6u;

// Scales light intensities so that they are compatible with the default
// Impress camera exposure settings.
constexpr float kDeeplightIntensityScale = 200;
}  // namespace

DeeplightController::DeeplightController(
    filament::Engine* engine, imp::EnvironmentLightFactory* env_light_factory)
    : engine_(engine),
      env_light_factory_(env_light_factory),
      ar_light_estimate_(nullptr),
      ar_image_cubemap_{nullptr} {}

DeeplightController::~DeeplightController() { ReleaseCubemap(); }

void DeeplightController::ReleaseCubemap() {
  for (int i = 0; i < kCubemapFaceCount; ++i) {
    if (ar_image_cubemap_[i] != nullptr) {
      ArImage_release(ar_image_cubemap_[i]);
    }
    ar_image_cubemap_[i] = nullptr;
  }
}

void DeeplightController::Update(ArSession_* session, ArConfig_* config,
                                 ArFrame_* frame) {
  IMP_TRACE();
  ArConfig_getLightEstimationMode(session, config, &light_estimation_mode_);

  if (light_estimation_mode_ == AR_LIGHT_ESTIMATION_MODE_DISABLED) {
    return;
  }

  if (!ar_light_estimate_) {
    ArLightEstimate_* ar_light_estimate;
    ArLightEstimate_create(session, &ar_light_estimate);
    ar_light_estimate_ = UniqueArLightEstimate(ar_light_estimate);
  }
  ArFrame_getLightEstimate(session, frame, ar_light_estimate_.get());

  // Check whether this is a valid light estimate.
  ArLightEstimateState light_estimate_state = AR_LIGHT_ESTIMATE_STATE_NOT_VALID;
  ArLightEstimate_getState(session, ar_light_estimate_.get(),
                           &light_estimate_state);
  if (light_estimate_state == AR_LIGHT_ESTIMATE_STATE_NOT_VALID) {
    return;
  }
  // Always get the ambient light estimate.
  ArLightEstimate_getColorCorrection(session, ar_light_estimate_.get(),
                                     &ambient_light_estimate_.v[0]);
  ambient_light_estimate_.w = std::max(ambient_light_estimate_.w, 0.0f);

  if (light_estimation_mode_ == AR_LIGHT_ESTIMATION_MODE_ENVIRONMENTAL_HDR) {
    // Update the main directional light, getting the direction relative to the
    // directional light anchor.
    ArLightEstimate_getEnvironmentalHdrMainLightDirection(
        session, ar_light_estimate_.get(), main_light_direction_.v);

    float3 main_light_intensity;
    ArLightEstimate_getEnvironmentalHdrMainLightIntensity(
        session, ar_light_estimate_.get(), main_light_intensity.v);

    float3 ambient_spherical_harmonics[9];
    ArLightEstimate_getEnvironmentalHdrAmbientSphericalHarmonics(
        session, ar_light_estimate_.get(), ambient_spherical_harmonics[0].v);

    for (int srcIndex = 0; srcIndex < 9; ++srcIndex) {
      int destIndex = kEnvironmentalHDRToFilamentSHIndexMap[srcIndex];
      irradiance_data_[destIndex] =
          ambient_spherical_harmonics[srcIndex] *
          kEnvironmentalHDRToFilamentSHCoefficients[destIndex];
    }

    main_light_intensity_scalar_ = std::max(
        1.0f,
        std::max(main_light_intensity.x,
                 std::max(main_light_intensity.y, main_light_intensity.z)));
    main_light_color_ = main_light_intensity / main_light_intensity_scalar_;

    // First release the old cubemap before creating a new one.
    ReleaseCubemap();
    // Get new cubemap.
    ArLightEstimate_acquireEnvironmentalHdrCubemap(
        session, ar_light_estimate_.get(), ar_image_cubemap_);
  }
}

std::vector<float3> DeeplightController::GetSphericalHarmonicsLighting() {
  IMP_TRACE();
  std::vector<float3> spherical_harmonics(9);
  for (int srcIndex = 0; srcIndex < 9; ++srcIndex) {
    int destIndex = kEnvironmentalHDRToFilamentSHIndexMap[srcIndex];
    spherical_harmonics[srcIndex] =
        irradiance_data_[destIndex] /
        kEnvironmentalHDRToFilamentSHCoefficients[destIndex];
  }
  return spherical_harmonics;
}

bool DeeplightController::IsHdrLightingEnabled() const {
  return light_estimation_mode_ == AR_LIGHT_ESTIMATION_MODE_ENVIRONMENTAL_HDR;
}

std::unique_ptr<HdrLighting> DeeplightController::GetHdrLighting(
    ArSession* session) {
  IMP_TRACE();
  // We only return new scene lighting if we have another cubemap.
  if (ar_image_cubemap_[0] == nullptr) {
    return nullptr;
  }

  uint8_t* buffer =
      new uint8_t[kCubemapFaceSize * kCubemapFaceSize * kCubemapFaceCount *
                  sizeof(filament::math::half4)];

  for (int i = 0; i < kCubemapFaceCount; ++i) {
    int32_t width = 0;
    int32_t height = 0;
    ArImage_getWidth(session, ar_image_cubemap_[i], &width);
    ArImage_getHeight(session, ar_image_cubemap_[i], &height);
    if (width != kCubemapFaceSize || height != kCubemapFaceSize) {
      IMP_LOG(imp::FATAL) << "Expected " << kCubemapFaceSize << "x" << kCubemapFaceSize
                 << " cubemap but found " << width << "x" << height;
    }

    ArImageFormat format = AR_IMAGE_FORMAT_INVALID;
    ArImage_getFormat(session, ar_image_cubemap_[i], &format);
    if (format != AR_IMAGE_FORMAT_RGBA_FP16) {
      IMP_LOG(imp::FATAL) << "Expected RGBA_FP16 cubemap but got format: " << format;
    }

    int32_t num_planes = 0;
    ArImage_getNumberOfPlanes(session, ar_image_cubemap_[i], &num_planes);
    if (num_planes != 1) {
      IMP_LOG(imp::FATAL) << "Expected 1 plane per face in cubemap but got "
                 << num_planes;
    }

    const uint8_t* plane_data = nullptr;
    int32_t plane_data_length = 0;
    ArImage_getPlaneData(session, ar_image_cubemap_[i], 0, &plane_data,
                         &plane_data_length);
    if (plane_data_length !=
        kCubemapFaceSize * kCubemapFaceSize * sizeof(filament::math::half4)) {
      IMP_LOG(imp::FATAL) << "Expected "
                 << (kCubemapFaceSize * kCubemapFaceSize *
                     sizeof(filament::math::half4))
                 << " bytes for plane data plane per face in cubemap but got "
                 << plane_data_length;
    }

    int32_t pixel_stride = 0;
    ArImage_getPlanePixelStride(session, ar_image_cubemap_[i], 0,
                                &pixel_stride);
    if (pixel_stride != sizeof(filament::math::half4)) {
      IMP_LOG(imp::FATAL) << "Expected " << sizeof(filament::math::half4)
                 << " pixel stride in cubemap image " << i << " but got "
                 << pixel_stride;
    }

    int32_t row_stride = 0;
    ArImage_getPlaneRowStride(session, ar_image_cubemap_[i], 0, &row_stride);
    if (row_stride != kCubemapFaceSize * sizeof(filament::math::half4)) {
      IMP_LOG(imp::FATAL) << "Expected "
                 << kCubemapFaceSize * sizeof(filament::math::half4)
                 << " row stride in cubemap image " << i << " but got "
                 << row_stride;
    }

    memcpy(&buffer[kCubemapFaceSize * kCubemapFaceSize *
                   sizeof(filament::math::half4) * i],
           plane_data, plane_data_length);
  }

  // Release ar images since we won't use them again.
  ReleaseCubemap();

  filament::backend::PixelBufferDescriptor cubemap_pixel_buffer(
      buffer,
      kCubemapFaceSize * kCubemapFaceSize * kCubemapFaceCount *
          sizeof(filament::math::half4),
      filament::backend::PixelDataFormat::RGBA,
      filament::backend::PixelDataType::HALF,
      [](void* buffer, size_t size, void* user) {
        uint8_t* cubemap_buffer = reinterpret_cast<uint8_t*>(user);
        delete[] cubemap_buffer;
      },
      reinterpret_cast<void*>(buffer));

  // Process and fill cubemaps.
  const uint8_t levels = 1 + std::log2(kCubemapFaceSize);
  filament::Texture* cubemap_texture =
      filament::Texture::Builder{}
          .width(kCubemapFaceSize)
          .height(kCubemapFaceSize)
          .levels(levels)
          .sampler(filament::Texture::Sampler::SAMPLER_CUBEMAP)
          .format(filament::Texture::InternalFormat::R11F_G11F_B10F)
          .build(*engine_);

  filament::Texture::PrefilterOptions options;
  options.mirror = false;
  cubemap_texture->generatePrefilterMipmap(
      *engine_, std::move(cubemap_pixel_buffer),
      filament::Texture::FaceOffsets(kCubemapFaceSize * kCubemapFaceSize *
                                     sizeof(filament::math::half4)),
      &options);

  filament::IndirectLight* indirect_light =
      filament::IndirectLight::Builder{}
          .irradiance(3, irradiance_data_)
          .intensity(kDeeplightIntensityScale)
          .reflections(cubemap_texture)
          .build(*engine_);

  auto hdr_lighting = std::make_unique<HdrLighting>();
  hdr_lighting->environment_light =
      env_light_factory_->WrapIndirectLight(indirect_light);

  hdr_lighting->directional_light_info.emplace(
      HdrLighting::DirectionalLightInfo{
          .color = main_light_color_,
          .intensity = kDeeplightIntensityScale * main_light_intensity_scalar_,
          .direction = main_light_direction_ * -1.0f,
          .cast_shadow = true});

  return hdr_lighting;
}

float4 DeeplightController::GetAmbientLighting(ArSession* session) {
  return ambient_light_estimate_ * float4{1, 1, 1, kDeeplightIntensityScale};
}

}  // namespace imp::ar
