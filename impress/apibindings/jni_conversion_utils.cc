// Copyright 2025 Google LLC
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

#include "apibindings/jni_conversion_utils.h"

#include <cstdint>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "core/material_library/generic_material_spec.h"
#include "core/material_library/schemas/generic_material_generated.h"

namespace {

// LINT.IfChange(generic_material_spec_jni_conversion_bindings)

absl::StatusOr<imp::schemas::GenericMaterialLightingModel>
ToGenericMaterialLightingModel(int lighting_model_value) {
  static const absl::flat_hash_set<uint8_t> valid_standards = {
      static_cast<uint8_t>(imp::schemas::GenericMaterialLightingModel::Lit),
      static_cast<uint8_t>(imp::schemas::GenericMaterialLightingModel::Unlit),
  };

  if (valid_standards.contains(static_cast<uint8_t>(lighting_model_value))) {
    return static_cast<imp::schemas::GenericMaterialLightingModel>(
        lighting_model_value);
  } else {
    return absl::InvalidArgumentError(absl::StrFormat(
        "Invalid lighting model provided: %d", lighting_model_value));
  }
}

absl::StatusOr<imp::schemas::GenericMaterialBlendMode>
ToGenericMaterialBlendMode(int blend_mode_value) {
  static const absl::flat_hash_set<uint8_t> valid_standards = {
      static_cast<uint8_t>(imp::schemas::GenericMaterialBlendMode::Opaque),
      static_cast<uint8_t>(imp::schemas::GenericMaterialBlendMode::Masked),
      static_cast<uint8_t>(imp::schemas::GenericMaterialBlendMode::Transparent),
      static_cast<uint8_t>(imp::schemas::GenericMaterialBlendMode::Refractive),
  };

  if (valid_standards.contains(static_cast<uint8_t>(blend_mode_value))) {
    return static_cast<imp::schemas::GenericMaterialBlendMode>(
        blend_mode_value);
  } else {
    return absl::InvalidArgumentError(
        absl::StrFormat("Invalid blend mode provided: %d", blend_mode_value));
  }
}

absl::StatusOr<imp::schemas::GenericMaterialDoubleSidedMode>
ToGenericMaterialDoubleSidedMode(int double_sided_mode_value) {
  static const absl::flat_hash_set<uint8_t> valid_standards = {
      static_cast<uint8_t>(
          imp::schemas::GenericMaterialDoubleSidedMode::SingleSided),
      static_cast<uint8_t>(
          imp::schemas::GenericMaterialDoubleSidedMode::DoubleSided),
  };

  if (valid_standards.contains(static_cast<uint8_t>(double_sided_mode_value))) {
    return static_cast<imp::schemas::GenericMaterialDoubleSidedMode>(
        double_sided_mode_value);
  } else {
    return absl::InvalidArgumentError(absl::StrFormat(
        "Invalid double sided mode provided: %d", double_sided_mode_value));
  }
}

// LINT.ThenChange(//depot/google3/third_party/split_engine/schemas/split_engine_material.fbs:generic_material_spec_jni_conversion_schema)

// LINT.IfChange(texture_sampler_jni_conversion_bindings)

absl::StatusOr<filament::TextureSampler::MinFilter> ToFilamentMinFilter(
    int min_filter_value) {
  static const absl::flat_hash_set<uint8_t> valid_standards = {
      static_cast<uint8_t>(filament::TextureSampler::MinFilter::LINEAR),
      static_cast<uint8_t>(filament::TextureSampler::MinFilter::NEAREST),
      static_cast<uint8_t>(
          filament::TextureSampler::MinFilter::LINEAR_MIPMAP_LINEAR),
      static_cast<uint8_t>(
          filament::TextureSampler::MinFilter::LINEAR_MIPMAP_NEAREST),
      static_cast<uint8_t>(
          filament::TextureSampler::MinFilter::NEAREST_MIPMAP_LINEAR),
      static_cast<uint8_t>(
          filament::TextureSampler::MinFilter::NEAREST_MIPMAP_NEAREST),
  };

  if (valid_standards.contains(static_cast<uint8_t>(min_filter_value))) {
    return static_cast<filament::TextureSampler::MinFilter>(min_filter_value);
  } else {
    return absl::InvalidArgumentError(
        absl::StrFormat("Invalid min filter provided: %d", min_filter_value));
  }
}

absl::StatusOr<filament::TextureSampler::MagFilter> ToFilamentMagFilter(
    int mag_filter_value) {
  static const absl::flat_hash_set<uint8_t> valid_standards = {
      static_cast<uint8_t>(filament::TextureSampler::MagFilter::LINEAR),
      static_cast<uint8_t>(filament::TextureSampler::MagFilter::NEAREST),
  };

  if (valid_standards.contains(static_cast<uint8_t>(mag_filter_value))) {
    return static_cast<filament::TextureSampler::MagFilter>(mag_filter_value);
  } else {
    return absl::InvalidArgumentError(
        absl::StrFormat("Invalid mag filter provided: %d", mag_filter_value));
  }
}

absl::StatusOr<filament::TextureSampler::WrapMode> ToFilamentWrapMode(
    int wrap_mode_value) {
  static const absl::flat_hash_set<uint8_t> valid_standards = {
      static_cast<uint8_t>(filament::TextureSampler::WrapMode::CLAMP_TO_EDGE),
      static_cast<uint8_t>(filament::TextureSampler::WrapMode::MIRRORED_REPEAT),
      static_cast<uint8_t>(filament::TextureSampler::WrapMode::REPEAT),
  };

  if (valid_standards.contains(static_cast<uint8_t>(wrap_mode_value))) {
    return static_cast<filament::TextureSampler::WrapMode>(wrap_mode_value);
  } else {
    return absl::InvalidArgumentError(
        absl::StrFormat("Invalid wrap mode provided: %d", wrap_mode_value));
  }
}

absl::StatusOr<filament::TextureSampler::CompareMode> ToFilamentCompareMode(
    int compare_mode_value) {
  static const absl::flat_hash_set<uint8_t> valid_standards = {
      static_cast<uint8_t>(filament::TextureSampler::CompareMode::NONE),
      static_cast<uint8_t>(
          filament::TextureSampler::CompareMode::COMPARE_TO_TEXTURE),
  };

  if (valid_standards.contains(static_cast<uint8_t>(compare_mode_value))) {
    return static_cast<filament::TextureSampler::CompareMode>(
        compare_mode_value);
  } else {
    return absl::InvalidArgumentError(absl::StrFormat(
        "Invalid compare mode provided: %d", compare_mode_value));
  }
}

absl::StatusOr<filament::TextureSampler::CompareFunc> ToFilamentCompareFunc(
    int compare_func_value) {
  static const absl::flat_hash_set<uint8_t> valid_standards = {
      static_cast<uint8_t>(filament::TextureSampler::CompareFunc::A),
      static_cast<uint8_t>(filament::TextureSampler::CompareFunc::E),
      static_cast<uint8_t>(filament::TextureSampler::CompareFunc::G),
      static_cast<uint8_t>(filament::TextureSampler::CompareFunc::GE),
      static_cast<uint8_t>(filament::TextureSampler::CompareFunc::L),
      static_cast<uint8_t>(filament::TextureSampler::CompareFunc::LE),
      static_cast<uint8_t>(filament::TextureSampler::CompareFunc::N),
      static_cast<uint8_t>(filament::TextureSampler::CompareFunc::NE),
  };

  if (valid_standards.contains(static_cast<uint8_t>(compare_func_value))) {
    return static_cast<filament::TextureSampler::CompareFunc>(
        compare_func_value);
  } else {
    return absl::InvalidArgumentError(absl::StrFormat(
        "Invalid compare func provided: %d", compare_func_value));
  }
}

// LINT.ThenChange(//depot/google3/third_party/split_engine/schemas/split_engine_material.fbs:texture_sampler_jni_conversion_schema)

}  // namespace

namespace imp {

absl::StatusOr<imp::GenericMaterialSpec> BuildGenericMaterialSpecFromValues(
    jint lighting_model_value, jint blend_mode_value,
    jint double_sided_mode_value) {
  // Map the Java enum values to the corresponding Impress Split Engine enum
  // constants.
  absl::StatusOr<imp::schemas::GenericMaterialLightingModel> lighting_model =
      ToGenericMaterialLightingModel(lighting_model_value);
  if (!lighting_model.ok()) {
    return lighting_model.status();
  }
  absl::StatusOr<imp::schemas::GenericMaterialBlendMode> blend_mode =
      ToGenericMaterialBlendMode(blend_mode_value);
  if (!blend_mode.ok()) {
    return blend_mode.status();
  }
  absl::StatusOr<imp::schemas::GenericMaterialDoubleSidedMode>
      double_sided_mode =
          ToGenericMaterialDoubleSidedMode(double_sided_mode_value);
  if (!double_sided_mode.ok()) {
    return double_sided_mode.status();
  }

  // Depth clear is a hack that was added for Geo and is not needed for the
  // API bindings layer, so we set that field of the spec to disabled.
  return imp::GenericMaterialSpec(
      *lighting_model, *blend_mode, *double_sided_mode,
      imp::schemas::GenericMaterialDepthClearMaterial::Disabled);
}

absl::StatusOr<filament::TextureSampler> BuildTextureSamplerFromValues(
    jint min_filter_value, jint mag_filter_value, jint wrap_mode_s_value,
    jint wrap_mode_t_value, jint wrap_mode_r_value, jint compare_mode_value,
    jint compare_func_value, jint anisotropy_log2_value) {
  // Map the Java enum values to the corresponding Filament enum constants.
  absl::StatusOr<filament::TextureSampler::MinFilter> min_filter =
      ToFilamentMinFilter(min_filter_value);
  if (!min_filter.ok()) {
    return min_filter.status();
  }
  absl::StatusOr<filament::TextureSampler::MagFilter> mag_filter =
      ToFilamentMagFilter(mag_filter_value);
  if (!mag_filter.ok()) {
    return mag_filter.status();
  }
  absl::StatusOr<filament::TextureSampler::WrapMode> wrap_mode_s =
      ToFilamentWrapMode(wrap_mode_s_value);
  if (!wrap_mode_s.ok()) {
    return wrap_mode_s.status();
  }
  absl::StatusOr<filament::TextureSampler::WrapMode> wrap_mode_t =
      ToFilamentWrapMode(wrap_mode_t_value);
  if (!wrap_mode_t.ok()) {
    return wrap_mode_t.status();
  }
  absl::StatusOr<filament::TextureSampler::WrapMode> wrap_mode_r =
      ToFilamentWrapMode(wrap_mode_r_value);
  if (!wrap_mode_r.ok()) {
    return wrap_mode_r.status();
  }
  absl::StatusOr<filament::TextureSampler::CompareMode> compare_mode =
      ToFilamentCompareMode(compare_mode_value);
  if (!compare_mode.ok()) {
    return compare_mode.status();
  }
  absl::StatusOr<filament::TextureSampler::CompareFunc> compare_func =
      ToFilamentCompareFunc(compare_func_value);
  if (!compare_func.ok()) {
    return compare_func.status();
  }

  filament::TextureSampler native_sampler(*min_filter, *mag_filter);
  native_sampler.setWrapModeS(*wrap_mode_s);
  native_sampler.setWrapModeT(*wrap_mode_t);
  native_sampler.setWrapModeR(*wrap_mode_r);
  native_sampler.setCompareMode(*compare_mode, *compare_func);
  native_sampler.setAnisotropy(static_cast<float>(1 << anisotropy_log2_value));

  return native_sampler;
}

}  // namespace imp
