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

#include "core/material_library/flatbuffer_utils.h"

#include "filament/filament/include/filament/TextureSampler.h"
#include "core/common/type_helpers.h"
#include "core/material_library/schemas/generic_material_generated.h"

namespace imp {

namespace {

// Verify impress::schemas::MinFilter and filament::TextureSampler::MinFilter
// enums match.
static_assert(DoEnumsMatch(filament::TextureSampler::MinFilter::NEAREST,
                           schemas::MinFilter::NEAREST),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::TextureSampler::MinFilter::LINEAR,
                           schemas::MinFilter::LINEAR),
              "Enum mismatch");
static_assert(
    DoEnumsMatch(filament::TextureSampler::MinFilter::NEAREST_MIPMAP_NEAREST,
                 schemas::MinFilter::NEAREST_MIPMAP_NEAREST),
    "Enum mismatch");
static_assert(
    DoEnumsMatch(filament::TextureSampler::MinFilter::LINEAR_MIPMAP_NEAREST,
                 schemas::MinFilter::LINEAR_MIPMAP_NEAREST),
    "Enum mismatch");
static_assert(
    DoEnumsMatch(filament::TextureSampler::MinFilter::NEAREST_MIPMAP_LINEAR,
                 schemas::MinFilter::NEAREST_MIPMAP_LINEAR),
    "Enum mismatch");
static_assert(
    DoEnumsMatch(filament::TextureSampler::MinFilter::LINEAR_MIPMAP_LINEAR,
                 schemas::MinFilter::LINEAR_MIPMAP_LINEAR),
    "Enum mismatch");
static_assert(schemas::MinFilter::MAX ==
                  schemas::MinFilter::LINEAR_MIPMAP_LINEAR,
              "New fields added but assert not updated");

// Verify imp::schemas::WrapMode and filament::TextureSampler::WrapMode
// enums match.
static_assert(DoEnumsMatch(filament::TextureSampler::WrapMode::REPEAT,
                           schemas::WrapMode::REPEAT),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::TextureSampler::WrapMode::CLAMP_TO_EDGE,
                           schemas::WrapMode::CLAMP_TO_EDGE),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::TextureSampler::WrapMode::MIRRORED_REPEAT,
                           schemas::WrapMode::MIRRORED_REPEAT),
              "Enum mismatch");
static_assert(schemas::WrapMode::MAX == schemas::WrapMode::MIRRORED_REPEAT,
              "New fields added but assert not updated");

static_assert(DoEnumsMatch(filament::TextureSampler::CompareMode::NONE,
                           schemas::CompareMode::NONE),
              "Enum mismatch");
static_assert(
    DoEnumsMatch(filament::TextureSampler::CompareMode::COMPARE_TO_TEXTURE,
                 schemas::CompareMode::COMPARE_TO_TEXTURE),
    "Enum mismatch");
static_assert(schemas::CompareMode::MAX ==
                  schemas::CompareMode::COMPARE_TO_TEXTURE,
              "New fields added but assert not updated");

// Verify imp::schemas::CompareFunc and filament::TextureSampler::CompareFunc
// enums match.
static_assert(DoEnumsMatch(filament::TextureSampler::CompareFunc::LE,
                           schemas::CompareFunc::LE),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::TextureSampler::CompareFunc::GE,
                           schemas::CompareFunc::GE),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::TextureSampler::CompareFunc::L,
                           schemas::CompareFunc::L),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::TextureSampler::CompareFunc::G,
                           schemas::CompareFunc::G),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::TextureSampler::CompareFunc::E,
                           schemas::CompareFunc::E),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::TextureSampler::CompareFunc::NE,
                           schemas::CompareFunc::NE),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::TextureSampler::CompareFunc::A,
                           schemas::CompareFunc::A),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::TextureSampler::CompareFunc::N,
                           schemas::CompareFunc::N),
              "Enum mismatch");
static_assert(schemas::CompareFunc::MAX == schemas::CompareFunc::N,
              "New fields added but assert not updated");

}  // namespace

}  // namespace imp
