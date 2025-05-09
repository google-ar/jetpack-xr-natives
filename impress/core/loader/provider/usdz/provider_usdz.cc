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

#include "core/loader/provider/usdz/provider_usdz.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/common/buffer_access.h"
#include "core/common/flatbuffer_helpers.h"
#include "core/common/robin_map.h"
#include "core/common/typed_id.h"
#include "core/loader/provider/details/provider_details_common.h"
#include "core/loader/provider/details/usdz_provider.h"
#include "core/loader/provider/schemas/loaded_model_generated.h"
#include "core/loader/provider/usdz/usdz_consumer.h"
#include "core/model/model_data.h"
#include "third_party/tinyusdz/src/stage.hh"
#include "third_party/tinyusdz/src/tinyusdz.hh"

namespace imp::loader::details::provider_usdz {
namespace {
using MinFilter = schemas::MinFilter;
using MagFilter = schemas::MagFilter;
using WrapMode = schemas::WrapMode;
using WeakSkinId = imp::TypedId<model::ModelData::SkinData, int>;
using ::flatbuffers::FlatBufferBuilder;

class TinyUsdzProvider : public UsdzProvider {
 public:
  ~TinyUsdzProvider() override {}

  // Attempt to parse the gltf file, retrieving a tinygltf::Model.  This can
  // be used in subsequent calls to TryLoadGltf.
  absl::Status TryParseUsdz(LoaderState *state) override;

  bool IsParsed() override;

  // Check a successful parsed model for pending resources.  This allows us to
  // flag missing resources encountered during a successful parsing, and
  // update the tracking information that allows us to maintain those links
  // when we attempt to load.
  bool HasPendingResources(LoaderState *state) override;

  // Attempt a single usdz load.
  absl::StatusOr<FlatBufferAccess<schemas::LoadedModel>> TryLoadUsdz(
      LoaderState *state) override;

 private:
  std::optional<tinyusdz::Stage> stage_;
};

class PkzipLocalHeader {
 public:
  PkzipLocalHeader(const uint8_t *addr) : addr_(addr) {}
  template <typename T>
  T Get(size_t offset) const {
    T result;
    memcpy(&result, addr_ + offset, sizeof(T));
    return result;
  }
  uint16_t Get16(size_t offset) const { return Get<uint16_t>(offset); }
  uint32_t Get32(size_t offset) const { return Get<uint32_t>(offset); }

 private:
  const uint8_t *addr_;
};

absl::StatusOr<RobinMap<std::string, BufferAccess>> CollectResources(
    const uint8_t *addr, size_t length) {
  RobinMap<std::string, BufferAccess> resources;
  size_t offset = 0;
  // See (broken link)
  constexpr auto kPkzipLocalHeaderLength = 30;
  while ((offset + kPkzipLocalHeaderLength) < length) {
    constexpr auto kSignatureOffset = 0;
    constexpr auto kCompressedSizeOffset = 18;
    constexpr auto kUncompressedSizeOffset = 22;
    constexpr auto kNameLengthOffset = 26;
    constexpr auto kExtraLengthOffset = 28;
    constexpr auto kExpectedSignature = 0x04034b50;
    auto header = PkzipLocalHeader(addr + offset);
    offset += kPkzipLocalHeaderLength;

    uint32_t signature = header.Get32(kSignatureOffset);
    if (signature != kExpectedSignature) {
      if (!offset) return absl::InternalError("invalid file");
      // Otherwise we're past the PkZip portion of the file
      break;
    }
    uint16_t name_length = header.Get16(kNameLengthOffset);
    uint16_t extra_length = header.Get16(kExtraLengthOffset);
    uint32_t uncompressed_size = header.Get32(kUncompressedSizeOffset);
    uint32_t compressed_size = header.Get32(kCompressedSizeOffset);

    std::string name(name_length, ' ');
    memcpy(&name[0], addr + offset, name_length);
    offset += name_length;
    offset += extra_length;

    // No compression or ZIP64 support.
    if ((compressed_size != uncompressed_size) ||
        (compressed_size == 0xffffffff))
      return absl::InternalError("Invalid usdz file");

    resources[name] = BufferAccess::Wrap(addr + offset, compressed_size);

    offset += compressed_size;
  }
  return resources;
}

absl::Status TinyUsdzProvider::TryParseUsdz(LoaderState *state_ptr) {
  tinyusdz::Stage stage;
  std::string warn;
  std::string err;
  const auto options = tinyusdz::USDLoadOptions();

  std::string filename =
      absl::StrCat(state_ptr->basename_, state_ptr->extension_);

  bool result = LoadUSDFromMemory(state_ptr->primary_resource_.Data(),
                                  state_ptr->primary_resource_.Size(), filename,
                                  &stage, &warn, &err, options);
  if (!result) {
    return absl::InternalError(err.empty() ? std::string("Failed to parse USDZ")
                                           : err);
  }

  stage_.emplace(std::move(stage));

  return absl::OkStatus();
}

bool TinyUsdzProvider::IsParsed() { return stage_.has_value(); }

bool TinyUsdzProvider::HasPendingResources(LoaderState *state_ptr) {
  return false;
}

absl::StatusOr<FlatBufferAccess<schemas::LoadedModel>>
TinyUsdzProvider::TryLoadUsdz(LoaderState *state_ptr) {
  absl::StatusOr<RobinMap<std::string, BufferAccess>> resources =
      CollectResources(state_ptr->primary_resource_.Data(),
                       state_ptr->primary_resource_.Size());
  if (!resources.ok()) return resources.status();
  auto usdz_consumer = UsdzConsumer(*stage_, std::move(resources.value()));
  return usdz_consumer.BuildLoadedModel();
}

}  // namespace

std::unique_ptr<UsdzProvider> CreateUsdzProvider() {
  return std::make_unique<TinyUsdzProvider>();
}

}  // namespace imp::loader::details::provider_usdz
