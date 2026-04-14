#include "split_engine/materials/gsplat_background_material_serializer.h"

#include <memory>
#include <utility>

#include "absl/memory/memory.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/async/future.h"
#include "core/materials/material.h"
#include "core/split_engine/materials/builtin_texture_parameter_creator.h"
#include "core/split_engine/materials/split_engine_builtin_material.h"
#include "core/view/base_view.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace android_xr {

imp::Future<std::unique_ptr<GsplatBackgroundMaterialSerializer>>
GsplatBackgroundMaterialSerializer::Create(imp::BaseView& view) {
  auto fbb = std::make_unique<flatbuffers::FlatBufferBuilder>();
  flatbuffers::Offset<android_xr::schemas::BuiltInMaterialGsplatBackgroundSpec>
      spec_offset =
          android_xr::schemas::CreateBuiltInMaterialGsplatBackgroundSpec(*fbb);
  return RequestBuiltInMaterial(view, std::move(fbb),
                                android_xr::schemas::BuiltInMaterialSpec::
                                    BuiltInMaterialGsplatBackgroundSpec,
                                spec_offset.Union())
      .Then([&view](imp::OwnedMaterialPtr material) {
        return absl::WrapUnique(
            new GsplatBackgroundMaterialSerializer(view, std::move(material)));
      });
}

GsplatBackgroundMaterialSerializer::GsplatBackgroundMaterialSerializer(
    imp::BaseView& view, imp::OwnedMaterialPtr material)
    : SplitEngineBuiltinMaterial(
          view,
          android_xr::schemas::BuiltInMaterialParameters::
              BuiltInMaterialGsplatBackgroundParameters,
          std::move(material)) {}

GsplatBackgroundMaterialSerializer::~GsplatBackgroundMaterialSerializer() {
  Cleanup();
}

flatbuffers::Offset<void>
GsplatBackgroundMaterialSerializer::SerializeParameters(
    flatbuffers::FlatBufferBuilder& fbb,
    imp::split_engine::BuiltInTextureParameterCreator&
        texture_parameter_creator) const {
  return android_xr::schemas::CreateBuiltInMaterialGsplatBackgroundParameters(
             fbb)
      .Union();
}

}  // namespace android_xr
