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

#include <cstddef>
#include <memory>
#include <ostream>
#include <sstream>
#include <string>
#include <utility>

#include "zetasql/base/init_google.h"
#include "absl/flags/flag.h"
#include "core/common/log.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_split.h"
#include "absl/strings/string_view.h"
#include "flatbuffers/vector.h"
#include "core/common/buffer_access.h"
#include "core/common/enum_flags.h"
#include "core/common/file_helpers.h"
#include "core/common/flatbuffer_helpers.h"
#include "core/common/optional_error.h"
#include "core/common/resource_helpers.h"
#include "core/common/schemas/math_generated.h"
#include "core/common/schemas/render_generated.h"
#include "core/loader/data/embedded_imp_default_gltf_materials.h"
#include "core/loader/data/embedded_placeholder_textures.h"
#include "core/loader/loader_options.h"
#include "core/loader/provider/extensions/verification.h"
#include "core/loader/provider/provider.h"
#include "core/loader/provider/schemas/loaded_model_generated.h"
#include "core/loader/provider/usdz/provider_usdz.h"
#include "core/material_library/schemas/generic_material_generated.h"
#include "mediapipe/framework/port/status_macros.h"

ABSL_FLAG(bool, info, false, "Print info about the model");
ABSL_FLAG(std::string, save_flatbuffer, "",
          "If provided, convert to flatbuffer and save to path");

namespace imp::loader {

namespace {

std::ostream& PrintInfo(std::ostream& stream,
                        const FlatBufferAccess<schemas::LoadedModel>& model) {
  stream << absl::StrFormat("Flatbuffer size: %llu\n", model.Buffer().Size());
  stream << absl::StrFormat(
      "Entities: %llu\n",
      flatbuffers::VectorLength(model->entity_graph()->entities()));

  const auto* images = model->images();
  const auto* textures = model->textures();

  for (int image_index = 0; image_index < images->size(); image_index++) {
    const schemas::TextureInfo* texture = textures->Get(image_index);
    const schemas::ImageFileData* image =
        images->GetAs<schemas::ImageFileData>(image_index);

    stream << absl::StrFormat(
        " - Image: '%s' bytes: %d\n", texture->name()->str().c_str(),
        static_cast<int>(flatbuffers::VectorLength(image->buffer())));

    Flags<schemas::TextureInfoFlags> flags = ToFlags(texture->flags());
    if (flags.Test(schemas::TextureInfoFlags::IsSrgb)) {
      stream << absl::StrFormat(
          "   - flags: +%s\n",
          schemas::EnumNameTextureInfoFlags(schemas::TextureInfoFlags::IsSrgb));
    }
    if (flags.Test(schemas::TextureInfoFlags::GenerateMips)) {
      stream << absl::StrFormat("   - flags: +%s\n",
                                schemas::EnumNameTextureInfoFlags(
                                    schemas::TextureInfoFlags::GenerateMips));
    }
    if (flags.Test(schemas::TextureInfoFlags::IsR11G11B10)) {
      stream << absl::StrFormat("   - flags: +%s\n",
                                schemas::EnumNameTextureInfoFlags(
                                    schemas::TextureInfoFlags::IsR11G11B10));
    }
    if (flags.Test(schemas::TextureInfoFlags::IsLookup)) {
      stream << absl::StrFormat("   - flags: +%s\n",
                                schemas::EnumNameTextureInfoFlags(
                                    schemas::TextureInfoFlags::IsLookup));
    }
  }

  const auto* vertex_buffers = model->vertex_buffers();
  for (const schemas::VertexBufferInfo* vertex_buffer : *vertex_buffers) {
    stream << absl::StrFormat(" - Vertex Buffer: '(%llu vertices)'\n",
                              vertex_buffer->vertex_count());
    const auto* blocks = vertex_buffer->blocks();
    for (const schemas::VertexBlockInfo* block : *blocks) {
      stream << absl::StrFormat(
          "   - Vertex block (%llu attributes, %llu bytes)\n",
          block->attributes()->size(), block->buffer()->size());
      const auto* attributes = block->attributes();
      for (const schemas::VertexAttributeInfo* attr : *attributes) {
        stream << absl::StrFormat(
            "     - Vertex Attribute: %s (%s) offset=%u normalized=%d\n",
            schemas::EnumNameVertexAttribute(attr->attribute()),
            schemas::EnumNameAttributeType(attr->type()), attr->offset(),
            attr->normalized());
      }
    }
  }

  const auto* index_buffers = model->index_buffers();
  for (const schemas::IndexBufferInfo* index_buffer : *index_buffers) {
    stream << absl::StrFormat(" - Index Buffer: '%s', %llu bytes\n",
                              schemas::EnumNameIndexType(index_buffer->type()),
                              index_buffer->buffer()->size());
  }

  const auto* materials = model->materials();
  for (const schemas::MaterialInfo* material : *materials) {
    stream << absl::StrFormat(" - Material: '%s'\n",
                              material->material()->name()->str());
    const schemas::GenericMaterialParameters* params =
        material->material()->params();
    if (params) {
      if (params->base_color()) {
        const auto* base_color = params->base_color();
        if (base_color->texture()) {
          stream << absl::StrFormat(
              "   - Param: 'base_color_texture' = (texture ID): %d\n",
              base_color->texture()->texture());
        }
        if (base_color->factor()) {
          stream << absl::StrFormat(
              "   - Param: 'base_color_factor' = (float4): %.3f %.3f %.3f "
              "%.3f\n",
              base_color->factor()->x(), base_color->factor()->y(),
              base_color->factor()->z(), base_color->factor()->w());
        }
      }
      if (params->metallic_roughness()) {
        const auto* metallic_roughness = params->metallic_roughness();
        if (metallic_roughness->texture()) {
          stream << absl::StrFormat(
              "   - Param: 'metallic_roughness_texture' = (texture ID): %d\n",
              metallic_roughness->texture()->texture());
        }
        if (metallic_roughness->metallic_factor()) {
          stream << absl::StrFormat(
              "   - Param: 'metallic_factor' = (float): %.3f\n",
              metallic_roughness->metallic_factor()->value());
        }
        if (metallic_roughness->roughness_factor()) {
          stream << absl::StrFormat(
              "   - Param: 'roughness_factor' = (float): %.3f\n",
              metallic_roughness->roughness_factor()->value());
        }
      }
      if (params->normal()) {
        const auto* normal = params->normal();
        if (normal->texture()) {
          stream << absl::StrFormat(
              "   - Param: 'normal_texture' = (texture ID): %d\n",
              normal->texture()->texture());
        }
        if (normal->factor()) {
          stream << absl::StrFormat(
              "   - Param: 'normal_scale' = (float): %.3f\n",
              normal->factor()->value());
        }
      }
      if (params->ambient_occlusion()) {
        const auto* ambient_occlusion = params->ambient_occlusion();
        if (ambient_occlusion->texture()) {
          stream << absl::StrFormat(
              "   - Param: 'ambient_occlusion_texture' = (texture ID): %d\n",
              ambient_occlusion->texture()->texture());
        }
        if (ambient_occlusion->factor()) {
          stream << absl::StrFormat(
              "   - Param: 'ambient_occlusion_strength' = (float): %.3f\n",
              ambient_occlusion->factor()->value());
        }
      }
      if (params->emissive()) {
        const auto* emissive = params->emissive();
        if (emissive->texture()) {
          stream << absl::StrFormat(
              "   - Param: 'emissive_texture' = (texture ID): %d\n",
              emissive->texture()->texture());
        }
        if (emissive->factor()) {
          stream << absl::StrFormat(
              "   - Param: 'emissive_factor' = (float3): %.3f %.3f %.3f\n",
              emissive->factor()->x(), emissive->factor()->y(),
              emissive->factor()->z());
        }
      }
      if (params->clearcoat()) {
        const auto* clearcoat = params->clearcoat();
        if (clearcoat->intensity_texture()) {
          stream << absl::StrFormat(
              "   - Param: 'clearcoat_intensity_texture' = (texture ID): %d\n",
              clearcoat->intensity_texture()->texture());
        }
        if (clearcoat->normal_texture()) {
          stream << absl::StrFormat(
              "   - Param: 'clearcoat_normal_texture' = (texture ID): %d\n",
              clearcoat->normal_texture()->texture());
        }
      }
      if (params->sheen()) {
        const auto* sheen = params->sheen();
        if (sheen->color_texture()) {
          stream << absl::StrFormat(
              "   - Param: 'sheen_color_texture' = (texture ID): %d\n",
              sheen->color_texture()->texture());
        }
        if (sheen->color_factor()) {
          stream << absl::StrFormat(
              "   - Param: 'sheen_color_factor' = (float3): %.3f %.3f %.3f\n",
              sheen->color_factor()->x(), sheen->color_factor()->y(),
              sheen->color_factor()->z());
        }
      }
    }
  }

  stream << absl::StrFormat("Textures: %llu\n",
                            flatbuffers::VectorLength(model->textures()));
  stream << absl::StrFormat("Vertex buffers: %llu\n",
                            flatbuffers::VectorLength(model->vertex_buffers()));

  size_t total_vertices = 0;
  for (const auto& buf : *model->vertex_buffers()) {
    total_vertices += buf->vertex_count();
  }
  stream << absl::StrFormat("Vertex count: %llu\n", total_vertices);

  return stream;
}

}  // namespace

class LoaderTool {
 public:
  explicit LoaderTool(absl::string_view filename) : filename_(filename) {}

  OptionalError Load() {
    if (loaded_) {
      IMP_LOG(imp::ERROR) << "Already loaded";
    }

    BufferAccess access;
    MP_RETURN_IF_ERROR(LoadBinary(filename_, &access));

    if (GetExtensionFromFilename(filename_) == ".flat") {
      MP_RETURN_IF_ERROR(
          optional_features::VerifyAndGetModel(std::move(access), &model_));
    } else {
      MP_ASSIGN_OR_RETURN(
          provider_,
          Provider::Create(filename_, std::move(access), LoaderOptions{},
                           Provider::CreateDefaultGltfProvider(),
                           details::provider_usdz::CreateUsdzProvider()));
      MP_RETURN_IF_ERROR(provider_->Load());
      MP_RETURN_IF_ERROR(provider_->GetLoadedModel(&model_));
    }

    loaded_ = true;
    return NoError();
  }

  OptionalError SaveFlatbuffer(absl::string_view output_file) {
    if (!loaded_) {
      IMP_LOG(imp::ERROR) << "Not loaded";
    }

    MP_RETURN_IF_ERROR(SaveBinary(output_file, model_.Buffer()));
    return NoError();
  }

  void PrintInfo() {
    if (!loaded_) {
      IMP_LOG(imp::ERROR) << "Not loaded";
    }
    std::stringstream stream;
    imp::loader::PrintInfo(stream, model_);
    std::string result = stream.str();
    for (auto line : absl::StrSplit(result, '\n')) {
      IMP_LOG(imp::INFO) << line;
    }
  }

 private:
  bool loaded_ = false;
  std::unique_ptr<Provider> provider_;
  FlatBufferAccess<schemas::LoadedModel> model_;

  std::string filename_;
};

}  // namespace imp::loader

int main(int argc, char** argv) {
  InitGoogle(argv[0], &argc, &argv, true);

  imp::RegisterPackagedResources(embedded_imp_default_gltf_materials_create());
  imp::RegisterPackagedResources(embedded_placeholder_textures_create());

  if (argc != 2) {
    IMP_LOG(imp::ERROR) << "Missing input file";
  }

  imp::loader::LoaderTool tool(argv[1]);
  if (auto error = tool.Load(); !error.ok()) {
    IMP_LOG(imp::INFO) << "Load error: " << error;
    return 1;
  }

  std::string save_flatbuffer = absl::GetFlag(FLAGS_save_flatbuffer);
  if (!save_flatbuffer.empty()) {
    if (auto error = tool.SaveFlatbuffer(save_flatbuffer); !error.ok()) {
      IMP_LOG(imp::INFO) << "Save flatbuffer error: " << error;
      return 2;
    }
  }

  if (absl::GetFlag(FLAGS_info)) {
    tool.PrintInfo();
  }

  return 0;
}
