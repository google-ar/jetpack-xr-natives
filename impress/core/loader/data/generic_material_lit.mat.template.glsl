// Copyright 2025 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

material {
  name: "{NAME}",
  constants : [
    {
      name : enableColorConversion,
      type : bool,
      default : false
    }
  ],
  shadingModel: lit,
  {BLENDING}
  flipUV: false,
  specularAmbientOcclusion: simple,
  {DOUBLE_SIDED}
  parameters: [
    // Generic list of available samplers, assignable to different usages.
    {REASSIGNABLE_SAMPLER_DECLARATION}

    {AR_OCCLUSION_SAMPLERS}

    // LINT.IfChange(generic_material_parameters)

    // Bitflags used to determine if UV0 or UV1 should be used for each sampler.
    // If a bit is 0, UV0 is used. If a bit is 1, UV1 is used.
    { type : int, name : samplers_uv_bitflags },

    // Indexed array of matrices used to transform the uv coords of samplers.
    // This is used to implement KHR_texture_transform.
    // The size of this array should match the reassignable samplers available.
    { type : mat3[{REASSIGNABLE_SAMPLER_COUNT}], name : samplers_uv_matrices },
    // LINT.ThenChange(//depot/google3/third_party/impress/core/material_library/generic_material_constants.h:generic_material_parameters)

    // Base Color
    { type : int, name : baseColorIndex, precision: low },
    { type : float4, name : baseColorFactor },

    // Metallic-Roughness Map
    { type : int, name : metallicRoughnessIndex, precision: low },
    { type : float, name : metallicFactor },
    { type : float, name : roughnessFactor },

    // Normal Map
    { type : int, name : normalIndex, precision: low },
    { type : float, name : normalScale },

    // Ambient Occlusion
    { type : int, name : aoIndex, precision: low },
    { type : float, name : aoStrength },

    // Emissive Map
    { type : int, name : emissiveIndex, precision: low },
    { type : float3, name : emissiveFactor },

    // KHR_materials_clearcoat
    { type : int, name : clearcoatIndex, precision: low },
    { type : int, name : clearcoatRoughnessIndex, precision: low },
    { type : int, name : clearcoatNormalIndex, precision: low },
    { type : float3, name : clearcoat_roughness_normal_factors },

    // KHR_materials_sheen
    { type : float3, name : sheenColorFactor },
    { type : int, name : sheenColorIndex, precision: low },
    { type : float, name : sheenRoughnessFactor },
    { type : int, name : sheenRoughnessIndex, precision: low },

    // KHR_materials_ior
    { type : float, name : indexOfRefraction },

    // KHR_materials_transmission
    { type : int, name : transmissionIndex, precision: low },
    { type : float, name : transmissionFactor },

    // ARCore and ARKit Occlusions
    { type : float3, name : backgroundUvFromNdc0 },
    { type : float3, name : backgroundUvFromNdc1 },
    { type : float, name : depthTolerancePerMm },
    { type : float, name : maximumOcclusionFactor },
    { type : float, name : edgeBlur },
    { type : float, name : depthTextureAspectRatio },
    { type : float, name : fadeFactor }
  ],
  requires: [
    color,
    position,
    tangents,
    uv0,
    uv1
  ],
  variables: [ renderedDepth ],
}

vertex {
  void materialVertex(inout MaterialVertexInputs material) {
    material.renderedDepth.x = -(getViewFromWorldMatrix() * getWorldPosition(material)).z;
  }
}

fragment {
  #include "generic_material_helpers.glsl"
#if defined(AR_OCCLUSION)
  #include "{OCCLUSION_HELPERS_GLSL}"
#endif
  #include "color_conversion_helpers.glsl"

// Get samples from all of the generically assignable texture samplers.
//
// These samplers are mapped to usages (i.e. baseColor) based on uniforms
// specifying an index into this table.
//
// The final two samples are fallbacks for usages that don't have a texture
// assigned to them. The last sample is for normals, last - 1 is white.
//
// For example, if there is no baseColorTexture, then
// baseColorIndex will be set to the index for the whiteFallbackSample.
highp vec4 getSample(lowp int samplerIndex) {
  // samplerIndex can be greater than the number of samplers in the case of
  // fallback samples, so we clamp it before calling uvForSampler. The UV does
  // not matter in the fallback case.
  lowp int clampedSamplerIndex =
      min(samplerIndex, {REASSIGNABLE_SAMPLER_COUNT} - 1);
  highp vec2 uv = uvForSampler(clampedSamplerIndex);

  switch (samplerIndex)  {
    {REASSIGNABLE_SAMPLER_SWITCH}
  }
}

  void material(inout MaterialInputs material) {
    // Normal Map
    vec3 normalSample = getSample(materialParams.normalIndex).xyz;
    material.normal = normalSample * 2.0 - 1.0;
    material.normal.xy *= materialParams.normalScale;

#if defined(CLEARCOAT)
    // KHR_materials_clearcoat: Normal
    float clearcoatNormalScale =
      materialParams.clearcoat_roughness_normal_factors.z;
    material.clearCoatNormal =
      getSample(materialParams.clearcoatNormalIndex).xyz * 2.0 - 1.0;
    material.clearCoatNormal.xy *= clearcoatNormalScale;
#endif

    prepareMaterial(material);

    // Base Color
    material.baseColor = getColor();
    material.baseColor *= materialParams.baseColorFactor;
    material.baseColor *= getSample(materialParams.baseColorIndex);
#if defined(BLEND_MODE_TRANSPARENT)
    material.baseColor.rgb *= material.baseColor.a;
#endif

    // Metallic-Roughness Map
    vec2 roughnessMetalnessSample =
      getSample(materialParams.metallicRoughnessIndex).yz;
    material.roughness =
      materialParams.roughnessFactor * roughnessMetalnessSample.x;
    material.metallic =
      materialParams.metallicFactor * roughnessMetalnessSample.y;

    // Ambient Occlusion
    material.ambientOcclusion =
      getSample(materialParams.aoIndex).x * materialParams.aoStrength;

    // Emissive Map
    material.emissive =
      vec4(getSample(materialParams.emissiveIndex).rgb *
           materialParams.emissiveFactor, 0.0);

#if defined(CLEARCOAT)
    // KHR_materials_clearcoat: Clearcoat and Roughness
    float clearcoatFactor =
      materialParams.clearcoat_roughness_normal_factors.x;
    float clearcoatRoughnessFactor =
      materialParams.clearcoat_roughness_normal_factors.y;
    material.clearCoat = clearcoatFactor *
      getSample(materialParams.clearcoatIndex).r;
    material.clearCoatRoughness = clearcoatRoughnessFactor *
      getSample(materialParams.clearcoatRoughnessIndex).g;
#endif

#if defined(SHEEN)
    // KHR_materials_sheen
    material.sheenColor = materialParams.sheenColorFactor *
      getSample(materialParams.sheenColorIndex).rgb;
    material.sheenRoughness = materialParams.sheenRoughnessFactor *
      getSample(materialParams.sheenRoughnessIndex).a;
#endif

#if defined(MATERIAL_HAS_REFRACTION)
    // KHR_materials_ior
    material.ior = materialParams.indexOfRefraction;
    // TODO: this is the correct default, but this is meant to be a
    // user-accessible parameter, as a part of KHR_materials_volume
    material.thickness = 0.0;

#if defined(TRANSMISSION)
    // KHR_materials_transmission
    material.transmission = materialParams.transmissionFactor *
      getSample(materialParams.transmissionIndex).r;
#endif
#endif //  defined(MATERIAL_HAS_REFRACTION)

#if defined(AR_OCCLUSION)
    vec3 backgroundColor = vec3(0.0);
    vec2 backgroundUv = vec2(0.0);
     if (materialParams.maximumOcclusionFactor > 0.0
         || materialParams.fadeFactor > 0.0) {
      backgroundUv = getBackgroundUvFromWorldPosition(getWorldPosition());
      vec3 backgroundSample = texture(materialParams_cameraTexture,
                                      backgroundUv).xyz;
      backgroundColor.xyz = inverseTonemap(srgbToLinear(backgroundSample.xyz));
    }

    if (materialParams.fadeFactor > 0.0) {
      material.postLightingColor =
          vec4(materialParams.fadeFactor * backgroundColor.xyz,
               materialParams.fadeFactor);
    } else if (materialParams.maximumOcclusionFactor > 0.0) {
      float renderedDepth = variable_renderedDepth.x;
      float visibility = getVisibility(renderedDepth, backgroundUv);
      material.postLightingColor =
        vec4((1.0 - visibility) * backgroundColor.xyz, 1.0-visibility);
    }
#endif

  if (materialConstants_enableColorConversion) {
    material.baseColor.xyz = applyGlobalMaterialConversionMatrix(material.baseColor.xyz);
  }

  }
}
