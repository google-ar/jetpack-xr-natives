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
  shadingModel: unlit,
  {BLENDING}
  flipUV: false,
  {DOUBLE_SIDED}
  parameters: [
    // Generic list of available samplers, assignable to different usages.
    // Unlit materials only have one, but this structure is used for
    // consistency with lit materials so that no branching is needed in code
    // between lit/unlit materials.
    { type : sampler2d, name : samplerZero },

    {AR_OCCLUSION_SAMPLERS}

    // Bitflags used to determine if UV0 or UV1 should be used for each sampler.
    // If a bit is 0, UV0 is used. If a bit is 1, UV1 is used.
    { type : int, name : samplers_uv_bitflags },

    // Indexed array of matrices used to transform the uv coords of samplers.
    // This is used to implement KHR_texture_transform.
    // Needs to be 2 because glsl doesn't support arrays of 1.
    { type : mat3[2], name : samplers_uv_matrices },

    // Base Color
    { type : int, name : baseColorIndex, precision: low },
    { type : float4, name : baseColorFactor },

    // ARCore and ARKit Occlusions
    { type : float3, name : backgroundUvFromNdc0 },
    { type : float3,  name : backgroundUvFromNdc1 },
    { type : float, name : depthTolerancePerMm },
    { type : float, name : maximumOcclusionFactor },
    { type : float, name : edgeBlur },
    { type : float, name : depthTextureAspectRatio },
    { type : float, name : fadeFactor }
  ],
  requires: [
    color,
    position,
    uv0,
    uv1
  ],
  variables: [ renderedDepth ],
  featureLevel: 0
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

  highp vec4 getSample(lowp int samplerIndex)
  {
    // ESSL 1.0 does not support switch statements.
    if (samplerIndex == 0) {
      vec2 uv = uvForSampler(samplerIndex);
      return texture2D(materialParams_samplerZero, uv);
    } else if (samplerIndex == 16) {
      return whiteFallbackSample();
    } else if (samplerIndex == 17) {
      return normalFallbackSample();
    }
  }

  void material(inout MaterialInputs material) {
    prepareMaterial(material);

    // Base Color
    // TODO: Filament doesn't crash when required attributes are
    // missing: See RenderableManager::Builder::build
    // This means that if the vertex doesn't have any color set, it will
    // output all 0. (or some garbage value)
    material.baseColor = getColor();
    material.baseColor *= materialParams.baseColorFactor;
    material.baseColor *= getSample(materialParams.baseColorIndex);
#if defined(BLEND_MODE_TRANSPARENT)
    material.baseColor.rgb *= material.baseColor.a;
#endif

#if defined(AR_OCCLUSION)
    vec3 backgroundColor = vec3(0.0);
    vec2 backgroundUv = vec2(0.0);
    if (materialParams.maximumOcclusionFactor > 0.0
        || materialParams.fadeFactor > 0.0) {
      backgroundUv = getBackgroundUvFromWorldPosition(getWorldPosition());
      vec3 backgroundSample = texture2D(materialParams_cameraTexture,
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
