/*
 * Copyright 2025 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.google.ar.imp.apibindings;

import com.google.errorprone.annotations.CanIgnoreReturnValue;

/**
 * KhronosPbrMaterialSpec class used to define the Khronos PBR material spec to be used when
 * creating the material.
 *
 * @hide
 */
public final class KhronosPbrMaterialSpec {
  // LINT.IfChange(khronos_pbr_material_spec_conversion_bindings)

  /** Lighting model to use for the material. Defines how the material interacts with light. */
  public enum LightingModel {
    /** The material responds to light sources using PBR principles. */
    LIT(0),
    /** The material does not respond to light sources. */
    UNLIT(1);

    final int lightingModel;

    LightingModel(int lightingModel) {
      this.lightingModel = lightingModel;
    }

    // Returns the int value of the lighting model.
    public int getValue() {
      return lightingModel;
    }
  }

  /**
   * Blend mode to use for the material. Determines how the material is blended with the background.
   */
  public enum BlendMode {
    /** The material is fully opaque and occludes objects behind it. */
    OPAQUE(0),
    /** Blend mode for transparency with dithering. */
    MASKED(1),
    /** The material allows light to pass through, based on its alpha value. */
    TRANSPARENT(2),
    /** The material refracts light. */
    REFRACTIVE(3);

    final int blendMode;

    BlendMode(int blendMode) {
      this.blendMode = blendMode;
    }

    // Returns the int value of the blend mode.
    public int getValue() {
      return blendMode;
    }
  }

  /**
   * Double sided mode to use for the material. Determines whether both sides of the material should
   * be rendered.
   */
  public enum DoubleSidedMode {
    /** Only the front face of the material is rendered. */
    SINGLE_SIDED(0),
    /** Both the front and back faces of the material are rendered. */
    DOUBLE_SIDED(1);

    final int doubleSidedMode;

    DoubleSidedMode(int doubleSidedMode) {
      this.doubleSidedMode = doubleSidedMode;
    }

    // Returns the int value of the double sided mode.
    public int getValue() {
      return doubleSidedMode;
    }
  }

  // LINT.ThenChange(//depot/google3/third_party/split_engine/schemas/split_engine_material.fbs:khronos_pbr_material_spec_conversion_schema)

  private final LightingModel lightingModel;
  private final BlendMode blendMode;
  private final DoubleSidedMode doubleSidedMode;

  private KhronosPbrMaterialSpec(Builder builder) {
    this.lightingModel = builder.lightingModel;
    this.blendMode = builder.blendMode;
    this.doubleSidedMode = builder.doubleSidedMode;
  }

  public LightingModel getLightingModel() {
    return lightingModel;
  }

  public BlendMode getBlendMode() {
    return blendMode;
  }

  public DoubleSidedMode getDoubleSidedMode() {
    return doubleSidedMode;
  }

  /** Use Builder to construct a KhronosPbrMaterialSpec object instance. */
  public static class Builder {
    private LightingModel lightingModel = LightingModel.UNLIT;
    private BlendMode blendMode = BlendMode.OPAQUE;
    private DoubleSidedMode doubleSidedMode = DoubleSidedMode.SINGLE_SIDED;

    @CanIgnoreReturnValue
    public Builder setLightingModel(LightingModel lightingModel) {
      this.lightingModel = lightingModel;
      return this;
    }

    @CanIgnoreReturnValue
    public Builder setBlendMode(BlendMode blendMode) {
      this.blendMode = blendMode;
      return this;
    }

    @CanIgnoreReturnValue
    public Builder setDoubleSidedMode(DoubleSidedMode doubleSidedMode) {
      this.doubleSidedMode = doubleSidedMode;
      return this;
    }

    public KhronosPbrMaterialSpec build() {
      return new KhronosPbrMaterialSpec(this);
    }
  }
}
