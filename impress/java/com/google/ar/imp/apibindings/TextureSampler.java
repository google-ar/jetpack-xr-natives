/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
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
 * TextureSampler class used to define the way a texture gets sampled. The fields of this sampler
 * are based on the public Filament TextureSampler class, but do not need to be kept in sync.
 * https://github.com/google/filament/blob/main/android/filament-android/src/main/java/com/google/android/filament/TextureSampler.java
 *
 * @hide
 */
public final class TextureSampler {
  // LINT.IfChange(texture_sampler_conversion_bindings)

  /** Defines how texture coordinates outside the range [0, 1] are handled. */
  public enum WrapMode {
    /** The edge of the texture extends to infinity. */
    CLAMP_TO_EDGE(0),
    /** The texture infinitely repeats in the wrap direction. */
    REPEAT(1),
    /** The texture infinitely repeats and mirrors in the wrap direction. */
    MIRRORED_REPEAT(2);

    final int wrapMode;

    WrapMode(int wrapMode) {
      this.wrapMode = wrapMode;
    }

    // Returns the int value of the wrap mode.
    public int getValue() {
      return wrapMode;
    }
  }

  /**
   * Specifies how the texture is sampled when it's minified (appears smaller than its original
   * size).
   */
  public enum MinFilter {
    /** No filtering. Nearest neighbor is used. */
    NEAREST(0),
    /** Box filtering. Weighted average of 4 neighbors is used. */
    LINEAR(1),
    /** Mip-mapping is activated. But no filtering occurs. */
    NEAREST_MIPMAP_NEAREST(2),
    /** Box filtering within a mip-map level. */
    LINEAR_MIPMAP_NEAREST(3),
    /** Mip-map levels are interpolated, but no other filtering occurs. */
    NEAREST_MIPMAP_LINEAR(4),
    /** Both interpolated Mip-mapping and linear filtering are used. */
    LINEAR_MIPMAP_LINEAR(5);

    final int minFilter;

    MinFilter(int minFilter) {
      this.minFilter = minFilter;
    }

    // Returns the int value of the min filter.
    public int getValue() {
      return minFilter;
    }
  }

  /**
   * Specifies how the texture is sampled when it's magnified (appears larger than its original
   * size).
   */
  public enum MagFilter {
    /** No filtering. Nearest neighbor is used. */
    NEAREST(0),
    /** Box filtering. Weighted average of 4 neighbors is used. */
    LINEAR(1);

    final int magFilter;

    MagFilter(int magFilter) {
      this.magFilter = magFilter;
    }

    // Returns the int value of the mag filter.
    public int getValue() {
      return magFilter;
    }
  }

  /**
   * Used for depth texture comparisons, determining how the sampled depth value is compared to a
   * reference depth.
   */
  public enum CompareMode {
    NONE(0),
    COMPARE_TO_TEXTURE(1);

    final int compareMode;

    CompareMode(int compareMode) {
      this.compareMode = compareMode;
    }

    // Returns the int value of the compare mode.
    public int getValue() {
      return compareMode;
    }
  }

  /** Comparison functions for the depth sampler. */
  public enum CompareFunc {
    /** Less or equal */
    LE(0),
    /** Greater or equal */
    GE(1),
    /** Strictly less than */
    L(2),
    /** Strictly greater than */
    G(3),
    /** Equal */
    E(4),
    /** Not equal */
    NE(5),
    /** Always. Depth testing is deactivated. */
    A(6),
    /** Never. The depth test always fails. */
    N(7);

    final int compareFunc;

    CompareFunc(int compareFunc) {
      this.compareFunc = compareFunc;
    }

    // Returns the int value of the compare func.
    public int getValue() {
      return compareFunc;
    }
  }

  // LINT.ThenChange(//depot/google3/third_party/split_engine/schemas/split_engine_material.fbs:texture_sampler_conversion_schema)

  private final MinFilter minFilter;
  private final MagFilter magFilter;
  private final WrapMode wrapModeS;
  private final WrapMode wrapModeT;
  private final WrapMode wrapModeR;
  private final CompareMode compareMode;
  private final CompareFunc compareFunc;
  // Controls the level of anisotropic filtering applied to the texture, improving the appearance of
  // textures at steep angles. Higher values mean more samples and better quality, but also
  // increased GPU load
  private final int anisotropyLog2;

  private TextureSampler(Builder builder) {
    this.minFilter = builder.minFilter;
    this.magFilter = builder.magFilter;
    this.wrapModeS = builder.wrapModeS;
    this.wrapModeT = builder.wrapModeT;
    this.wrapModeR = builder.wrapModeR;
    this.compareMode = builder.compareMode;
    this.compareFunc = builder.compareFunc;
    this.anisotropyLog2 = builder.anisotropyLog2;
  }

  public MinFilter getMinFilter() {
    return minFilter;
  }

  public MagFilter getMagFilter() {
    return magFilter;
  }

  public WrapMode getWrapModeS() {
    return wrapModeS;
  }

  public WrapMode getWrapModeT() {
    return wrapModeT;
  }

  public WrapMode getWrapModeR() {
    return wrapModeR;
  }

  public CompareMode getCompareMode() {
    return compareMode;
  }

  public CompareFunc getCompareFunc() {
    return compareFunc;
  }

  public int getAnisotropyLog2() {
    return anisotropyLog2;
  }

  /** Use Builder to construct a TextureSampler object instance. */
  public static class Builder {
    private MinFilter minFilter = MinFilter.LINEAR;
    private MagFilter magFilter = MagFilter.LINEAR;
    private WrapMode wrapModeS = WrapMode.REPEAT;
    private WrapMode wrapModeT = WrapMode.REPEAT;
    private WrapMode wrapModeR = WrapMode.REPEAT;
    private CompareMode compareMode = CompareMode.NONE;
    private CompareFunc compareFunc = CompareFunc.LE;
    private int anisotropyLog2 = 0;

    @CanIgnoreReturnValue
    public Builder setMinFilter(MinFilter minFilter) {
      this.minFilter = minFilter;
      return this;
    }

    @CanIgnoreReturnValue
    public Builder setMagFilter(MagFilter magFilter) {
      this.magFilter = magFilter;
      return this;
    }

    @CanIgnoreReturnValue
    public Builder setWrapModeS(WrapMode wrapModeS) {
      this.wrapModeS = wrapModeS;
      return this;
    }

    @CanIgnoreReturnValue
    public Builder setWrapModeT(WrapMode wrapModeT) {
      this.wrapModeT = wrapModeT;
      return this;
    }

    @CanIgnoreReturnValue
    public Builder setWrapModeR(WrapMode wrapModeR) {
      this.wrapModeR = wrapModeR;
      return this;
    }

    @CanIgnoreReturnValue
    public Builder setCompareMode(CompareMode compareMode) {
      this.compareMode = compareMode;
      return this;
    }

    @CanIgnoreReturnValue
    public Builder setCompareFunc(CompareFunc compareFunc) {
      this.compareFunc = compareFunc;
      return this;
    }

    @CanIgnoreReturnValue
    public Builder setAnisotropyLog2(int anisotropyLog2) {
      this.anisotropyLog2 = anisotropyLog2;
      return this;
    }

    public TextureSampler build() {
      return new TextureSampler(this);
    }
  }
}
