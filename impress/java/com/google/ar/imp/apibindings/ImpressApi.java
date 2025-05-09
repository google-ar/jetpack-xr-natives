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

import android.view.Surface;
import androidx.annotation.IntDef;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import com.google.ar.imp.view.View;
import com.google.common.util.concurrent.ListenableFuture;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;

/** Interface for the JNI API for communicating with the Impress Split Engine instance. */
public interface ImpressApi {

  /**
   * Specifies how the Surface content will be routed for stereo viewing. Applications must render
   * into the surface in accordance with what is specified here in order for the compositor to
   * correctly produce a stereoscopic view to the user.
   *
   * <p>Values here match values from androidx.media3.common.C.StereoMode in
   * //third_party/java/android_libs/media:common
   */
  @Retention(RetentionPolicy.SOURCE)
  @IntDef({
    StereoMode.MONO,
    StereoMode.TOP_BOTTOM,
    StereoMode.SIDE_BY_SIDE,
    StereoMode.MULTIVIEW_LEFT_PRIMARY,
    StereoMode.MULTIVIEW_RIGHT_PRIMARY
  })
  public @interface StereoMode {
    // Each eye will see the entire surface (no separation)
    public static final int MONO = 0;
    // The [top, bottom] halves of the surface will map to [left, right] eyes
    public static final int TOP_BOTTOM = 1;
    // The [left, right] halves of the surface will map to [left, right] eyes
    public static final int SIDE_BY_SIDE = 2;
    // Multiview video, [primary, auxiliary] views will map to [left, right] eyes
    public static final int MULTIVIEW_LEFT_PRIMARY = 4;
    // Multiview video, [primary, auxiliary] views will map to [right, left] eyes
    public static final int MULTIVIEW_RIGHT_PRIMARY = 5;
  }

  /** This method initializes the Impress Split Engine instance. */
  void setup(@NonNull View view);

  /** Called when the activity or fragment is resumed. */
  void onResume();

  /** Called when the activity or fragment is paused. */
  void onPause();

  /** This method releases the asset pointer of a previously loaded image based lighting asset. */
  void releaseImageBasedLightingAsset(long iblToken);

  /**
   * This method loads an image based lighting asset from the assets folder and returns a future
   * with a token that can be used to reference the asset in other JNI calls.
   */
  @NonNull
  ListenableFuture<Long> loadImageBasedLightingAsset(@NonNull String path);

  /**
   * This method loads an image based lighting asset from a byte array and returns a future with a
   * token that can be used to reference the asset in other JNI calls.
   */
  @NonNull
  ListenableFuture<Long> loadImageBasedLightingAsset(@NonNull byte[] data, @NonNull String key);

  /**
   * This method loads a glTF model from the local assets folder or a remote URL, and returns a
   * future with the model token that can be used to reference the model in other JNI calls.
   */
  @NonNull
  ListenableFuture<Long> loadGltfAsset(@NonNull String path);

  /**
   * This method loads a glTF model from a byte array and returns a future with the model token that
   * can be used to reference the model in other JNI calls.
   */
  // TODO: Add an accessor which gets the model token from a name.
  @NonNull
  ListenableFuture<Long> loadGltfAsset(@NonNull byte[] data, @NonNull String key);

  // TODO - Add support for cancellation of loading operations (GLTF, EXR, etc.)

  /** This method releases the asset pointer of a previously loaded glTF model. */
  void releaseGltfAsset(long gltfToken);

  /**
   * This method instantiates a glTF model from a previously loaded model and returns an entity ID
   * corresponding to the Impress node associated with the model. Using this method will enable the
   * collider for the model.
   */
  int instanceGltfModel(long gltfToken);

  /**
   * This method instantiates a glTF model from a previously loaded model and returns an entity ID
   * corresponding to the Impress node associated with the model. It gives the ability to disable
   * the collider for the model.
   */
  int instanceGltfModel(long gltfToken, boolean enableCollider);

  /**
   * Toggle the collider of a glTF model.
   *
   * @param impressNode The integer ID of the Impress node for the instance of the glTF model.
   * @param enableCollider If the glTF model should have a collider or not.
   */
  void setGltfModelColliderEnabled(int impressNode, boolean enableCollider);

  /**
   * Starts an animation on an instanced GLTFModel.
   *
   * @param impressNode The integer ID of the Impress node for the instance of the GLTF
   * @param animationName A nullable String which contains a requested animation to play. If null is
   *     provided, this will attempt to play the first animation it finds
   * @param looping True if the animation should loop. Note that if the animation is looped, the
   *     returned Future will never fire successfully.
   * @return a ListenableFuture which fires when the animation stops. It will return an exception if
   *     the animation can't play.
   */
  // TODO: (broken link) - Remove CompletableFuture from SE integration.
  @NonNull
  ListenableFuture<Void> animateGltfModel(
      int impressNode, @Nullable String animationName, boolean looping);

  /**
   * Stops an animation on an instanced GLTFModel.
   *
   * @param impressNode The integer ID of the Impress node for the instance of the GLTF
   */
  void stopGltfModelAnimation(int impressNode);

  /** This method creates an Impress node and returns its entity ID. */
  int createImpressNode();

  /** This method destroys an Impress node using its entity ID. */
  void destroyImpressNode(int impressNode);

  /** This method parents an Impress node to another using their respective entity IDs. */
  void setImpressNodeParent(int impressNodeChild, int impressNodeParent);

  /**
   * This method creates an Impress node with a stereo panel and returns the entity ID. Note that
   * the StereoSurfaceEntity will not be render anything until the canvas shape is set.
   *
   * @param stereoMode The [Int] stereoMode to apply. Must be a member of StereoMode.
   * @throws InvalidAgumentException if stereoMode is invalid.
   * @return An int impress node ID which can be used for updating the surface later
   */
  int createStereoSurface(@StereoMode int stereoMode);

  /**
   * This method sets the canvas shape of a StereoSurfaceEntity using its Impress ID.
   *
   * @param impressNode The Impress node which hosts the StereoSurfaceEntity to be updated.
   * @param width The width in local spatial units to set the quad to.
   * @param height The height in local spatial units to set the quad to.
   */
  void setStereoSurfaceEntityCanvasShapeQuad(int impressNode, float width, float height);

  /**
   * This method sets the canvas shape of a StereoSurfaceEntity using its Impress ID.
   *
   * @param impressNode The Impress node which hosts the StereoSurfaceEntity to be updated.
   * @param radius The radius in local spatial units to set the sphere to.
   */
  void setStereoSurfaceEntityCanvasShapeSphere(int impressNode, float radius);

  /**
   * This method sets the canvas shape of a StereoSurfaceEntity using its Impress ID.
   *
   * @param impressNode The Impress node which hosts the StereoSurfaceEntity to be updated.
   * @param radius The radius in local spatial units of the hemisphere.
   */
  void setStereoSurfaceEntityCanvasShapeHemisphere(int impressNode, float radius);

  /**
   * Updates the StereoMode for an impress node hosting a StereoSurface.
   *
   * @param panelImpressNode The Impress node which hosts the panel to be updated.
   * @param stereoMode The [Int] stereoMode to apply. Must be a member of StereoMode
   * @throws InvalidAgumentException if stereoMode is invalid.
   */
  void setStereoModeForStereoSurface(int panelImpressNode, @StereoMode int stereoMode);

  /**
   * Updates the radius of the (alpha) feathered edges for an Impress node hosting a StereoSurface.
   *
   * @param panelImpressNode The Impress node which hosts the panel to be updated.
   * @param radiusX The radius of the left/right feathering.
   * @param radiusY The radius of the top/bottom feathering.
   */
  void setFeatherRadiusForStereoSurface(int panelImpressNode, float radiusX, float radiusY);

  /**
   * Sets the primary alpha mask for a stereo surface. The alpha mask will be composited into the
   * alpha channel of the surface. If null or empty, the alpha mask will be disabled.
   *
   * @param panelImpressNode The Impress node which hosts the panel to be updated.
   * @param alphaMask The primary alpha mask texture
   */
  void setPrimaryAlphaMaskForStereoSurface(int panelImpressNode, long alphaMask);

  /**
   * Sets the auxiliary alpha mask for a stereo surface. The alpha mask will be composited into the
   * alpha channel of the surface if an interleaved video is being rendered.
   *
   * @param panelImpressNode The Impress node which hosts the panel to be updated.
   * @param alphaMask The auxiliary alpha mask texture. Only used with interleaved video formats. If
   *     null or empty, the alpha mask will be disabled.
   */
  void setAuxiliaryAlphaMaskForStereoSurface(int panelImpressNode, long alphaMask);

  /**
   * Retrieve the android surface for this stereo panel
   *
   * @param panelImpressNode The Impress node which hosts the panel to be updated.
   * @return A Surface backed by an imp::AndroidExternalTextureSurface
   */
  Surface getSurfaceFromStereoSurface(int panelImpressNode);

  /**
   * This method loads a local texture from the assets folder or a remote texture from a URLand
   * returns a future with the texture token that can be used to reference the texture in other JNI
   * calls.
   *
   * @param path The name of the texture file to load or the URL of the remote texture.
   * @param sampler The sampler to use when loading the texture.
   * @return A future that resolves to the texture when it is loaded.
   */
  @NonNull
  ListenableFuture<Texture> loadTexture(@NonNull String path, @NonNull TextureSampler sampler);

  /**
   * This method borrows the reflection texture from the currently set environment IBL.
   *
   * @return A texture that can be used to reference the texture in other JNI calls.
   */
  @NonNull
  Texture borrowReflectionTexture();

  /**
   * This method borrows the reflection texture from the given IBL.
   *
   * @return A texture that can be used to reference the texture in other JNI calls.
   */
  @NonNull
  Texture getReflectionTextureFromIbl(long iblToken);

  /**
   * This method creates a water material and returns a future with the material native handle that
   * can be used to reference the water material in other JNI calls.
   *
   * @param isAlphaMapVersion True if the water material should be the alpha map version.
   * @return A WaterMaterial backed by an imp::WaterMaterial. The WaterMaterial can be destroyed by
   *     passing it to destroyNativeObject.
   */
  ListenableFuture<WaterMaterial> createWaterMaterial(boolean isAlphaMapVersion);

  /**
   * This method sets the reflection cube for the water material.
   *
   * @param nativeWaterMaterial The native handle of the water material to be updated.
   * @param reflectionCube The native handle of the texture to be used as the reflection cube.
   */
  void setReflectionCubeOnWaterMaterial(long nativeWaterMaterial, long reflectionCube);

  /**
   * This method sets the normal map for the water material.
   *
   * @param nativeWaterMaterial The native handle of the water material to be updated.
   * @param normalMap The native handle of the texture to be used as the normal map.
   */
  void setNormalMapOnWaterMaterial(long nativeWaterMaterial, long normalMap);

  /**
   * This method sets the normal tiling for the water material.
   *
   * @param nativeWaterMaterial The native handle of the water material to be updated.
   * @param normalTiling The tiling to use for the normal map.
   */
  void setNormalTilingOnWaterMaterial(long nativeWaterMaterial, float normalTiling);

  /**
   * This method sets the normal speed for the water material.
   *
   * @param nativeWaterMaterial The native handle of the water material to be updated.
   * @param normalSpeed The speed to use for the normal map.
   */
  void setNormalSpeedOnWaterMaterial(long nativeWaterMaterial, float normalSpeed);

  /**
   * This method sets the alpha step multiplier for the water material.
   *
   * @param nativeWaterMaterial The native handle of the water material to be updated.
   * @param alphaStepMultiplier The alpha step multiplier to use for the water material.
   */
  void setAlphaStepMultiplierOnWaterMaterial(long nativeWaterMaterial, float alphaStepMultiplier);

  /**
   * This method sets the alpha map for the water material.
   *
   * @param nativeWaterMaterial The native handle of the water material to be updated.
   * @param alphaMap The native handle of the texture to be used as the alpha map.
   */
  void setAlphaMapOnWaterMaterial(long nativeWaterMaterial, long alphaMap);

  /**
   * This method sets the normal z for the water material.
   *
   * @param nativeWaterMaterial The native handle of the water material to be updated.
   * @param normalZ The normal z to use for the water material.
   */
  void setNormalZOnWaterMaterial(long nativeWaterMaterial, float normalZ);

  /**
   * This method sets the normal boundary for the water material.
   *
   * @param nativeWaterMaterial The native handle of the water material to be updated.
   * @param boundary The normal boundary to use for the water material.
   */
  void setNormalBoundaryOnWaterMaterial(long nativeWaterMaterial, float boundary);

  /**
   * This method sets the alpha step U for the water material.
   *
   * @param nativeWaterMaterial The native handle of the water material to be updated.
   * @param x The x coordinate of the alpha step U.
   * @param y The y coordinate of the alpha step U.
   * @param z The z coordinate of the alpha step U.
   * @param w The w coordinate of the alpha step U.
   */
  void setAlphaStepUOnWaterMaterial(long nativeWaterMaterial, float x, float y, float z, float w);

  /**
   * This method sets the alpha step V for the water material.
   *
   * @param nativeWaterMaterial The native handle of the water material to be updated.
   * @param x The x coordinate of the alpha step V.
   * @param y The y coordinate of the alpha step V.
   * @param z The z coordinate of the alpha step V.
   * @param w The w coordinate of the alpha step V.
   */
  void setAlphaStepVOnWaterMaterial(long nativeWaterMaterial, float x, float y, float z, float w);

  /**
   * This method destroys a native Impress object using its native handle.
   *
   * @param nativeHandle The native handle of the native Impress object to be destroyed.
   */
  void destroyNativeObject(long nativeHandle);

  /**
   * This method sets the material override for the mesh of a glTF model.
   *
   * @param impressNode The integer ID of the Impress node for the instance of the glTF model.
   * @param nativeMaterial The native handle of the material to be used as the override.
   * @param meshName The name of the mesh to be overridden.
   */
  void setMaterialOverride(int impressNode, long nativeMaterial, @NonNull String meshName);

  /**
   * This method sets the IBL asset preference of the client to be set by the system.
   *
   * @param iblToken The native handle of the IBL asset to be used by the system.
   * @throws NotFoundException if iblToken is not a previously loaded IBL asset.
   * @throws IllegalStateException if the SplitEngineSerializer is not valid.
   */
  void setPreferredEnvironmentLight(long iblToken);

  /**
   * This method clears the IBL asset preference of the client to be set by the system.
   *
   * @throws IllegalStateException if the SplitEngineSerializer is not valid.
   */
  void clearPreferredEnvironmentIblAsset();

  /**
   * This method disposes all of the resources associated with the Impress Split Engine instance.
   *
   * <p>This should be called when the Impress Split Engine instance is no longer needed.
   */
  void disposeAllResources();
}
