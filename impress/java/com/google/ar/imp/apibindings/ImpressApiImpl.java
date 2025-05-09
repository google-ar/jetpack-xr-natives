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

import android.content.res.Resources.NotFoundException;
import android.util.Log;
import android.view.Surface;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.concurrent.futures.CallbackToFutureAdapter;
import com.google.ar.imp.view.View;
import com.google.common.util.concurrent.ListenableFuture;

// TODO: (broken link) - Add unit tests for this class.

/** Implementation of the JNI API for communicating with the Impress Split Engine instance. */
public final class ImpressApiImpl implements ImpressApi {
  private static final String TAG = ImpressApiImpl.class.getSimpleName();

  private View view;

  /*
   * This is mostly here to throw on unsupported values. The int cast works as long as
   * ImpressApi.StereoMode is in sync with imp::MediaStereoMode.
   */
  private int validateStereoMode(@StereoMode int stereoMode) {
    switch (stereoMode) {
      case StereoMode.MONO:
      case StereoMode.TOP_BOTTOM:
      case StereoMode.SIDE_BY_SIDE:
      case StereoMode.MULTIVIEW_LEFT_PRIMARY:
      case StereoMode.MULTIVIEW_RIGHT_PRIMARY:
        return stereoMode;
      default:
        throw new IllegalArgumentException(
            "Unspported value for ImpressApi.StereoMode: " + stereoMode);
    }
  }

  @Override
  public void setup(View view) {
    this.view = view;
    nSetup(view.getNativeHandle());
  }

  @Override
  public void onResume() {
    view.onResume();
  }

  @Override
  public void onPause() {
    view.onPause();
  }

  @Override
  public void releaseImageBasedLightingAsset(long iblToken) {
    Status<?> status = nReleaseImageBasedLightingAsset(view.getNativeHandle(), iblToken);
    if (status instanceof Status.Error.NotFound notFound) {
      throw new IllegalStateException(
          "Could not release image based lighting asset with token: "
              + iblToken
              + " with error: "
              + notFound.getDetails());
    }
  }

  @Override
  @NonNull
  public ListenableFuture<Long> loadImageBasedLightingAsset(@NonNull String path) {
    return CallbackToFutureAdapter.getFuture(
        completer -> {
          // TODO: (broken link) - Add a cancellationListener to the completer here when the loading
          //                     APIs support cancellation.
          nLoadImageBasedLightingAssetFromPath(
              view.getNativeHandle(),
              // The underlying C++ code will hold a reference to this (anoynomous) IAssetLoader
              // until the load is complete.
              new IAssetLoader() {

                @Override
                public void onSuccess(long value) {
                  completer.set(value);
                }

                @Override
                public void onFailure(@NonNull String message) {
                  // TODO: (broken link) - Publish a more precisely typed Exception interface for
                  // this.
                  // Alternatively we could return null here and have some means of also surfacing
                  // an error message to the application.
                  completer.setException(new Exception(message));
                }
              },
              path);
          return "LoadImageBasedLightingAsset Operation";
        });
  }

  @Override
  @NonNull
  public ListenableFuture<Long> loadImageBasedLightingAsset(
      @NonNull byte[] data, @NonNull String key) {
    return CallbackToFutureAdapter.getFuture(
        completer -> {
          // TODO: (broken link) - Add a cancellationListener to the completer here when the loading
          //                     APIs support cancellation.
          nLoadImageBasedLightingAssetFromByteArray(
              view.getNativeHandle(),
              // The underlying C++ code will hold a reference to this (anoynomous) IAssetLoader
              // until the load is complete.
              new IAssetLoader() {

                @Override
                public void onSuccess(long value) {
                  completer.set(value);
                }

                @Override
                public void onFailure(@NonNull String message) {
                  // TODO: (broken link) - Publish a more precisely typed Exception interface for
                  // this.
                  // Alternatively we could return null here and have some means of also surfacing
                  // an error message to the application.
                  completer.setException(new Exception(message));
                }
              },
              data,
              key);
          return "LoadImageBasedLightingAsset Operation";
        });
  }

  @Override
  @NonNull
  public ListenableFuture<Long> loadGltfAsset(@NonNull String path) {
    return CallbackToFutureAdapter.getFuture(
        completer -> {
          // TODO: (broken link) - Add a cancellationListener to the completer here when the loading
          //                     APIs support cancellation.
          nLoadGltfAssetFromPath(
              view.getNativeHandle(),
              // The underlying C++ code will hold a reference to this (anoynomous) IAssetLoader
              // until the load is complete.
              // TODO: Revisit the way C++ --> Java code is called back for the
              // IAssetLoader (proguard)
              new IAssetLoader() {

                @Override
                public void onSuccess(long value) {
                  completer.set(value);
                }

                @Override
                public void onFailure(@NonNull String message) {
                  // TODO: (broken link) - Publish a more precisely typed Exception interface for
                  // this.
                  // Alternatively we could return null here and have some means of also surfacing
                  // an error message to the application.
                  completer.setException(new Exception(message));
                }
              },
              path);
          return "LoadGltfAsset Operation";
        });
  }

  @Override
  @NonNull
  public ListenableFuture<Long> loadGltfAsset(@NonNull byte[] data, @NonNull String key) {
    return CallbackToFutureAdapter.getFuture(
        completer -> {
          // TODO: (broken link) - Add a cancellationListener to the completer here when the loading
          //                     APIs support cancellation.
          nLoadGltfAssetFromByteArray(
              view.getNativeHandle(),
              // The underlying C++ code will hold a reference to this (anoynomous) IAssetLoader
              // until the load is complete.
              new IAssetLoader() {

                @Override
                public void onSuccess(long value) {
                  completer.set(value);
                }

                @Override
                public void onFailure(@NonNull String message) {
                  // TODO: (broken link) - Publish a more precisely typed Exception interface for
                  // this.
                  // Alternatively we could return null here and have some means of also surfacing
                  // an error message to the application.
                  completer.setException(new Exception(message));
                }
              },
              data,
              key);
          return "LoadGltfAsset Operation";
        });
  }

  @Override
  public void releaseGltfAsset(long gltfToken) {
    Status<?> status = nReleaseGltfAsset(view.getNativeHandle(), gltfToken);
    if (status instanceof Status.Error.NotFound notFound) {
      throw new IllegalStateException(
          "Could not release asset with token: "
              + gltfToken
              + " with error: "
              + notFound.getDetails());
    }
  }

  @Override
  public int instanceGltfModel(long gltfToken) {
    Log.i(TAG, "Instantiating model with token: " + gltfToken);
    // TODO: (broken link) - Re-establish Collider when an Interactable/Movable/etc Component is
    //                     attached
    Status<?> status =
        nInstanceGltfModel(view.getNativeHandle(), gltfToken, /* enableCollider= */ false);
    if (status instanceof Status.Error.InvalidArgument invalidArgument) {
      throw new IllegalStateException(
          "Could not instance model with token: "
              + gltfToken
              + " with error: "
              + invalidArgument.getDetails());
    }
    return ((Status.SuccessWithIntValue) status).getData();
  }

  @Override
  public int instanceGltfModel(long gltfToken, boolean enableCollider) {
    Log.i(
        TAG,
        "Instantiating model with token: "
            + gltfToken
            + " with enablecollider parameter set to: "
            + enableCollider);
    Status<?> status = nInstanceGltfModel(view.getNativeHandle(), gltfToken, enableCollider);
    if (status instanceof Status.Error.InvalidArgument invalidArgument) {
      throw new IllegalStateException(
          "Instantiating model with token: "
              + gltfToken
              + " with enablecollider parameter set to: "
              + enableCollider
              + " failed with error: "
              + invalidArgument.getDetails());
    }
    return ((Status.SuccessWithIntValue) status).getData();
  }

  // TODO: Add support for toggling the collider on StereoSurface.
  @Override
  public void setGltfModelColliderEnabled(int impressNode, boolean enableCollider) {
    Status<?> status =
        nSetGltfModelColliderEnabled(view.getNativeHandle(), impressNode, enableCollider);
    if (status instanceof Status.Error.InvalidArgument invalidArgument) {
      throw new IllegalStateException(
          "Could not set collider enabled for model with impress node: "
              + impressNode
              + " with error: "
              + invalidArgument.getDetails());
    }
  }

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
  @Override
  @NonNull
  public ListenableFuture<Void> animateGltfModel(
      int impressNode, @Nullable String animationName, boolean looping) {

    return CallbackToFutureAdapter.getFuture(
        completer -> {
          nAnimateGltfModel(
              view.getNativeHandle(),
              impressNode,
              animationName,
              looping,
              new IAssetAnimator() {
                // Hold a reference to the completer to ensure it isn't garbage collected until the
                // C++ side releases the reference to the IAssetAnimator. The future returned by
                // CallbackToFutureAdapter.getFuture() aggressively tries to let the garbage
                // collector clean up the completer as an optimization - we're concerned that this
                // could cause the future to never fire, or cancel incorrectly and return an error,
                // especially since the code that calls this simply allows the future to go out of
                // scope without storing it.  This might not actually be a problem, but this code
                // shouldn't be harmful and should reduce the uncertainty.
                // We should eventually have a different way of
                // communicating animation completion back to the application. See (broken link).
                CallbackToFutureAdapter.Completer<Void> mCompleter = completer;

                @Override
                public void onComplete() {
                  // Setting null here is required since we don't have a return value.
                  mCompleter.set(null);
                }

                @Override
                public void onFailure(@NonNull String message) {
                  mCompleter.setException(new IllegalStateException(message));
                }
              });
          return "AnimateGltfModel Operation";
        });
  }

  /**
   * Stops an animation on an instanced GLTFModel.
   *
   * @param impressNode The integer ID of the Impress node for the instance of the GLTF
   */
  @Override
  public void stopGltfModelAnimation(int impressNode) {

    Status<?> status = nStopGltfModelAnimation(view.getNativeHandle(), impressNode);
    if (status instanceof Status.Error.InvalidArgument invalidArgument) {
      throw new IllegalStateException(
          "Could not stop animation of model with id: "
              + impressNode
              + " with error: "
              + invalidArgument.getDetails());
    }
  }

  @Override
  public int createImpressNode() {
    return nCreateImpressNode(view.getNativeHandle());
  }

  @Override
  public void destroyImpressNode(int impressNode) {
    Status<?> status = nDestroyImpressNode(view.getNativeHandle(), impressNode);
    if (status instanceof Status.Error.InvalidArgument invalidArgument) {
      throw new IllegalStateException(
          "Could not destroy impress node with id: "
              + impressNode
              + " with error: "
              + invalidArgument.getDetails());
    }
  }

  @Override
  public void setImpressNodeParent(int impressNodeChild, int impressNodeParent) {
    Status<?> status =
        nSetImpressNodeParent(view.getNativeHandle(), impressNodeChild, impressNodeParent);
    if (status instanceof Status.Error.InvalidArgument invalidArgument) {
      throw new IllegalStateException(
          "Could not set impress node with id: "
              + impressNodeParent
              + " as parent of impress node with id: "
              + impressNodeChild
              + " with error: "
              + invalidArgument.getDetails());
    }
  }

  @Override
  public int createStereoSurface(@StereoMode int stereoMode) {
    return nCreateStereoSurfaceEntity(view.getNativeHandle(), validateStereoMode(stereoMode));
  }

  @Override
  public void setStereoSurfaceEntityCanvasShapeQuad(int impressNode, float width, float height) {
    nSetStereoSurfaceEntityCanvasShapeQuad(view.getNativeHandle(), impressNode, width, height);
  }

  @Override
  public void setStereoSurfaceEntityCanvasShapeSphere(int impressNode, float radius) {
    nSetStereoSurfaceEntityCanvasShapeSphere(view.getNativeHandle(), impressNode, radius);
  }

  @Override
  public void setStereoSurfaceEntityCanvasShapeHemisphere(int impressNode, float radius) {
    nSetStereoSurfaceEntityCanvasShapeHemisphere(view.getNativeHandle(), impressNode, radius);
  }

  @Override
  public void setStereoModeForStereoSurface(int panelImpressNode, @StereoMode int stereoMode) {
    nSetStereoModeForStereoSurfaceEntity(
        view.getNativeHandle(), panelImpressNode, validateStereoMode(stereoMode));
  }

  @Override
  public void setFeatherRadiusForStereoSurface(int panelImpressNode, float radiusX, float radiusY) {
    nSetFeatherRadiusForStereoSurfaceEntity(
        view.getNativeHandle(), panelImpressNode, radiusX, radiusY);
  }

  @Override
  public Surface getSurfaceFromStereoSurface(int panelImpressNode) {
    return nGetSurfaceFromStereoSurfaceEntity(view.getNativeHandle(), panelImpressNode);
  }

  @Override
  public void setPrimaryAlphaMaskForStereoSurface(int panelImpressNode, long alphaMask) {
    nSetPrimaryAlphaMaskForStereoSurfaceEntity(view.getNativeHandle(), panelImpressNode, alphaMask);
  }

  @Override
  public void setAuxiliaryAlphaMaskForStereoSurface(int panelImpressNode, long alphaMask) {
    nSetAuxiliaryAlphaMaskForStereoSurfaceEntity(
        view.getNativeHandle(), panelImpressNode, alphaMask);
  }

  @Override
  @NonNull
  public ListenableFuture<Texture> loadTexture(
      @NonNull String path, @NonNull TextureSampler sampler) {
    return CallbackToFutureAdapter.getFuture(
        completer -> {
          // TODO: (broken link) - Add a cancellationListener to the completer here when the loading
          //                     APIs support cancellation.
          nLoadTexture(
              view.getNativeHandle(),
              // The underlying C++ code will hold a reference to this (anoynomous) IAssetLoader
              // until the load is complete.
              // TODO: Revisit the way C++ --> Java code is called back for the
              // IAssetLoader (proguard)
              new IAssetLoader() {

                @Override
                public void onSuccess(long value) {
                  Texture texture =
                      new Texture.Builder()
                          .setImpressApi(ImpressApiImpl.this)
                          .setNativeTexture(value)
                          .setTextureSampler(sampler)
                          .build();
                  completer.set(texture);
                }

                @Override
                public void onFailure(@NonNull String message) {
                  // TODO: (broken link) - Publish a more precisely typed Exception interface for
                  // this.
                  // Alternatively we could return null here and have some means of also surfacing
                  // an error message to the application.
                  completer.setException(new Exception(message));
                }
              },
              path,
              sampler.getMinFilter().getValue(),
              sampler.getMagFilter().getValue(),
              sampler.getWrapModeS().getValue(),
              sampler.getWrapModeT().getValue(),
              sampler.getWrapModeR().getValue(),
              sampler.getCompareMode().getValue(),
              sampler.getCompareFunc().getValue(),
              sampler.getAnisotropyLog2());
          return "LoadTexture Operation";
        });
  }

  @Override
  @NonNull
  public Texture borrowReflectionTexture() {
    Status<?> status = nBorrowReflectionTexture(view.getNativeHandle());
    if (status instanceof Status.Error.NotFound notFound) {
      throw new IllegalStateException("No reflection texture with error: " + notFound.getDetails());
    }
    return new Texture.Builder()
        .setImpressApi(ImpressApiImpl.this)
        .setNativeTexture(((Status.SuccessWithLongValue) status).getData())
        .build();
  }

  @Override
  @NonNull
  public Texture getReflectionTextureFromIbl(long iblToken) {
    Status<?> status = nGetReflectionTextureFromIbl(view.getNativeHandle(), iblToken);
    if (status instanceof Status.Error.NotFound notFound) {
      throw new NotFoundException("No reflection texture with error: " + notFound.getDetails());
    }
    return new Texture.Builder()
        .setImpressApi(ImpressApiImpl.this)
        .setNativeTexture(((Status.SuccessWithLongValue) status).getData())
        .build();
  }

  @Override
  public ListenableFuture<WaterMaterial> createWaterMaterial(boolean isAlphaMapVersion) {
    return CallbackToFutureAdapter.getFuture(
        completer -> {
          // TODO: (broken link) - Add a cancellationListener to the completer here when the loading
          //                     APIs support cancellation.
          nCreateWaterMaterial(
              view.getNativeHandle(),
              // The underlying C++ code will hold a reference to this (anoynomous) IAssetLoader
              // until the load is complete.
              // TODO: Revisit the way C++ --> Java code is called back for the
              // IAssetLoader (proguard)
              new IAssetLoader() {

                @Override
                public void onSuccess(long value) {
                  WaterMaterial waterMaterial =
                      new WaterMaterial.Builder()
                          .setImpressApi(ImpressApiImpl.this)
                          .setNativeMaterial(value)
                          .build();
                  completer.set(waterMaterial);
                }

                @Override
                public void onFailure(@NonNull String message) {
                  // TODO: (broken link) - Publish a more precisely typed Exception interface for
                  // this.
                  // Alternatively we could return null here and have some means of also surfacing
                  // an error message to the application.
                  completer.setException(new Exception(message));
                }
              },
              isAlphaMapVersion);
          return "CreateWaterMaterial Operation";
        });
  }

  @Override
  public void setReflectionCubeOnWaterMaterial(long nativeWaterMaterial, long reflectionCube) {
    Status<?> status =
        nSetReflectionCubeOnWaterMaterial(
            view.getNativeHandle(), nativeWaterMaterial, reflectionCube);
    if (status instanceof Status.Error.InvalidArgument invalidArgument) {
      throw new IllegalStateException(
          "Could not set reflection cube on water material with error: "
              + invalidArgument.getDetails());
    }
  }

  @Override
  public void setNormalMapOnWaterMaterial(long nativeWaterMaterial, long normalMap) {
    Status<?> status =
        nSetNormalMapOnWaterMaterial(view.getNativeHandle(), nativeWaterMaterial, normalMap);
    if (status instanceof Status.Error.InvalidArgument invalidArgument) {
      throw new IllegalStateException(
          "Could not set normal map on water material with error: " + invalidArgument.getDetails());
    }
  }

  @Override
  public void setNormalTilingOnWaterMaterial(long nativeWaterMaterial, float normalTiling) {
    Status<?> status =
        nSetNormalTilingOnWaterMaterial(view.getNativeHandle(), nativeWaterMaterial, normalTiling);
    if (status instanceof Status.Error.InvalidArgument invalidArgument) {
      throw new IllegalStateException(
          "Could not set normal tiling on water material with error: "
              + invalidArgument.getDetails());
    }
  }

  @Override
  public void setNormalSpeedOnWaterMaterial(long nativeWaterMaterial, float normalSpeed) {
    Status<?> status =
        nSetNormalSpeedOnWaterMaterial(view.getNativeHandle(), nativeWaterMaterial, normalSpeed);
    if (status instanceof Status.Error.InvalidArgument invalidArgument) {
      throw new IllegalStateException(
          "Could not set normal speed on water material with error: "
              + invalidArgument.getDetails());
    }
  }

  @Override
  public void setAlphaStepMultiplierOnWaterMaterial(
      long nativeWaterMaterial, float alphaStepMultiplier) {
    Status<?> status =
        nSetAlphaStepMultiplierOnWaterMaterial(
            view.getNativeHandle(), nativeWaterMaterial, alphaStepMultiplier);
    if (status instanceof Status.Error.InvalidArgument invalidArgument) {
      throw new IllegalStateException(
          "Could not set alpha step multiplier on water material with error: "
              + invalidArgument.getDetails());
    }
  }

  @Override
  public void setAlphaMapOnWaterMaterial(long nativeWaterMaterial, long alphaMap) {
    Status<?> status =
        nSetAlphaMapOnWaterMaterial(view.getNativeHandle(), nativeWaterMaterial, alphaMap);
    if (status instanceof Status.Error.InvalidArgument invalidArgument) {
      throw new IllegalStateException(
          "Could not set alpha map on water material with error: " + invalidArgument.getDetails());
    }
  }

  @Override
  public void setNormalZOnWaterMaterial(long nativeWaterMaterial, float normalZ) {
    Status<?> status =
        nSetNormalZOnWaterMaterial(view.getNativeHandle(), nativeWaterMaterial, normalZ);
    if (status instanceof Status.Error.InvalidArgument invalidArgument) {
      throw new IllegalStateException(
          "Could not set normal z on water material with error: " + invalidArgument.getDetails());
    }
  }

  @Override
  public void setNormalBoundaryOnWaterMaterial(long nativeWaterMaterial, float normalBoundary) {
    Status<?> status =
        nSetNormalBoundaryOnWaterMaterial(
            view.getNativeHandle(), nativeWaterMaterial, normalBoundary);
    if (status instanceof Status.Error.InvalidArgument invalidArgument) {
      throw new IllegalStateException(
          "Could not set normal boundary on water material with error: "
              + invalidArgument.getDetails());
    }
  }

  @Override
  public void setAlphaStepUOnWaterMaterial(
      long nativeWaterMaterial, float x, float y, float z, float w) {
    throw new UnsupportedOperationException("Stub API to be removed.");
  }

  @Override
  public void setAlphaStepVOnWaterMaterial(
      long nativeWaterMaterial, float x, float y, float z, float w) {
    throw new UnsupportedOperationException("Stub API to be removed.");
  }

  @Override
  public void destroyNativeObject(long nativeHandle) {
    nDestroyNativeObject(view.getNativeHandle(), nativeHandle);
  }

  @Override
  public void setMaterialOverride(int impressNode, long nativeMaterial, @NonNull String meshName) {
    Status<?> status =
        nSetMaterialOverride(view.getNativeHandle(), impressNode, nativeMaterial, meshName);
    if (status instanceof Status.Error.Internal internal) {
      throw new InternalError(
          "Could not set material override with error: " + internal.getDetails());
    } else if (status instanceof Status.Error.InvalidArgument invalidArgument) {
      throw new IllegalStateException(
          "Provided material is not valid with error: " + invalidArgument.getDetails());
    } else if (status instanceof Status.Error.NotFound notFound) {
      throw new NotFoundException(
          "Did not find with the provided name with error: " + notFound.getDetails());
    }
  }

  @Override
  public void setPreferredEnvironmentLight(long iblToken) {
    Log.i(TAG, "Setting IBL asset preference with token: " + iblToken);
    Status<?> status = nSetEnvironmentLight(view.getNativeHandle(), iblToken);
    String baseMessage =
        "Could not set IBL asset preference with token: " + iblToken + " with error: ";
    if (status instanceof Status.Error.NotFound notFound) {
      throw new NotFoundException(baseMessage + notFound.getDetails());
    } else if (status instanceof Status.Error.Internal internal) {
      throw new InternalError(baseMessage + internal.getDetails());
    }
  }

  @Override
  public void clearPreferredEnvironmentIblAsset() {
    Log.i(TAG, "Clearing skybox preference");
    Status<?> status = nClearEnvironmentLight(view.getNativeHandle());
    if (status instanceof Status.Error.Internal internal) {
      throw new InternalError(
          "Could not clear skybox preference with error: " + internal.getDetails());
    }
  }

  @Override
  public void disposeAllResources() {
    Log.i(TAG, "Disposing of the resources associated with the Impress Split Engine instance.");
    Status<?> status = nDisposeAllResources(view.getNativeHandle());
    if (status instanceof Status.Error.Internal internal) {
      throw new InternalError(
          "Could not dispose resources associated with the Impress Split Engine instance with"
              + " error: "
              + internal.getDetails());
    }
  }

  // LINT.IfChange(api)
  // returns the bridge handle after it's been initialized.
  private static native void nSetup(long view);

  private static native Status<?> nReleaseImageBasedLightingAsset(long view, long assetToken);

  private static native void nLoadImageBasedLightingAssetFromPath(
      long view, IAssetLoader assetLoader, String path);

  private static native void nLoadImageBasedLightingAssetFromByteArray(
      long view, IAssetLoader assetLoader, byte[] data, String key);

  private static native void nLoadGltfAssetFromPath(
      long view, IAssetLoader assetLoader, String path);

  private static native void nLoadGltfAssetFromByteArray(
      long view, IAssetLoader assetLoader, byte[] data, String key);

  private static native Status<?> nReleaseGltfAsset(long view, long gltfToken);

  private static native Status<?> nInstanceGltfModel(
      long view, long gltfToken, boolean enableCollider);

  private static native Status<?> nSetGltfModelColliderEnabled(
      long view, long gltfToken, boolean enableCollider);

  private static native void nAnimateGltfModel(
      long view, int impressNode, String animationName, boolean loop, IAssetAnimator assetAnimator);

  private static native Status<?> nStopGltfModelAnimation(long view, int impressNode);

  private static native int nCreateImpressNode(long view);

  private static native Status<?> nDestroyImpressNode(long view, int impressNode);

  private static native Status<?> nSetImpressNodeParent(
      long view, int impressNodeChild, int impressNodeParent);

  private static native int nCreateStereoSurfaceEntity(long view, int stereoMode);

  private static native void nSetStereoSurfaceEntityCanvasShapeQuad(
      long view, int impressNode, float width, float height);

  private static native void nSetStereoSurfaceEntityCanvasShapeSphere(
      long view, int impressNode, float radius);

  private static native void nSetStereoSurfaceEntityCanvasShapeHemisphere(
      long view, int impressNode, float radius);

  private static native Surface nGetSurfaceFromStereoSurfaceEntity(long view, int panelImpressNode);

  private static native void nSetFeatherRadiusForStereoSurfaceEntity(
      long view, int panelImpressNode, float radiusX, float radiusY);

  private static native void nSetStereoModeForStereoSurfaceEntity(
      long view, int panelImpressNode, int stereoMode);

  private static native void nSetPrimaryAlphaMaskForStereoSurfaceEntity(
      long view, int panelImpressNode, long alphaMask);

  private static native void nSetAuxiliaryAlphaMaskForStereoSurfaceEntity(
      long view, int panelImpressNode, long alphaMask);

  private static native void nLoadTexture(
      long view,
      IAssetLoader assetLoader,
      String path,
      int minFilter,
      int magFilter,
      int wrapModeS,
      int wrapModeT,
      int wrapModeR,
      int compareMode,
      int compareFunc,
      int anisotropyLog2);

  private static native Status<?> nBorrowReflectionTexture(long view);

  private static native Status<?> nGetReflectionTextureFromIbl(long view, long iblToken);

  private static native void nCreateWaterMaterial(
      long view, IAssetLoader assetLoader, boolean isAlphaMapVersion);

  private static native Status<?> nSetReflectionCubeOnWaterMaterial(
      long view, long nativeWaterMaterial, long reflectionCube);

  private static native Status<?> nSetNormalMapOnWaterMaterial(
      long view, long nativeWaterMaterial, long normalMap);

  private static native Status<?> nSetNormalTilingOnWaterMaterial(
      long view, long nativeWaterMaterial, float normalTiling);

  private static native Status<?> nSetNormalSpeedOnWaterMaterial(
      long view, long nativeWaterMaterial, float normalSpeed);

  private static native Status<?> nSetAlphaStepUOnWaterMaterial(
      long view, long nativeWaterMaterial, float x, float y, float z, float w);

  private static native Status<?> nSetAlphaStepVOnWaterMaterial(
      long view, long nativeWaterMaterial, float x, float y, float z, float w);

  private static native Status<?> nSetAlphaStepMultiplierOnWaterMaterial(
      long view, long nativeWaterMaterial, float alphaStepMultiplier);

  private static native Status<?> nSetAlphaMapOnWaterMaterial(
      long view, long nativeWaterMaterial, long alphaMap);

  private static native Status<?> nSetNormalZOnWaterMaterial(
      long view, long nativeWaterMaterial, float normalZ);

  private static native Status<?> nSetNormalBoundaryOnWaterMaterial(
      long view, long nativeWaterMaterial, float normalBoundary);

  private static native void nDestroyNativeObject(long view, long nativeHandle);

  private static native Status<?> nSetMaterialOverride(
      long view, int impressNode, long nativeMaterial, String meshName);

  private static native Status<?> nSetEnvironmentLight(long view, long iblToken);

  private static native Status<?> nClearEnvironmentLight(long view);

  private static native Status<?> nDisposeAllResources(long view);

  // LINT.ThenChange(//depot/google3/third_party/impress/apibindings/impress_api.cc:api)
}
