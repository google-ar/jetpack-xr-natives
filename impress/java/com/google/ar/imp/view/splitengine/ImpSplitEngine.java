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

package com.google.ar.imp.view.splitengine;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

/** A class that abstracts the state of the SplitEngine from the ImpSplitEngineApi. */
public final class ImpSplitEngine {
  /**
   * Parameters for setting up the ImpSplitEngineApi. Similar to SetupParams proto but Jetpack
   * policy requires we not use protos.
   */
  public static class SplitEngineSetupParams {
    @Nullable public String jniLibraryName;
    @Nullable public String viewIdentifier;
    public int bridgeBufferSizeKb;
  }

  /** The screen size in pixels. */
  public static class ScreenSize {
    private final int widthPixels;
    private final int heightPixels;

    public ScreenSize(int widthPixels, int heightPixels) {
      this.widthPixels = widthPixels;
      this.heightPixels = heightPixels;
    }

    public int getWidthPixels() {
      return widthPixels;
    }

    public int getHeightPixels() {
      return heightPixels;
    }
  }

  /** The field of view. */
  public static class Fov {
    private final float angleLeft;
    private final float angleRight;
    private final float angleUp;
    private final float angleDown;

    public Fov(float angleLeft, float angleRight, float angleUp, float angleDown) {
      this.angleLeft = angleLeft;
      this.angleRight = angleRight;
      this.angleUp = angleUp;
      this.angleDown = angleDown;
    }

    public float getAngleLeft() {
      return angleLeft;
    }

    public float getAngleRight() {
      return angleRight;
    }

    public float getAngleUp() {
      return angleUp;
    }

    public float getAngleDown() {
      return angleDown;
    }
  }

  /** Pose represented as a Vec3 translation and a Quat4 rotation. */
  public static class Pose {
    private final float tx;
    private final float ty;
    private final float tz;

    private final float qx;
    private final float qy;
    private final float qz;
    private final float qw;

    public Pose(float tx, float ty, float tz, float qx, float qy, float qz, float qw) {
      this.tx = tx;
      this.ty = ty;
      this.tz = tz;

      this.qx = qx;
      this.qy = qy;
      this.qz = qz;
      this.qw = qw;
    }

    public float tx() {
      return tx;
    }

    public float ty() {
      return ty;
    }

    public float tz() {
      return tz;
    }

    public float qx() {
      return qx;
    }

    public float qy() {
      return qy;
    }

    public float qz() {
      return qz;
    }

    public float qw() {
      return qw;
    }
  }

  /** Represents the view parameters of a 3D frustum. */
  public static class ViewProjection {
    private final Fov fov;
    private final Pose pose;

    public ViewProjection(@NonNull Fov fov, @NonNull Pose pose) {
      this.fov = fov;
      this.pose = pose;
    }

    @NonNull
    public Fov getFov() {
      return fov;
    }

    @NonNull
    public Pose getPose() {
      return pose;
    }
  }

  /**
   * The parameters used by the SplitEngine to set up the camera.
   *
   * <p>This is a wrapper around native params in C++. The instance creater is expected to manage
   * the lifecycle of the native params using `destroyNativeParams()`.
   */
  public static class ViewUpdateParams {
    // Reference to the native object for the ViewUpdateParams. 0 means the native object has been
    // freed.
    private long nativeHandleParams;

    // Reference to the native object for the left eye ViewProjection of the ViewUpdateParams. 0
    // means the native object has been freed.
    private long nativeHandleLeftEye;

    // Reference to the native object for the right eye ViewProjection of the ViewUpdateParams. 0
    // means the native object has been freed.
    private long nativeHandleRightEye;

    public ViewUpdateParams(@NonNull ViewProjection leftEye, @NonNull ViewProjection rightEye) {
      nativeHandleParams = nCreateViewUpdateParams();

      nativeHandleLeftEye = nCreateViewProjection();
      nSetViewProjection(
          nativeHandleLeftEye,
          leftEye.getFov().getAngleLeft(),
          leftEye.getFov().getAngleRight(),
          leftEye.getFov().getAngleUp(),
          leftEye.getFov().getAngleDown(),
          leftEye.getPose().tx(),
          leftEye.getPose().ty(),
          leftEye.getPose().tz(),
          leftEye.getPose().qx(),
          leftEye.getPose().qy(),
          leftEye.getPose().qz(),
          leftEye.getPose().qw());

      nativeHandleRightEye = nCreateViewProjection();
      nSetViewProjection(
          nativeHandleRightEye,
          rightEye.getFov().getAngleLeft(),
          rightEye.getFov().getAngleRight(),
          rightEye.getFov().getAngleUp(),
          rightEye.getFov().getAngleDown(),
          rightEye.getPose().tx(),
          rightEye.getPose().ty(),
          rightEye.getPose().tz(),
          rightEye.getPose().qx(),
          rightEye.getPose().qy(),
          rightEye.getPose().qz(),
          rightEye.getPose().qw());

      nSetViewUpdateParams(nativeHandleParams, nativeHandleLeftEye, nativeHandleRightEye);
    }

    public void destroyNativeParams() {
      if (nativeHandleParams != 0) {
        nDestroyViewUpdateParams(nativeHandleParams);
        nativeHandleParams = 0;
      }
      if (nativeHandleLeftEye != 0) {
        nDestroyViewProjection(nativeHandleLeftEye);
        nativeHandleLeftEye = 0;
      }
      if (nativeHandleRightEye != 0) {
        nDestroyViewProjection(nativeHandleRightEye);
        nativeHandleRightEye = 0;
      }
    }

    public long getNativeHandle() {
      return nativeHandleParams;
    }
  }

  /**
   * A class that provides screen size and the parameters needed to set up the camera to the
   * SplitEngine.
   */
  // TODO: (broken link) - get the parameters in streaming based approach.
  public static interface SplitEngineViewParamsProvider {
    ScreenSize getScreenSize();

    @Nullable
    ViewUpdateParams getViewUpdateParams();
  }

  private ImpSplitEngine() {}

  private static native long nCreateViewUpdateParams();

  private static native void nSetViewUpdateParams(
      long paramsHandle, long leftEyeHandle, long rightEyeHandle);

  private static native void nDestroyViewUpdateParams(long paramsHandle);

  private static native long nCreateViewProjection();

  private static native void nSetViewProjection(
      long projectionHandle,
      float angleLeft,
      float angleRight,
      float angleUp,
      float angleDown,
      float translateX,
      float translateY,
      float translateZ,
      float quaternionX,
      float quaternionY,
      float quaternionZ,
      float quaternionW);

  private static native void nDestroyViewProjection(long projectionHandle);
}
