/*
 * Copyright 2025 Google LLC
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

package com.google.androidxr.splitengine;

import android.app.Activity;
import android.util.DisplayMetrics;
import androidx.annotation.Nullable;
import androidx.xr.scenecore.impl.JxrPlatformAdapterAxr;
import androidx.xr.scenecore.impl.perception.Fov;
import androidx.xr.scenecore.impl.perception.Pose;
import androidx.xr.scenecore.impl.perception.ViewProjection;
import androidx.xr.scenecore.impl.perception.ViewProjections;
import com.android.extensions.xr.node.Node;
import com.google.ar.imp.view.splitengine.ImpSplitEngine;
import com.google.common.util.concurrent.ThreadFactoryBuilder;
import java.util.concurrent.Executors;

/**
 * Provides the view update parameters for split engine to update the camera. These parameters are
 * fed into the SplitEngine Renderer to set up the camera based on the camera's FOV and pose coming
 * from the headless OpenXR Session held by JXRCore. Additionally, this class provides the screen
 * size of the device by querying the window manager.
 */
@SuppressWarnings("RestrictTo")
public class SplitEngineViewUpdateProvider implements ImpSplitEngine.SplitEngineViewParamsProvider {

  private static final String TAG = SplitEngineViewUpdateProvider.class.getSimpleName();
  private final Activity activity;
  private final JxrPlatformAdapterAxr platformAdapterAxr;

  public SplitEngineViewUpdateProvider(Activity activity, Node sceneNode, Node windowLeashNode) {
    this.activity = activity;
    this.platformAdapterAxr =
        JxrPlatformAdapterAxr.create(
            activity,
            Executors.newSingleThreadScheduledExecutor(
                new ThreadFactoryBuilder().setNameFormat("axroptimizedapp").build()),
            sceneNode,
            windowLeashNode);
  }

  @Override
  public ImpSplitEngine.ScreenSize getScreenSize() {
    DisplayMetrics displayMetrics = new DisplayMetrics();
    activity.getWindowManager().getDefaultDisplay().getMetrics(displayMetrics);
    return new ImpSplitEngine.ScreenSize(displayMetrics.widthPixels, displayMetrics.heightPixels);
  }

  /**
   * Returns the view update parameters for the current frame. This might return null if the
   * Perception session held by RealityCore has not yet been initialized.
   */
  @Override
  @Nullable
  public ImpSplitEngine.ViewUpdateParams getViewUpdateParams() {
    ViewProjections views = platformAdapterAxr.getStereoViewsInOpenXrUnboundedSpace();
    if (views == null) {
      return null;
    }
    return createViewUpdateParams(views);
  }

  // TODO: traorem - Remove this once SceneViewerXR can use JXR plane APIs directly.
  public long getNativeSession() {
    return platformAdapterAxr.getNativeSession();
  }

  // TODO: traorem - Remove this once SceneViewerXR can use JXR plane APIs directly.
  public long getNativeInstance() {
    return platformAdapterAxr.getNativeInstance();
  }

  private ImpSplitEngine.ViewUpdateParams createViewUpdateParams(ViewProjections views) {
    return new ImpSplitEngine.ViewUpdateParams(
        createViewProjection(views.getLeftEye()), createViewProjection(views.getRightEye()));
  }

  private ImpSplitEngine.ViewProjection createViewProjection(ViewProjection viewProjection) {
    return new ImpSplitEngine.ViewProjection(
        createFov(viewProjection.getFov()), createPose(viewProjection.getPose()));
  }

  private ImpSplitEngine.Fov createFov(Fov fov) {
    return new ImpSplitEngine.Fov(
        fov.getAngleLeft(), fov.getAngleRight(), fov.getAngleUp(), fov.getAngleDown());
  }

  private ImpSplitEngine.Pose createPose(Pose pose) {
    return new ImpSplitEngine.Pose(
        pose.tx(), pose.ty(), pose.tz(), pose.qx(), pose.qy(), pose.qz(), pose.qw());
  }
}
