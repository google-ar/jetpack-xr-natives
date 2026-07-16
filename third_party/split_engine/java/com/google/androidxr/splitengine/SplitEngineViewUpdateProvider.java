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
import androidx.xr.arcore.RenderViewpoint;
import androidx.xr.runtime.DeviceTrackingMode;
import androidx.xr.runtime.FieldOfView;
import androidx.xr.runtime.Session;
import androidx.xr.runtime.SessionCreateResult;
import androidx.xr.runtime.SessionCreateSuccess;
import androidx.xr.runtime.math.Pose;
import androidx.xr.runtime.math.Quaternion;
import androidx.xr.runtime.math.Vector3;
import com.android.extensions.xr.node.Node;
import com.google.ar.imp.view.splitengine.ImpSplitEngine;

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
  private final RenderViewpoint left;
  private final RenderViewpoint right;

  public SplitEngineViewUpdateProvider(Activity activity, Node sceneNode, Node windowLeashNode) {
    this(createSession(activity), activity, sceneNode, windowLeashNode);
  }

  public SplitEngineViewUpdateProvider(
      Session session, Activity activity, Node sceneNode, Node windowLeashNode) {
    this.activity = activity;
    left = RenderViewpoint.left(session);
    right = RenderViewpoint.right(session);
  }

  private static Session createSession(Activity activity) {
    SessionCreateResult result = Session.create(activity);
    if (result instanceof SessionCreateSuccess sessionCreateSuccess) {
      Session session = sessionCreateSuccess.getSession();
      session.configure(
          session
              .getConfig()
              .copy(
                  session.getConfig().getPlaneTracking(),
                  session.getConfig().getHandTracking(),
                  DeviceTrackingMode.LAST_KNOWN));
      return session;
    } else {
      throw new IllegalStateException("Failed to create session.");
    }
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
    if (left == null || right == null) {
      return null;
    }
    return createViewUpdateParams(left, right);
  }

  private ImpSplitEngine.ViewUpdateParams createViewUpdateParams(
      RenderViewpoint left, RenderViewpoint right) {
    return new ImpSplitEngine.ViewUpdateParams(
        createViewProjection(left.getState().getValue()),
        createViewProjection(right.getState().getValue()));
  }

  private ImpSplitEngine.ViewProjection createViewProjection(RenderViewpoint.State viewProjection) {
    return new ImpSplitEngine.ViewProjection(
        createFov(viewProjection.getFieldOfView()), createPose(viewProjection.getPose()));
  }

  private ImpSplitEngine.Fov createFov(FieldOfView fov) {
    return new ImpSplitEngine.Fov(
        fov.getAngleLeft(), fov.getAngleRight(), fov.getAngleUp(), fov.getAngleDown());
  }

  private ImpSplitEngine.Pose createPose(Pose pose) {
    Vector3 translation = pose.getTranslation();
    Quaternion rotation = pose.getRotation();
    return new ImpSplitEngine.Pose(
        translation.getX(),
        translation.getY(),
        translation.getZ(),
        rotation.getX(),
        rotation.getY(),
        rotation.getZ(),
        rotation.getW());
  }
}
