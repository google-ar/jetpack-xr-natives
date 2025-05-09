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

package com.google.ar.imp.core.scripting;

import android.content.Context;
import android.os.Build;
import android.view.Surface;
import android.view.WindowManager;
import androidx.annotation.RequiresApi;
import com.google.ar.imp.core.NodeHandleMessageOuterClass.NodeHandleMessage;
import com.google.ar.imp.core.SurfaceRendererScripting.CreateNodeWithSurfaceRendererRequest;
import com.google.ar.imp.core.SurfaceRendererScripting.SetCameraRequest;
import com.google.ar.imp.core.SurfaceRendererScripting.SetGroupRequest;
import com.google.ar.imp.core.SurfaceRendererScripting.SetViewPortSizeRequest;
import com.google.ar.imp.core.SurfaceRendererScripting.SurfaceRendererSettings;
import com.google.ar.imp.proto.Math.UInt2;
import com.google.ar.imp.view.View;
import java.util.Arrays;

/** An extension to Impress scripting API for managing SurfaceRenderers. */
public final class SurfaceRendererApi {
  private static final String CREATE_NODE_WITH_SURFACE_RENDERER_REQUEST_URL =
      "imp.surface_renderer.CreateNodeWithSurfaceRendererRequest";
  private static final String SET_CAMERA_REQUEST_URL = "imp.surface_renderer.SetCameraRequest";
  private static final String SET_GROUP_REQUEST_URL = "imp.surface_renderer.SetGroupRequest";
  private static final String SET_VIEW_PORT_SIZE_REQUEST_URL =
      "imp.surface_renderer.SetViewPortSizeRequest";

  private final ApiBridge apiBridge;

  public static SurfaceRendererApi create(View view) {
    // These three lines are the "enable scripting idiom". You can copy this small block into your
    // Impress-based Java activity to use the Java scripting interface.
    ScriptBridge scriptBridge = new ScriptBridge(view.getViewHostHandle());
    view.setScriptEndpoint(scriptBridge);
    return new SurfaceRendererApi(scriptBridge);
  }

  public SurfaceRendererApi(ApiBridge apiBridge) {
    this.apiBridge = apiBridge;
  }

  /** Create a new node with a SurfaceRenderer component */
  public Node createNodeWithSurfaceRenderer(Surface surface, String group, String cameraNodeName) {
    SurfaceRendererSettings settings =
        SurfaceRendererSettings.newBuilder()
            .setGroup(group)
            .setCameraNodeName(cameraNodeName)
            .build();

    return createNodeWithSurfaceRenderer(surface, settings);
  }

  public Node createNodeWithSurfaceRenderer(Surface surface, SurfaceRendererSettings settings) {
    return createNodeWithSurfaceRenderer(surface, null, settings);
  }

  /**
   * Specify which Context this surface is associated with. This will populate a displayId in
   * SurfaceRendererSettings and the created SurfaceRenderer.
   */
  public Node createNodeWithSurfaceRenderer(
      Surface surface, Context context, SurfaceRendererSettings settings) {
    SurfaceRendererSettings.Builder settingsBuilder = SurfaceRendererSettings.newBuilder();
    settingsBuilder.mergeFrom(settings);

    if (context != null) {
      final int displayId =
          Build.VERSION.SDK_INT >= Build.VERSION_CODES.R
              ? getDisplayIdAtLeastR(context)
              : getDisplayIdBeforeR(context);
      settingsBuilder.setDisplayId(displayId);
    }

    CreateNodeWithSurfaceRendererRequest request =
        CreateNodeWithSurfaceRendererRequest.newBuilder().setSettings(settingsBuilder).build();

    NodeHandleMessage nodeHandleMessage =
        apiBridge.sendRequest(
            new ApiRequest<CreateNodeWithSurfaceRendererRequest>(
                CREATE_NODE_WITH_SURFACE_RENDERER_REQUEST_URL, request, Arrays.asList(surface)),
            NodeHandleMessage.class);

    return new Node(apiBridge, nodeHandleMessage);
  }

  /** Specify which camera to use for the SurfaceRenderer */
  public void setCamera(Node surfaceRendererNode, String cameraNodeName) {
    SetCameraRequest request =
        SetCameraRequest.newBuilder()
            .setSurfaceRendererNode(surfaceRendererNode.getNodeHandle())
            .setCameraNodeName(cameraNodeName)
            .build();
    apiBridge.sendRequest(SET_CAMERA_REQUEST_URL, request);
  }

  /** Specify the group for the SurfaceRenderer */
  public void setGroup(Node surfaceRendererNode, String group) {
    SetGroupRequest request =
        SetGroupRequest.newBuilder()
            .setSurfaceRendererNode(surfaceRendererNode.getNodeHandle())
            .setGroup(group)
            .build();
    apiBridge.sendRequest(SET_GROUP_REQUEST_URL, request);
  }

  /** Specify the view port size for the SurfaceRenderer */
  public void setViewPortSize(Node surfaceRendererNode, int width, int height) {
    UInt2 viewPortSize = UInt2.newBuilder().setX(width).setY(height).build();
    SetViewPortSizeRequest request =
        SetViewPortSizeRequest.newBuilder()
            .setSurfaceRendererNode(surfaceRendererNode.getNodeHandle())
            .setViewPortSize(viewPortSize)
            .build();
    apiBridge.sendRequest(SET_VIEW_PORT_SIZE_REQUEST_URL, request);
  }

  @RequiresApi(Build.VERSION_CODES.R)
  private static int getDisplayIdAtLeastR(Context context) {
    return context.getDisplay().getDisplayId();
  }

  private static int getDisplayIdBeforeR(Context context) {
    WindowManager windowManager = (WindowManager) context.getSystemService(Context.WINDOW_SERVICE);
    return windowManager.getDefaultDisplay().getDisplayId();
  }
}
