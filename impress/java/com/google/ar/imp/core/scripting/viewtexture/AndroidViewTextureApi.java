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

package com.google.ar.imp.core.scripting.viewtexture;

import static com.google.common.util.concurrent.MoreExecutors.directExecutor;

import android.view.View;
import androidx.annotation.Nullable;
import com.google.ar.imp.core.AndroidViewRenderer.CreateSurfaceTextureQuadRequest;
import com.google.ar.imp.core.AndroidViewRenderer.GetAttachedViewRequest;
import com.google.ar.imp.core.AndroidViewRenderer.InputForwardingMode;
import com.google.ar.imp.core.AndroidViewRenderer.UpdateSurfaceTextureQuadColliderRequest;
import com.google.ar.imp.core.scripting.ApiBridge;
import com.google.ar.imp.core.scripting.ApiRequest;
import com.google.ar.imp.core.scripting.Node;
import com.google.ar.imp.core.scripting.ScriptApi;
import com.google.ar.imp.proto.MaterialDefinition;
import com.google.ar.imp.proto.Math.Box;
import com.google.common.util.concurrent.Futures;
import com.google.common.util.concurrent.ListenableFuture;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** An extension to the Impress scripting API for rendering Android views to textured quads. */
public final class AndroidViewTextureApi {
  private final ApiBridge apiBridge;

  public AndroidViewTextureApi(ApiBridge apiBridge) {
    this.apiBridge = apiBridge;
  }

  /**
   * Sets up a node to hold a quad with the given view rendered continuously to texture. This also
   * sets AndroidViewRenderer's input forwarding mode to be INPUT_FORWARDING_MODE_DEFAULT, which
   * means it forwards PointerHitEvent as touch events. The material is set to the default unlit
   * texture external material. The blend priority is set to the default blend priority.
   */
  public ListenableFuture<Void> attachViewToNode(Node node, View view) {
    return attachViewToNode(node, view, 0, 0);
  }

  /**
   * Sets up a node to hold a quad with the given view rendered continuously to texture. This also
   * sets AndroidViewRenderer's input forwarding mode to be INPUT_FORWARDING_MODE_DEFAULT, which
   * means it forwards PointerHitEvent as touch events. The material is set to the default unlit
   * texture external material. The blend priority is set to the default blend priority.
   *
   * <p>The width and height correspond to the size of the surface texture the view is rendered to.
   */
  public ListenableFuture<Void> attachViewToNode(Node node, View view, int width, int height) {
    return attachViewToNode(
        node, view, width, height, InputForwardingMode.INPUT_FORWARDING_MODE_DEFAULT);
  }

  /**
   * Sets up a node to hold a quad with the given view rendered continuously to texture. The
   * material is set to the default unlit external material. The blend priority is set to the
   * default blend priority.
   */
  public ListenableFuture<Void> attachViewToNode(
      Node node, View view, int width, int height, InputForwardingMode inputForwardingMode) {
    return attachViewToNode(node, view, width, height, inputForwardingMode, null);
  }

  /**
   * Sets up a node to hold a quad with the given view rendered continuously to texture. The blend
   * priority is set to the default blend priority.
   */
  public ListenableFuture<Void> attachViewToNode(
      Node node,
      View view,
      int width,
      int height,
      InputForwardingMode inputForwardingMode,
      @Nullable MaterialDefinition material) {
    return attachViewToNode(node, view, width, height, inputForwardingMode, material, null);
  }

  /** Sets up a node to hold a quad with the given view rendered continuously to texture. */
  public ListenableFuture<Void> attachViewToNode(
      Node node,
      View view,
      int width,
      int height,
      InputForwardingMode inputForwardingMode,
      @Nullable MaterialDefinition material,
      @Nullable Integer blendPriority) {
    CreateSurfaceTextureQuadRequest.Builder request =
        CreateSurfaceTextureQuadRequest.newBuilder()
            .setTarget(node.getNodeHandle())
            .setInputForwardingMode(inputForwardingMode)
            .setWidth(width)
            .setHeight(height);
    if (material != null) {
      request.setMaterial(material);
    }
    if (blendPriority != null) {
      request.setBlendPriority(blendPriority);
    }
    return apiBridge.sendRequestAsync(
        new ApiRequest<CreateSurfaceTextureQuadRequest>(
            "imp.android.CreateSurfaceTextureQuadRequest", request.build(), Arrays.asList(view)));
  }

  /** Updates the collider of the Android View. */
  public ListenableFuture<Void> updateCollider(Node node, Box collider) {
    UpdateSurfaceTextureQuadColliderRequest request =
        UpdateSurfaceTextureQuadColliderRequest.newBuilder()
            .setTarget(node.getNodeHandle())
            .setCollider(collider)
            .build();
    return apiBridge.sendRequestAsync(
        new ApiRequest<UpdateSurfaceTextureQuadColliderRequest>(
            "imp.android.UpdateSurfaceTextureQuadColliderRequest", request));
  }

  /**
   * Creates a new node and calls attachViewToNode.
   *
   * <p>Note: this is a convenience helper for backwards compatibility.
   */
  public ListenableFuture<Node> createAndroidViewNode(View view) {
    ScriptApi scriptApi = new ScriptApi(apiBridge);
    Node node = scriptApi.createNode();
    return Futures.transform(
        attachViewToNode(node, view), (emptyMessage) -> node, directExecutor());
  }

  // Gets the View that this Node is attached-to/rendering.
  public View getAttachedView(Node node) {
    GetAttachedViewRequest request =
        GetAttachedViewRequest.newBuilder().setTarget(node.getNodeHandle()).build();
    List<Object> out = new ArrayList<>();
    apiBridge.sendRequest(
        new ApiRequest<GetAttachedViewRequest>(
            "imp.android.GetAttachedViewRequest", request, null, out));
    return (View) out.get(0);
  }
}
