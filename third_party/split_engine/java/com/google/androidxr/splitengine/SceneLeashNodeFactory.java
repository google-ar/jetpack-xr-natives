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
import com.android.extensions.xr.XrExtensions;
import com.android.extensions.xr.node.Node;
import com.android.extensions.xr.node.NodeTransaction;

/** Helper factory to create a scene node on a window leash. */
public final class SceneLeashNodeFactory {
  /** Wrapper class to hold a scene node on a window leash. */
  public static class SceneLeash {
    private SceneLeash(Node sceneNode, Node windowLeashNode) {
      this.sceneNode = sceneNode;
      this.windowLeashNode = windowLeashNode;
    }

    /**
     * Returns the scene node.
     *
     * @deprecated use {@link #getSceneNode()} instead.
     */
    @Deprecated public final Node sceneNode;

    /**
     * Returns the window leash node.
     *
     * @deprecated use {@link #getWindowLeashNode()} instead.
     */
    @Deprecated public final Node windowLeashNode;

    public Node getSceneNode() {
      return sceneNode;
    }

    public Node getWindowLeashNode() {
      return windowLeashNode;
    }
  }

  public static SceneLeash createSceneLeash(XrExtensions xrExtensions, Activity activity) {
    Node sceneNode = xrExtensions.createNode();
    Node windowLeashNode = xrExtensions.createNode();
    try (NodeTransaction transaction = xrExtensions.createNodeTransaction()) {
      transaction.setParent(windowLeashNode, sceneNode).apply();
    }
    // TODO: (broken link) - Check async results.
    xrExtensions.attachSpatialScene(
        activity, sceneNode, windowLeashNode, Runnable::run, (result) -> {});
    return new SceneLeash(sceneNode, windowLeashNode);
  }

  private SceneLeashNodeFactory() {}
}
