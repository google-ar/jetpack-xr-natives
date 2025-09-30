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
import android.os.StrictMode;
import android.os.StrictMode.ThreadPolicy;
import android.util.Log;
import androidx.annotation.Nullable;
import com.android.extensions.xr.XrExtensions;
import com.android.extensions.xr.node.InputEvent;
import com.android.extensions.xr.node.Mat4f;
import com.android.extensions.xr.node.Node;
import com.android.extensions.xr.node.NodeTransaction;
import com.android.extensions.xr.node.NodeTransform;
import com.android.extensions.xr.subspace.Subspace;
import com.google.androidxr.splitengine.SceneLeashNodeFactory.SceneLeash;
import com.google.ar.imp.view.FrameScheduler;
import com.google.ar.imp.view.View;
import com.google.ar.imp.view.splitengine.ImpSplitEngineRenderer;
import com.google.imp.splitengine.extensions.AndroidXrRendererConnection;
import com.google.imp.splitengine.extensions.IRendererConnection;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;

/** Manages CPM nodes created for SplitEgine applications. */
public class SplitEngineSubspaceManager {

  private static final String TAG = SplitEngineSubspaceManager.class.getSimpleName();

  @SuppressWarnings("NonFinalStaticField")
  private static volatile boolean libraryLoaded = false;

  // Default library to load if none is provided. Matches the name from imp.bzl
  // returned by imp_default_jni_binary_name()
  // LINT.IfChange
  private static final String DEFAULT_LIBRARY_NAME = "imp_view_split_engine_jni";
  // LINT.ThenChange(//depot/google3/third_party/impress/build_tools/imp.bzl)

  @Nullable private final XrExtensions xrExtensions;
  private final Node applicationSceneNode;
  private final Node applicationWindowLeashNode;
  private View view;
  private final IRendererConnection systemRendererConnection;
  private long subspaceManagerNativeHandle;
  private final FrameScheduler frameScheduler;

  private final HashMap<Integer, SubspaceNode> subspaceNodes = new HashMap<>();

  // Creates a new subspace referring to the subspaceName. Returns the subspace node.
  public SubspaceNode createSubspace(String subspaceName) {
    return createSubspace(subspaceName, 0);
  }

  public SubspaceNode createSubspace(String subspaceName, int existingRootEntityId) {
    int subspaceId = nGetNextSubspaceId(subspaceManagerNativeHandle);
    return createSubspace(subspaceId, subspaceName, existingRootEntityId);
  }

  // Creates a new subspace, and registers it with the Split Engine Subspace Manager.
  private SubspaceNode createSubspace(int subspaceId, String subspaceName) {
    // 0 is the invalid node, which means a new one will be automatically created.
    return createSubspace(subspaceId, subspaceName, 0);
  }

  // Creates a new subspace, and registers it with the Split Engine Subspace Manager.
  private SubspaceNode createSubspace(
      int subspaceId, String subspaceName, int existingRootEntityId) {
    SubspaceNode subspaceNode = createSubspace(subspaceId);
    try (NodeTransaction transaction = xrExtensions.createNodeTransaction()) {
      transaction
          .setName(subspaceNode.getSubspaceNode(), subspaceName + "_system_side_subspace_root")
          .apply();
    }
    nRegisterSubspace(subspaceManagerNativeHandle, subspaceId, existingRootEntityId);
    nCreateSubspace(subspaceManagerNativeHandle, subspaceId, subspaceName);
    return subspaceNode;
  }

  private SubspaceNode createSubspace(int subspaceId) {
    Log.d(TAG, String.format("Creating Subspace with ID %d", subspaceId));
    Node node = xrExtensions.createNode();

    try (NodeTransaction transaction = xrExtensions.createNodeTransaction()) {
      Subspace subspace =
          xrExtensions.createSubspace(
              ((AndroidXrRendererConnection) systemRendererConnection).getConnectionHandle(),
              subspaceId);
      transaction.setSubspace(node, subspace).setParent(node, applicationSceneNode).apply();
    }
    SubspaceNode subspaceNode = new SubspaceNode(subspaceId, node);
    subspaceNode.nodeTransformSubscription =
        subspaceNode
            .getSubspaceNode()
            .subscribeToTransform(
                Runnable::run,
                (NodeTransform transformUpdate) ->
                    updateSubspaceTransform(subspaceId, transformUpdate.getTransform()));
    subspaceNodes.put(subspaceId, subspaceNode);
    return subspaceNode;
  }

  // Forwards an input event to the subspace with ID.
  public void forwardInputEvent(InputEvent event, int subspaceId) {
    if (SplitEngineInputEvent.isInputEventValidForSubspace(event)) {
      frameScheduler.runOnFrameThread(
          () -> {
            // TODO: Improve memory allocation on native side.
            SplitEngineInputEvent wrappedEvent = SplitEngineInputEvent.createFromInputEvent(event);
            nForwardInputEvent(
                subspaceManagerNativeHandle, subspaceId, wrappedEvent.getNativeHandle());
            wrappedEvent.destroyNativeEvent();
          });
    }
  }

  // Anchors the subspace with ID to the given anchor.
  public void anchorSubspace(int subspaceId, SubspaceNode.Anchor anchor) {
    SubspaceNode node = subspaceNodes.get(subspaceId);
    if (node == null) {
      Log.e(TAG, "No node found for subspace " + subspaceId);
      return;
    }
    @SuppressWarnings("EnumOrdinal")
    int anchorType = anchor.ordinal();
    nUpdateSubspaceAnchor(subspaceManagerNativeHandle, subspaceId, anchorType);
    node.anchor = anchor;
  }

  /** Returns the node that is the parent of the application window. */
  public Node getWindowLeashNode() {
    return applicationWindowLeashNode;
  }

  /** Returns the node that is the parent of all subspaces. */
  public Node getSceneNode() {
    return applicationSceneNode;
  }

  // Shows the subspace with ID.
  public void showSubspace(int subspaceId) {
    Node node = subspaceNodes.get(subspaceId).getSubspaceNode();
    if (node != null) {
      try (NodeTransaction transaction = xrExtensions.createNodeTransaction()) {
        transaction.setParent(node, applicationSceneNode).apply();
      }
    } else {
      Log.e(TAG, "No node found for subspace " + subspaceId);
    }
  }

  // Hides the subspace with ID.
  public void hideSubspace(int subspaceId) {
    Node node = subspaceNodes.get(subspaceId).getSubspaceNode();
    if (node != null) {
      try (NodeTransaction transaction = xrExtensions.createNodeTransaction()) {
        transaction.setParent(node, null).apply();
      }
    } else {
      Log.e(TAG, "No node found for subspace " + subspaceId);
    }
  }

  // Deletes the subspace with ID.
  // Any reference to the subspace node in the application should also be dropped.
  // This also destroys the subspace node in impress, and any attached nodes along with the subspace
  // content.
  public void deleteSubspace(int subspaceId) {
    SubspaceNode subspaceNode = subspaceNodes.get(subspaceId);
    if (subspaceNode != null) {
      nDestroySubspace(subspaceManagerNativeHandle, subspaceId);
      closePreviousSubscription(subspaceNode);
      // Dropping the subspace node will trigger the deletion of the subspace in the system.
      subspaceNodes.remove(subspaceId);
    } else {
      Log.e(TAG, "No node found for subspace " + subspaceId);
    }
  }

  // Destroys the subspace manager.
  public void destroy() {
    for (SubspaceNode subspaceNode : new ArrayList<>(subspaceNodes.values())) {
      deleteSubspace(subspaceNode.subspaceId);
    }
    nDestroySubspaceManager(subspaceManagerNativeHandle, view.getViewHostHandle());
  }

  public SplitEngineSubspaceManager(
      Activity activity, ImpSplitEngineRenderer renderer, @Nullable XrExtensions xrExtensions) {
    this(renderer, xrExtensions, SceneLeashNodeFactory.createSceneLeash(xrExtensions, activity));
  }

  private SplitEngineSubspaceManager(
      ImpSplitEngineRenderer renderer, @Nullable XrExtensions xrExtensions, SceneLeash sceneLeash) {
    this(
        renderer,
        xrExtensions,
        sceneLeash.getSceneNode(),
        sceneLeash.getWindowLeashNode(),
        DEFAULT_LIBRARY_NAME);
  }

  // Alternate constructor that can be used to initialize the SplitEngineSubspaceManager.
  public SplitEngineSubspaceManager(
      ImpSplitEngineRenderer renderer,
      @Nullable XrExtensions xrExtensions,
      Node applicationSceneNode,
      Node applicationWindowLeashNode) {
    this(
        renderer,
        xrExtensions,
        applicationSceneNode,
        applicationWindowLeashNode,
        DEFAULT_LIBRARY_NAME);
  }

  public SplitEngineSubspaceManager(
      ImpSplitEngineRenderer renderer,
      @Nullable XrExtensions xrExtensions,
      Node applicationSceneNode,
      Node applicationWindowLeashNode,
      String nativeLibrary) {
    this.xrExtensions = xrExtensions;
    this.applicationSceneNode = applicationSceneNode;
    this.applicationWindowLeashNode = applicationWindowLeashNode;

    if (nativeLibrary == null || nativeLibrary.isEmpty()) {
      nativeLibrary = DEFAULT_LIBRARY_NAME;
    }
    loadLibrary(nativeLibrary);

    this.view = renderer.getView();
    this.systemRendererConnection = renderer.getRendererConnection();
    this.frameScheduler = renderer.getFrameScheduler();
    this.subspaceManagerNativeHandle = nSetupNativeSubspaceManager(view.getViewHostHandle());
  }

  protected static synchronized void loadLibrary(String nativeLibraryName) {
    if (libraryLoaded) {
      return;
    }

    // Temporarily allow disk reads to load the library.
    ThreadPolicy oldPolicy = StrictMode.getThreadPolicy();
    StrictMode.setThreadPolicy(new ThreadPolicy.Builder(oldPolicy).permitDiskReads().build());
    Log.i(TAG, "Loading native library: " + nativeLibraryName);
    try {
      System.loadLibrary(nativeLibraryName);
    } catch (UnsatisfiedLinkError e) {
      Log.e(TAG, "Unable to load " + nativeLibraryName);
      return;
    } finally {
      StrictMode.setThreadPolicy(oldPolicy);
    }
    libraryLoaded = true;
  }

  private void updateSubspaceTransform(int subspaceId, Mat4f transform) {
    nForwardSubspaceTransform(
        subspaceManagerNativeHandle, subspaceId, transform.getFlattenedMatrix());
  }

  private void closePreviousSubscription(SubspaceNode subspaceNode) {
    if (subspaceNode.nodeTransformSubscription != null) {
      try {
        subspaceNode.nodeTransformSubscription.close();
        subspaceNode.nodeTransformSubscription = null;
      } catch (IOException e) {
        Log.e(TAG, "No subscription to close for subspace " + subspaceNode.subspaceId);
      }
    }
  }

  // LINT.IfChange
  private static native long nSetupNativeSubspaceManager(long viewHandle);

  private static native int nGetNextSubspaceId(long subspaceManagerHandle);

  private static native void nRegisterSubspace(
      long subspaceManagerHandle, int subspaceId, int existingRootEntityId);

  private static native void nCreateSubspace(
      long subspaceManagerHandle, int subspaceId, String subspaceName);

  private static native void nDestroySubspace(long subspaceManagerHandle, int subspaceId);

  private static native void nForwardInputEvent(
      long subspaceManagerHandle, int subspaceId, long inputEventHandle);

  private static native void nUpdateSubspaceAnchor(
      long subspaceManagerHandle, int subspaceId, int anchorType);

  private static native void nForwardSubspaceTransform(
      long subspaceManagerHandle, int subspaceId, float[] transform);

  private static native void nDestroySubspaceManager(
      long subspaceManagerHandle, long viewHostHandle);
  // LINT.ThenChange(//depot/google3/third_party/split_engine/subspace_jni.cc)
}
