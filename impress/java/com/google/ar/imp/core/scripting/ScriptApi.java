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

import com.google.ar.imp.core.NodeHandleMessageOuterClass.NodeHandleMessage;
import com.google.ar.imp.core.scripting.Api.CreateNodeRequest;
import com.google.ar.imp.core.scripting.Api.FindNodeRequest;
import com.google.ar.imp.core.scripting.Api.GetCameraRequest;
import com.google.ar.imp.core.scripting.Api.LoadAndApplyEnvironmentLightFromInputStreamRequest;
import com.google.ar.imp.core.scripting.Api.LoadModelFromInputStreamRequest;
import com.google.ar.imp.core.scripting.Api.LoadModelRequest;
import com.google.ar.imp.core.scripting.Api.LoadSceneRequest;
import com.google.ar.imp.core.scripting.Api.SetCameraProjectionRequest;
import com.google.ar.imp.proto.Math.Mat4f;
import com.google.ar.imp.view.View;
import com.google.common.util.concurrent.Futures;
import com.google.common.util.concurrent.ListenableFuture;
import com.google.common.util.concurrent.MoreExecutors;
import com.google.protobuf.MessageLite;
import java.io.InputStream;
import java.util.Arrays;
import java.util.function.Consumer;

/** The default Impress API for loading models, sending and receiving events to/from native, etc. */
public class ScriptApi {
  private final ApiBridge apiBridge;

  public static ScriptApi create(View view) {
    // These three lines are the "enable scripting idiom". You can copy this small block into your
    // Impress-based Java activity to use the Java scripting interface.
    ScriptBridge scriptBridge = new ScriptBridge(view.getViewHostHandle());
    view.setScriptEndpoint(scriptBridge);
    return new ScriptApi(scriptBridge);
  }

  public ScriptApi(ApiBridge apiBridge) {
    this.apiBridge = apiBridge;
  }

  /** Returns the bridge so other scripting APIs can be created. */
  public ApiBridge getBridge() {
    return apiBridge;
  }

  /** Loads the model from the given uri. If the load fails, an ApiException will be set. */
  public ListenableFuture<Node> loadModel(String uri) {
    LoadModelRequest loadModel =
        LoadModelRequest.newBuilder().setRemoteUri(uri).setEnabled(true).build();
    ListenableFuture<NodeHandleMessage> future =
        apiBridge.sendRequestAsync(
            "imp.scripting.LoadModelRequest", loadModel, NodeHandleMessage.class);
    return Futures.transform(
        future,
        (nodeHandleMessage) -> new Node(apiBridge, nodeHandleMessage),
        MoreExecutors.directExecutor());
  }

  /**
   * Loads the model from the given input stream. modelKey is used to identify the asset for caching
   * purposes when re-using the asset.
   */
  public ListenableFuture<Node> loadModel(InputStream stream, String modelKey) {
    LoadModelFromInputStreamRequest loadModelFromInputStream =
        LoadModelFromInputStreamRequest.newBuilder().setModelKey(modelKey).setEnabled(true).build();
    ListenableFuture<NodeHandleMessage> future =
        apiBridge.sendRequestAsync(
            new ApiRequest<LoadModelFromInputStreamRequest>(
                "imp.scripting.LoadModelFromInputStreamRequest",
                loadModelFromInputStream,
                Arrays.asList(stream)),
            NodeHandleMessage.class);
    return Futures.transform(
        future,
        (nodeHandleMessage) -> new Node(apiBridge, nodeHandleMessage),
        MoreExecutors.directExecutor());
  }

  /**
   * Loads the pre-built image based lighting from the given input stream and applies it to the
   * environment light . iblKey is used to identify the asset for caching purposes when re-using the
   * asset.
   */
  public ListenableFuture<Void> loadAndApplyEnvironmentLight(InputStream stream, String iblKey) {
    LoadAndApplyEnvironmentLightFromInputStreamRequest loadAndApplyEnvironmentLightFromInputStream =
        LoadAndApplyEnvironmentLightFromInputStreamRequest.newBuilder().setIblKey(iblKey).build();
    return apiBridge.sendRequestAsync(
        new ApiRequest<LoadAndApplyEnvironmentLightFromInputStreamRequest>(
            "imp.scripting.LoadAndApplyEnvironmentLightFromInputStreamRequest",
            loadAndApplyEnvironmentLightFromInputStream,
            Arrays.asList(stream)));
  }

  /** Loads the given scene by isf asset string name (i.e. a native ISF file). */
  public ListenableFuture<Node> loadScene(String scene) {
    LoadSceneRequest loadScene =
        LoadSceneRequest.newBuilder().setScene(scene).setEnabled(true).build();
    ListenableFuture<NodeHandleMessage> future =
        apiBridge.sendRequestAsync(
            "imp.scripting.LoadSceneRequest", loadScene, NodeHandleMessage.class);
    return Futures.transform(
        future,
        (nodeHandleMessage) -> new Node(apiBridge, nodeHandleMessage),
        MoreExecutors.directExecutor());
  }

  /** Creates a new, empty node. */
  public Node createNode() {
    NodeHandleMessage nodeHandleMessage =
        apiBridge.sendRequest(
            "imp.scripting.CreateNodeRequest",
            CreateNodeRequest.getDefaultInstance(),
            NodeHandleMessage.class);
    return new Node(apiBridge, nodeHandleMessage);
  }

  /**
   * Gets the Node in the scene with the given name or location path.
   *
   * <p>The name should start with "//" and can reference nodes using the name field in an isf
   * scene, nodes with names set directly with node->SetName(name), or parts of glTF models that
   * have names defined in the asset.
   */
  public Node findNode(String locationPath) {
    NodeHandleMessage nodeHandleMessage =
        apiBridge.sendRequest(
            "imp.scripting.FindNodeRequest",
            FindNodeRequest.newBuilder().setLocationPath(locationPath).build(),
            NodeHandleMessage.class);
    return new Node(apiBridge, nodeHandleMessage);
  }

  /** Gets the Node associated with the native Impress camera. */
  public Node getCamera() {
    NodeHandleMessage nodeHandleMessage =
        apiBridge.sendRequest(
            "imp.scripting.GetCameraRequest",
            GetCameraRequest.getDefaultInstance(),
            NodeHandleMessage.class);
    return new Node(apiBridge, nodeHandleMessage);
  }

  /** Sets the projection matrix of the main Impress camera including near and far clip. */
  public void setProjection(Mat4f matrix, float near, float far) {
    apiBridge.sendRequest(
        "imp.scripting.SetCameraProjectionRequest",
        SetCameraProjectionRequest.newBuilder()
            .setMatrix(matrix)
            .setNear(near)
            .setFar(far)
            .build());
  }

  /** Sets the projection matrix of the main Impress camera. */
  public void setProjection(Mat4f matrix) {
    apiBridge.sendRequest(
        "imp.scripting.SetCameraProjectionRequest",
        SetCameraProjectionRequest.newBuilder().setMatrix(matrix).build());
  }

  /**
   * Adds a listener to the given event type (url and class). Returns an EventListenerHandle - call
   * release() on the handle to stop listening.
   */
  public <EventT extends MessageLite> EventListenerHandle addEventListener(
      String eventTypeUrl, Class<EventT> eventType, Consumer<EventT> listener) {
    return apiBridge.addEventListener(eventTypeUrl, eventType, null, listener);
  }

  /** Sends an event to native of the given type. */
  public <EventT extends MessageLite> void sendEvent(String eventTypeUrl, EventT event) {
    apiBridge.sendRequest(eventTypeUrl, event);
  }

  /** Overrides the default event dispatcher with a custom one. */
  public void setEventDispatcher(EventDispatcher customEventDispatcher) {
    apiBridge.setEventDispatcher(customEventDispatcher);
  }

  public void removeEventDispatcher() {
    apiBridge.removeEventDispatcher();
  }
}
