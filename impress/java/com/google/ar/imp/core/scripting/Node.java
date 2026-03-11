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

import androidx.annotation.Nullable;
import com.google.ar.imp.core.NodeHandleMessageOuterClass.NodeHandleMessage;
import com.google.ar.imp.core.scripting.Api.AddComponentRequest;
import com.google.ar.imp.core.scripting.Api.ComponentData;
import com.google.ar.imp.core.scripting.Api.DestroyNodeRequest;
import com.google.ar.imp.core.scripting.Api.GetChildrenRequest;
import com.google.ar.imp.core.scripting.Api.GetNodeActiveRequest;
import com.google.ar.imp.core.scripting.Api.GetNodeActiveResponse;
import com.google.ar.imp.core.scripting.Api.GetNodeNameRequest;
import com.google.ar.imp.core.scripting.Api.GetNodeNameResponse;
import com.google.ar.imp.core.scripting.Api.GetTransformMatrixRequest;
import com.google.ar.imp.core.scripting.Api.GetTransformMatrixResponse;
import com.google.ar.imp.core.scripting.Api.GetTransformRequest;
import com.google.ar.imp.core.scripting.Api.GetTransformResponse;
import com.google.ar.imp.core.scripting.Api.NodeListResponse;
import com.google.ar.imp.core.scripting.Api.PlayAnimationRequest;
import com.google.ar.imp.core.scripting.Api.SetNodeEnabledRequest;
import com.google.ar.imp.core.scripting.Api.SetNodeNameRequest;
import com.google.ar.imp.core.scripting.Api.SetParentRequest;
import com.google.ar.imp.core.scripting.Api.SetTransformMatrixRequest;
import com.google.ar.imp.core.scripting.Api.SetTransformRequest;
import com.google.ar.imp.core.scripting.Api.TransformType;
import com.google.ar.imp.core.scripting.Api.UpdateComponentRequest;
import com.google.ar.imp.core.scripting.Events.NodeEvent;
import com.google.ar.imp.proto.GltfAnimatorState.AnimOptions;
import com.google.ar.imp.proto.GltfAnimatorState.PlayCommand;
import com.google.ar.imp.proto.Math.Float3;
import com.google.ar.imp.proto.Math.Mat4f;
import com.google.ar.imp.proto.Math.Quatf;
import com.google.common.collect.Lists;
import com.google.protobuf.Any;
import com.google.protobuf.MessageLite;
import java.util.List;
import java.util.function.Consumer;

/** A Wrapper for a native Impress NodeHandle that can be used in the scripting API. */
public class Node {
  private static final String GET_TRANSFORM_REQUEST_TYPE_URL = "imp.scripting.GetTransformRequest";
  private static final String GET_TRANSFORM_MATRIX_REQUEST_TYPE_URL =
      "imp.scripting.GetTransformMatrixRequest";
  private static final String SET_TRANSFORM_REQUEST_TYPE_URL = "imp.scripting.SetTransformRequest";
  private static final String SET_TRANSFORM_MATRIX_REQUEST_TYPE_URL =
      "imp.scripting.SetTransformMatrixRequest";

  private enum TransformComponent {
    TRANSLATION,
    ROTATION,
    SCALE;
  }

  private final ApiBridge apiBridge;
  private final NodeHandleMessage nodeHandleMessage;

  Node(ApiBridge apiBridge, NodeHandleMessage nodeHandleMessage) {
    this.apiBridge = apiBridge;
    this.nodeHandleMessage = nodeHandleMessage;
  }

  /** Gets the NodeHandleMessage protobuf representation of this node for sending to native. */
  public NodeHandleMessage getNodeHandle() {
    return nodeHandleMessage;
  }

  /** Destroys this node and all its children. */
  public void destroy() {
    DestroyNodeRequest destroyNode =
        DestroyNodeRequest.newBuilder().setTarget(getNodeHandle()).build();
    apiBridge.sendRequest("imp.scripting.DestroyNodeRequest", destroyNode);
  }

  /** Enables or disables this node (when disabled, a node does not render/update). */
  public void setEnabled(boolean enabled) {
    SetNodeEnabledRequest setNodeEnabledRequest =
        SetNodeEnabledRequest.newBuilder().setTarget(getNodeHandle()).setEnabled(enabled).build();
    apiBridge.sendRequest("imp.scripting.SetNodeEnabledRequest", setNodeEnabledRequest);
  }

  /**
   * Returns if this node is active. A node is active if it's enabled and its parent is active. A
   * node is only rendered if it is active.
   */
  public boolean isActive() {
    GetNodeActiveRequest getNodeActiveRequest =
        GetNodeActiveRequest.newBuilder().setTarget(getNodeHandle()).build();
    return apiBridge
        .sendRequest(
            "imp.scripting.GetNodeActiveRequest", getNodeActiveRequest, GetNodeActiveResponse.class)
        .getActive();
  }

  public void setParent(@Nullable Node parent) {
    SetParentRequest.Builder setParentRequestBuilder = SetParentRequest.newBuilder();
    setParentRequestBuilder.setTarget(getNodeHandle());
    if (parent != null) {
      setParentRequestBuilder.setParent(parent.getNodeHandle());
    }
    apiBridge.sendRequest("imp.scripting.SetParentRequest", setParentRequestBuilder.build());
  }

  /** Returns the children of this node. */
  public List<Node> getChildren() {
    GetChildrenRequest getChildrenRequest =
        GetChildrenRequest.newBuilder().setTarget(getNodeHandle()).build();
    return Lists.transform(
        apiBridge
            .sendRequest(
                "imp.scripting.GetChildrenRequest", getChildrenRequest, NodeListResponse.class)
            .getNodesList(),
        (child) -> new Node(apiBridge, child));
  }

  /** Sets the name of the node. */
  public void setName(String name) {
    SetNodeNameRequest setNodeNameRequest =
        SetNodeNameRequest.newBuilder().setTarget(getNodeHandle()).setName(name).build();
    apiBridge.sendRequest("imp.scripting.SetNodeNameRequest", setNodeNameRequest);
  }

  /** Returns the name of the node, or the empty string if unnamed. */
  public String getName() {
    GetNodeNameRequest getNodeNameRequest =
        GetNodeNameRequest.newBuilder().setTarget(getNodeHandle()).build();
    return apiBridge
        .sendRequest(
            "imp.scripting.GetNodeNameRequest", getNodeNameRequest, GetNodeNameResponse.class)
        .getName();
  }

  public <ComponentDataT extends MessageLite> void addComponent(
      String typeUrl, ComponentDataT data) {
    Any dataAny = MessageUtils.toAny(typeUrl, data);

    AddComponentRequest addOrUpdateComponentRequest =
        AddComponentRequest.newBuilder()
            .setTarget(getNodeHandle())
            .setComponentData(ComponentData.newBuilder().setAnyData(dataAny).build())
            .build();
    apiBridge.sendRequest("imp.scripting.AddComponentRequest", addOrUpdateComponentRequest);
  }

  public <ComponentDataT extends MessageLite> void updateComponent(
      String typeUrl, ComponentDataT data) {
    Any dataAny = MessageUtils.toAny(typeUrl, data);

    UpdateComponentRequest updateComponentRequest =
        UpdateComponentRequest.newBuilder()
            .setTarget(getNodeHandle())
            .setComponentData(ComponentData.newBuilder().setAnyData(dataAny).build())
            .build();
    apiBridge.sendRequest("imp.scripting.UpdateComponentRequest", updateComponentRequest);
  }

  public void updateComponent(String textprotoData) {
    UpdateComponentRequest updateComponentRequest =
        UpdateComponentRequest.newBuilder()
            .setTarget(getNodeHandle())
            .setComponentData(ComponentData.newBuilder().setTextprotoData(textprotoData).build())
            .build();
    apiBridge.sendRequest("imp.scripting.UpdateComponentRequest", updateComponentRequest);
  }

  /**
   * Adds a listener to the given event type (url and class) originating from this node. Returns an
   * EventListenerHandle - call release() on the handle to stop listening.
   */
  public <EventT extends MessageLite> EventListenerHandle addEventListener(
      String eventTypeUrl, Class<EventT> eventType, Consumer<EventT> listener) {
    return apiBridge.addEventListener(eventTypeUrl, eventType, getNodeHandle(), listener);
  }

  /** Sends an event to native of the given type targeting this node. */
  public <EventT extends MessageLite> void sendEvent(String eventTypeUrl, EventT event) {
    NodeEvent nodeEvent =
        NodeEvent.newBuilder()
            .setTarget(getNodeHandle())
            .setEvent(MessageUtils.toAny(eventTypeUrl, event))
            .build();
    apiBridge.sendRequest("imp.scripting.NodeEvent", nodeEvent);
  }

  /** Gets the position of this node relative to its parent. */
  public Float3 getLocalPosition() {
    return getLocalTransform(TransformComponent.TRANSLATION).getTranslation();
  }

  /** Gets the position of this node in world space. */
  public Float3 getWorldPosition() {
    return getWorldTransform(TransformComponent.TRANSLATION).getTranslation();
  }

  /** Sets the position of this node relative to its parent to the given value. */
  public void setLocalPosition(Float3 position) {
    setLocalTransform(position, null, null);
  }

  /** Sets the position of this node in world space to the given value. */
  public void setWorldPosition(Float3 position) {
    setWorldTransform(position, null, null);
  }

  /** Gets the rotation of this node relative to its parent. */
  public Quatf getLocalRotation() {
    return getLocalTransform(TransformComponent.ROTATION).getRotation();
  }

  /** Gets the rotation of this node in world space. */
  public Quatf getWorldRotation() {
    return getWorldTransform(TransformComponent.ROTATION).getRotation();
  }

  /** Sets the rotation of this node relative to its parent to the given value. */
  public void setLocalRotation(Quatf rotation) {
    setLocalTransform(null, rotation, null);
  }

  /** Sets the rotation of this node in world space to the given value. */
  public void setWorldRotation(Quatf rotation) {
    setWorldTransform(null, rotation, null);
  }

  /** Gets the scale of this node relative to its parent. */
  public Float3 getLocalScale() {
    return getLocalTransform(TransformComponent.SCALE).getScale();
  }

  /** Gets the scale of this node in world space. */
  public Float3 getWorldScale() {
    return getWorldTransform(TransformComponent.SCALE).getScale();
  }

  /** Sets the scale of this node relative to its parent to the given value. */
  public void setLocalScale(Float3 scale) {
    setLocalTransform(null, null, scale);
  }

  /** Sets the scale of this node in world space to the given value. */
  public void setWorldScale(Float3 scale) {
    setWorldTransform(null, null, scale);
  }

  /** Gets the full affine transformation of this node relative to its parent. */
  public Mat4f getLocalTransform() {
    return getTransform(TransformType.TRANSFORM_TYPE_LOCAL);
  }

  /** Gets the transform of this node relative to its parent. */
  private GetTransformResponse getLocalTransform(TransformComponent component) {
    return getTransform(component, TransformType.TRANSFORM_TYPE_LOCAL);
  }

  /** Gets the full affine transformation of this node in world space. */
  public Mat4f getWorldTransform() {
    return getTransform(TransformType.TRANSFORM_TYPE_WORLD);
  }

  /** Gets the transform of this node in world space. */
  private GetTransformResponse getWorldTransform(TransformComponent component) {
    return getTransform(component, TransformType.TRANSFORM_TYPE_WORLD);
  }

  /**
   * Sets the TRS (translation/rotation/scale) of this node relative to its parent. Some components
   * can be null.
   */
  public void setLocalTransform(Float3 translation, Quatf rotation, Float3 scale) {
    setTransform(translation, rotation, scale, TransformType.TRANSFORM_TYPE_LOCAL);
  }

  /** Sets the local transform of this node to the given value. */
  public void setLocalTransform(Mat4f transform) {
    setTransform(transform, TransformType.TRANSFORM_TYPE_LOCAL);
  }

  /**
   * Sets the TRS (translation/rotation/scale) of this node in world space. Some components can be
   * null.
   */
  public void setWorldTransform(Float3 translation, Quatf rotation, Float3 scale) {
    setTransform(translation, rotation, scale, TransformType.TRANSFORM_TYPE_WORLD);
  }

  /** Sets the local transform of this node to the given value. */
  public void setWorldTransform(Mat4f transform) {
    setTransform(transform, TransformType.TRANSFORM_TYPE_WORLD);
  }

  /** Plays the animation with the given name on this node (must be a glTF node). */
  public void playGltfAnimation(String animationName, boolean looping) {
    AnimOptions options = AnimOptions.newBuilder().setLooping(looping).build();
    PlayCommand command =
        PlayCommand.newBuilder().setName(animationName).setOptions(options).build();
    playGltfAnimation(command);
  }

  /** Plays the animation with the given index on this node (must be a glTF node). */
  public void playGltfAnimation(int animationIndex, boolean looping) {
    AnimOptions options = AnimOptions.newBuilder().setLooping(looping).build();
    PlayCommand command =
        PlayCommand.newBuilder().setIndex(animationIndex).setOptions(options).build();
    playGltfAnimation(command);
  }

  /** Plays the first animation on this node (must be a glTF node). */
  public void playGltfAnimation(boolean looping) {
    playGltfAnimation(0, looping);
  }

  private void playGltfAnimation(PlayCommand command) {
    PlayAnimationRequest request =
        PlayAnimationRequest.newBuilder()
            .setTarget(getNodeHandle())
            .setPlayCommand(command)
            .build();
    apiBridge.sendRequest("imp.scripting.PlayAnimationRequest", request);
  }

  /**
   * Sets the TRS (translation/rotation/scale) of this node in the space given by transformType.
   * Some components can be null.
   */
  private void setTransform(
      Float3 translation, Quatf rotation, Float3 scale, TransformType transformType) {
    SetTransformRequest.Builder setTransformRequestBuilder = SetTransformRequest.newBuilder();
    setTransformRequestBuilder.setTarget(getNodeHandle());
    if (translation != null) {
      setTransformRequestBuilder.setTranslation(translation);
    }
    if (rotation != null) {
      setTransformRequestBuilder.setRotation(rotation);
    }
    if (scale != null) {
      setTransformRequestBuilder.setScale(scale);
    }
    setTransformRequestBuilder.setTransformType(transformType);
    apiBridge.sendRequest(SET_TRANSFORM_REQUEST_TYPE_URL, setTransformRequestBuilder.build());
  }

  /** Sets the transform of this node in the space given by transformType to the given value. */
  private void setTransform(Mat4f transform, TransformType transformType) {
    SetTransformMatrixRequest request =
        SetTransformMatrixRequest.newBuilder()
            .setTarget(getNodeHandle())
            .setTransform(transform)
            .setTransformType(transformType)
            .build();
    apiBridge.sendRequest(SET_TRANSFORM_MATRIX_REQUEST_TYPE_URL, request);
  }

  /** Gets one of the TRS components of this node in the space given by transformType. */
  private GetTransformResponse getTransform(
      TransformComponent component, TransformType transformType) {
    GetTransformRequest.Builder getTransformRequestBuilder = GetTransformRequest.newBuilder();
    getTransformRequestBuilder.setTarget(getNodeHandle());
    switch (component) {
      case TRANSLATION -> getTransformRequestBuilder.setTranslation(true);
      case ROTATION -> getTransformRequestBuilder.setRotation(true);
      case SCALE -> getTransformRequestBuilder.setScale(true);
    }
    getTransformRequestBuilder.setTransformType(transformType);
    return apiBridge.sendRequest(
        GET_TRANSFORM_REQUEST_TYPE_URL,
        getTransformRequestBuilder.build(),
        GetTransformResponse.class);
  }

  /** Gets the full affine transformation of this node in the space given by transformType. */
  private Mat4f getTransform(TransformType transformType) {
    GetTransformMatrixRequest request =
        GetTransformMatrixRequest.newBuilder()
            .setTarget(getNodeHandle())
            .setTransformType(transformType)
            .build();
    GetTransformMatrixResponse response =
        apiBridge.sendRequest(
            GET_TRANSFORM_MATRIX_REQUEST_TYPE_URL, request, GetTransformMatrixResponse.class);
    return response.getTransform();
  }
}
