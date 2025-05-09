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

import static com.android.extensions.xr.node.InputEvent.ACTION_DOWN;
import static com.android.extensions.xr.node.InputEvent.ACTION_MOVE;

import androidx.annotation.NonNull;
import com.android.extensions.xr.node.InputEvent;
import com.android.extensions.xr.node.InputEvent.HitInfo;
import com.android.extensions.xr.node.Vec3;

/** Wrapper around incoming XROS InputEvent. */
public final class SplitEngineInputEvent {
  // Reference to the native version of this object. A value of 0 means the native version has been
  // freed.
  private long nativeHandle;

  public static Boolean isInputEventValidForSubspace(@NonNull InputEvent event) {
    return event.getDispatchFlags() > 0
        || event.getHitInfo() != null
        || event.getSecondaryHitInfo() != null;
  }

  @NonNull
  public static SplitEngineInputEvent createFromInputEvent(@NonNull InputEvent event) {
    SplitEngineInputEvent out = new SplitEngineInputEvent();
    out.nativeHandle = nCreateSplitEngineInputEvent();
    long nHandle = out.nativeHandle;

    nSplitEngineInputEventSetDispatchFlags(nHandle, event.getDispatchFlags());
    nSplitEngineInputEventSetDeviceType(nHandle, event.getSource());
    nSplitEngineInputEventSetPointerType(nHandle, event.getPointerType());
    nSplitEngineInputEventSetTimestampMs(nHandle, event.getTimestamp());

    Vec3 origin = event.getOrigin();
    Vec3 direction = event.getDirection();
    nSplitEngineInputEventSetOrigin(nHandle, origin.x, origin.y, origin.z);
    nSplitEngineInputEventSetDirection(nHandle, direction.x, direction.y, direction.z);
    nSplitEngineInputEventSetButtonState(nHandle, getButtonState(event.getAction()));
    nSplitEngineInputEventSetAction(nHandle, event.getAction());

    setHitNode(nHandle, event.getHitInfo());
    setSecondaryHitNode(nHandle, event.getSecondaryHitInfo());

    return out;
  }

  public void destroyNativeEvent() {
    if (nativeHandle == 0) {
      return;
    }
    nDestroySplitEngineInputEvent(nativeHandle);
    nativeHandle = 0;
  }

  public long getNativeHandle() {
    return nativeHandle;
  }

  private SplitEngineInputEvent() {}

  private static int getButtonState(int action) {
    return (action == ACTION_DOWN || action == ACTION_MOVE) ? 1 : 0;
  }

  private static void setHitNode(long nHandle, HitInfo hitInfo) {
    if (hitInfo == null) {
      return;
    }
    int nodeId = hitInfo.getSubspaceImpressNodeId();
    Vec3 hitPosition = hitInfo.getHitPosition();
    boolean hitPositionIsValid = hitPosition != null;
    float[] transform = hitInfo.getTransform().getFlattenedMatrix();

    nSplitEngineInputEventCreateHitNode(nHandle);
    nSplitEngineInputEventSetHitNodeNodeId(nHandle, nodeId);
    nSplitEngineInputEventSetHitNodeHitPositionIsValid(nHandle, hitPositionIsValid);
    if (hitPositionIsValid) {
      nSplitEngineInputEventSetHitNodeHitPosition(
          nHandle, hitPosition.x, hitPosition.y, hitPosition.z);
    } else {
      nSplitEngineInputEventSetHitNodeHitPosition(nHandle, 0f, 0f, 0f);
    }
    nSplitEngineInputEventSetHitNodeTransform(
        nHandle,
        transform[0],
        transform[1],
        transform[2],
        transform[3],
        transform[4],
        transform[5],
        transform[6],
        transform[7],
        transform[8],
        transform[9],
        transform[10],
        transform[11],
        transform[12],
        transform[13],
        transform[14],
        transform[15]);
  }

  private static void setSecondaryHitNode(long nHandle, HitInfo hitInfo) {
    if (hitInfo == null) {
      return;
    }
    int nodeId = hitInfo.getSubspaceImpressNodeId();
    Vec3 hitPosition = hitInfo.getHitPosition();
    boolean hitPositionIsValid = hitPosition != null;
    float[] transform = hitInfo.getTransform().getFlattenedMatrix();

    nSplitEngineInputEventCreateSecondaryHitNode(nHandle);
    nSplitEngineInputEventSetSecondaryHitNodeNodeId(nHandle, nodeId);
    nSplitEngineInputEventSetSecondaryHitNodeHitPositionIsValid(nHandle, hitPositionIsValid);
    if (hitPositionIsValid) {
      nSplitEngineInputEventSetSecondaryHitNodeHitPosition(
          nHandle, hitPosition.x, hitPosition.y, hitPosition.z);
    } else {
      nSplitEngineInputEventSetSecondaryHitNodeHitPosition(nHandle, 0f, 0f, 0f);
    }
    nSplitEngineInputEventSetSecondaryHitNodeTransform(
        nHandle,
        transform[0],
        transform[1],
        transform[2],
        transform[3],
        transform[4],
        transform[5],
        transform[6],
        transform[7],
        transform[8],
        transform[9],
        transform[10],
        transform[11],
        transform[12],
        transform[13],
        transform[14],
        transform[15]);
  }

  private static native long nCreateSplitEngineInputEvent();

  private static native void nDestroySplitEngineInputEvent(long eventHandle);

  private static native void nSplitEngineInputEventSetDispatchFlags(
      long eventHandle, int dispatchFlags);

  private static native void nSplitEngineInputEventSetDeviceType(long eventHandle, int deviceType);

  private static native void nSplitEngineInputEventSetPointerType(
      long eventHandle, int pointerType);

  private static native void nSplitEngineInputEventSetTimestampMs(
      long eventHandle, long timestampMs);

  private static native void nSplitEngineInputEventSetOrigin(
      long eventHandle, float x, float y, float z);

  private static native void nSplitEngineInputEventSetDirection(
      long eventHandle, float x, float y, float z);

  private static native void nSplitEngineInputEventSetButtonState(
      long eventHandle, int buttonState);

  private static native void nSplitEngineInputEventSetAction(long eventHandle, int action);

  private static native void nSplitEngineInputEventCreateHitNode(long eventHandle);

  private static native void nSplitEngineInputEventSetHitNodeNodeId(long eventHandle, int nodeId);

  private static native void nSplitEngineInputEventSetHitNodeHitPositionIsValid(
      long eventHandle, boolean isValid);

  private static native void nSplitEngineInputEventSetHitNodeHitPosition(
      long eventHandle, float x, float y, float z);

  private static native void nSplitEngineInputEventSetHitNodeTransform(
      long eventHandle,
      float m00,
      float m01,
      float m02,
      float m03,
      float m10,
      float m11,
      float m12,
      float m13,
      float m20,
      float m21,
      float m22,
      float m23,
      float m30,
      float m31,
      float m32,
      float m33);

  private static native void nSplitEngineInputEventCreateSecondaryHitNode(long eventHandle);

  private static native void nSplitEngineInputEventSetSecondaryHitNodeNodeId(
      long eventHandle, int nodeId);

  private static native void nSplitEngineInputEventSetSecondaryHitNodeHitPositionIsValid(
      long eventHandle, boolean isValid);

  private static native void nSplitEngineInputEventSetSecondaryHitNodeHitPosition(
      long eventHandle, float x, float y, float z);

  private static native void nSplitEngineInputEventSetSecondaryHitNodeTransform(
      long eventHandle,
      float m00,
      float m01,
      float m02,
      float m03,
      float m10,
      float m11,
      float m12,
      float m13,
      float m20,
      float m21,
      float m22,
      float m23,
      float m30,
      float m31,
      float m32,
      float m33);
}
