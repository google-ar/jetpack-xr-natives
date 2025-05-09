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

package com.google.ar.imp.view.input;

import android.view.MotionEvent;
import java.util.HashMap;
import java.util.Map;

/** Manager for input handling. */
public class InputManager {

  private static final Map<Integer, Integer> actionToPointerEventIntMap =
      actionToPointerEventIntMap();

  private static Map<Integer, Integer> actionToPointerEventIntMap() {
    Map<Integer, Integer> actionToPointerEventIntMap = new HashMap<>();
    // LINT.IfChange
    // imp::PointerEventType::kCancel
    actionToPointerEventIntMap.put(MotionEvent.ACTION_CANCEL, 0);
    // imp::PointerEventType::kDown
    actionToPointerEventIntMap.put(MotionEvent.ACTION_DOWN, 1);
    actionToPointerEventIntMap.put(MotionEvent.ACTION_POINTER_DOWN, 1);
    // imp::PointerEventType::kUp
    actionToPointerEventIntMap.put(MotionEvent.ACTION_UP, 2);
    actionToPointerEventIntMap.put(MotionEvent.ACTION_POINTER_UP, 2);
    // imp::PointerEventType::kMove
    actionToPointerEventIntMap.put(MotionEvent.ACTION_MOVE, 3);
    // LINT.ThenChange(
    //   //depot/google3/third_party/impress/core/input/pointer_event.h
    // )
    return actionToPointerEventIntMap;
  }

  private final long viewHostHandle;

  public InputManager(long viewHostHandle) {
    this.viewHostHandle = viewHostHandle;
  }

  public boolean onTouchEvent(MotionEvent e) {
    int action = e.getActionMasked();
    if (!actionToPointerEventIntMap.containsKey(action)) {
      return false;
    }

    // During a move event, include all pointers from the Android MotionEvent. During
    // Up/Down/Cancel pointer events, only include the active pointer details in the event.
    int count = action == MotionEvent.ACTION_MOVE ? e.getPointerCount() : 1;
    int[] idList = new int[count];
    float[] xList = new float[count];
    float[] yList = new float[count];
    for (int listIndex = 0; listIndex < count; ++listIndex) {
      int eventIndex = action == MotionEvent.ACTION_MOVE ? listIndex : e.getActionIndex();
      idList[listIndex] = e.getPointerId(eventIndex);
      xList[listIndex] = e.getX(eventIndex);
      yList[listIndex] = e.getY(eventIndex);
    }

    Integer pointerEventType = actionToPointerEventIntMap.get(action);
    nProcessPointerEvent(viewHostHandle, pointerEventType, idList, xList, yList, e.getEventTime());
    return true;
  }

  // LINT.IfChange(api)
  protected static native void nProcessPointerEvent(
      long viewHostHandle, int action, int[] id, float[] x, float[] y, long timestamp);
  // LINT.ThenChange(
  //     //depot/google3/third_party/impress/core/view/platforms/android/view_jni.cc:api
  // )
}
