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

package com.google.ar.imp.view;

import android.view.MotionEvent;
import java.util.function.Function;

/** An Interface for handling touch and hover events in Impress views. */
public interface InputEventHandler {

  /**
   * Listen to touch events on Impress views.
   *
   * @param event The touch event to handle.
   * @param defaultOnTouchEventHandler The default implementation of the onTouchEvent method on the
   *     Impress view.
   * @return True if the event was handled, false otherwise.
   */
  boolean onTouchEvent(
      MotionEvent event, Function<MotionEvent, Boolean> defaultOnTouchEventHandler);

  /**
   * Listen to hover events on Impress views.
   *
   * @param event The hover event to handle.
   * @param defaultOnHoverEventHandler The default implementation of the onHoverEvent method on the
   *     Impress view.
   * @return True if the event was handled, false otherwise.
   */
  boolean onHoverEvent(
      MotionEvent event, Function<MotionEvent, Boolean> defaultOnHoverEventHandler);
}
