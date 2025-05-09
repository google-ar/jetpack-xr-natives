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

import android.view.Choreographer;

/** A wrapper for registering and unregistering frame callbacks with the Choreographer. */
interface ChoreographerHolder {
  /** Registers a frame callback to the Choreographer. */
  void postFrameCallback(Choreographer.FrameCallback callback);

  /** Removes the frame callback from the Choreographer. */
  void removeFrameCallback(Choreographer.FrameCallback callback);
}
