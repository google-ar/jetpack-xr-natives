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

package com.google.ar.imp.core.input;

import android.content.Context;
import android.view.KeyEvent;
import android.view.View;

/** This is a customized view to handle keyboard events. */
public class KeyboardView extends View {

  public KeyboardView(Context context, long viewHandle) {
    super(context);
    this.viewHandle = viewHandle;
  }

  @Override
  public boolean onKeyDown(int keyCode, KeyEvent event) {

    nProcessKeyboardEvent(
        viewHandle,
        event.getUnicodeChar(event.getModifiers()),
        keyCode,
        event.getAction(),
        event.getModifiers());

    return true;
  }

  @Override
  public boolean onKeyUp(int keyCode, KeyEvent event) {

    nProcessKeyboardEvent(
        viewHandle,
        event.getUnicodeChar(event.getModifiers()),
        keyCode,
        event.getAction(),
        event.getModifiers());

    return true;
  }

  private final long viewHandle;

  protected static native void nProcessKeyboardEvent(
      long homeViewHandle, int charCode, int keyCode, int action, int modifiers);
}
