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

import android.app.Activity;
import android.content.Context;
import android.content.ContextWrapper;
import android.view.ViewGroup;
import android.view.inputmethod.InputMethodManager;
import com.google.android.filament.proguard.UsedByNative;

/** Keyboard turn on/off class */
@UsedByNative("android_keyboard_controller.h")
class ImpKeyboardController {

  @UsedByNative("android_keyboard_controller.h")
  public ImpKeyboardController(Context context, long homeViewHandle) {
    this.context = context;
    keyboardView = new KeyboardView(context, homeViewHandle);
    // Need to keep the keyboard view at least 1 pixel.
    // Otherwise, its onKeyDown/Up method may not be called when keys are pressed.
    ViewGroup.LayoutParams layoutParams = new ViewGroup.LayoutParams(1, 1);
    keyboardView.setLayoutParams(layoutParams);
    keyboardView.setFocusable(true);
    keyboardView.setFocusableInTouchMode(true);
  }

  @UsedByNative("android_keyboard_controller.h")
  public void openKeyboard() {
    if (!hasKeyboard) {
      Activity activity = getActivity(context);
      ViewGroup viewgroup = (ViewGroup) activity.findViewById(android.R.id.content);
      viewgroup.addView(keyboardView);
      hasKeyboard = true;
    }
    keyboardView.requestFocus();

    InputMethodManager imm =
        (InputMethodManager) context.getSystemService(Context.INPUT_METHOD_SERVICE);

    imm.showSoftInput(keyboardView, InputMethodManager.SHOW_IMPLICIT);
  }

  @UsedByNative("android_input_keyboard.h")
  public void closeKeyboard() {
    InputMethodManager imm =
        (InputMethodManager) context.getSystemService(Context.INPUT_METHOD_SERVICE);
    imm.hideSoftInputFromWindow(keyboardView.getWindowToken(), 0);
    keyboardView.clearFocus();
  }

  private static Activity getActivity(Context context) {
    if (context instanceof Activity) {
      return (Activity) context;
    } else {
      return getActivity(((ContextWrapper) context).getBaseContext());
    }
  }

  private final KeyboardView keyboardView;
  private boolean hasKeyboard = false;
  private final Context context;
}
