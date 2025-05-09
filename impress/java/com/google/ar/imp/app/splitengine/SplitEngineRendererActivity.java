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

package com.google.ar.imp.app.splitengine;

import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import com.google.ar.imp.app.BasicImpActivity;

/** Activity for an app that handles intents to render content with the Impress Split Engine. */
public class SplitEngineRendererActivity extends BasicImpActivity {
  private static final String TAG = SplitEngineRendererActivity.class.getSimpleName();

  @Override
  public void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    Log.d(TAG, "Renderer app will start the SplitEngineSharedMemoryBridgeService service.");

    long nativeView = getImpView().getImpApi().getNativeHandle();
    long executor = getImpView().getImpApi().getView().getForegroundExecutor();

    Intent intent = new Intent();
    intent.setClassName(
        "com.google.ar.imp.app.splitengine",
        "com.google.ar.imp.app.splitengine.SplitEngineSharedMemoryBridgeService");
    intent.putExtra("impView", nativeView);
    intent.putExtra("executor", executor);
    startService(intent);
  }
}
