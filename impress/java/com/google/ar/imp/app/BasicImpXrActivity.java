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

package com.google.ar.imp.app;

import android.os.Bundle;
import androidx.appcompat.app.AppCompatActivity;
import com.google.ar.imp.view.xr.ImpXrRenderer;

/** Example activity for using Impress with OpenXR */
public class BasicImpXrActivity extends AppCompatActivity {

  protected ImpXrRenderer renderer;

  @Override
  public void onResume() {
    super.onResume();
    renderer.startFrameLoop();
  }

  @Override
  public void onPause() {
    super.onPause();
    renderer.stopFrameLoop();
  }

  @Override
  public void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    renderer = ImpXrRenderer.create(this, /* setupParams= */ null);
  }

  @Override
  public void onAttachedToWindow() {
    renderer.onWindowAttached();
  }

  @Override
  public void onDestroy() {
    super.onDestroy();
    renderer.destroy();
  }
}
