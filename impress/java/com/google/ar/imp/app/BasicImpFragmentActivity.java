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
import androidx.fragment.app.FragmentActivity;
import android.view.Choreographer;
import com.google.ar.imp.core.web.FragmentHost;
import com.google.ar.imp.view.ImpApi;
import com.google.ar.imp.view.ImpSurfaceView;
import com.google.ar.imp.view.SetupParams;

/** Basic activity that extends FragmentActivity. */
public class BasicImpFragmentActivity extends FragmentActivity implements FragmentHost {
  private static final String TAG = BasicImpFragmentActivity.class.getSimpleName();

  private ImpSurfaceView impView;
  private ImpApi impApi;
  private Choreographer choreographer;

  /** Inner class to manage our update loop. */
  // TODO: move choreographer setup into native code
  protected class FrameCallback implements Choreographer.FrameCallback {
    @Override
    public void doFrame(long frameTimeNanos) {
      // Schedule the next frame
      choreographer.postFrameCallback(this);
      if (impApi != null) {
        impApi.doFrame(frameTimeNanos);
      }
    }
  }

  private final FrameCallback frameScheduler = new FrameCallback();

  @Override
  public void onResume() {
    super.onResume();
    choreographer.postFrameCallback(frameScheduler);
  }

  @Override
  public void onPause() {
    super.onPause();
    choreographer.removeFrameCallback(frameScheduler);
  }

  @Override
  public void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    impView = new ImpSurfaceView(this, this);
    impApi = impView.createImpApiSync(SetupParams.getDefaultInstance());
    setContentView(impView);
    choreographer = Choreographer.getInstance();
  }

  @Override
  public void onDestroy() {
    super.onDestroy();
    if (impApi != null) {
      impApi.releaseResources();
    }
  }
}
