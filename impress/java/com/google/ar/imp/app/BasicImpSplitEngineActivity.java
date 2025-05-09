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
import com.google.ar.imp.view.ImpApiScubaProxyTestView;
import com.google.ar.imp.view.splitengine.ImpSplitEngineRenderer;
import com.google.vr.realitycore.runtime.androidxr.extensions.XrExtensionsProvider;

/** Example activity for using Impress with Split Engine mode. */
public class BasicImpSplitEngineActivity extends AppCompatActivity {
  private static final String TAG = BasicImpSplitEngineActivity.class.getSimpleName();
  public static final String IS_UNDER_TEST_KEY = "IS_UNDER_TEST";

  protected ImpSplitEngineRenderer splitEngineRenderer;
  private ImpApiScubaProxyTestView impView;
  private boolean isUnderTest = false;

  @Override
  public void onResume() {
    super.onResume();
    if (!isUnderTest) {
      splitEngineRenderer.startFrameLoop();
    }
  }

  @Override
  public void onPause() {
    super.onPause();

    splitEngineRenderer.stopFrameLoop();
  }

  @Override
  public void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);

    Bundle extras = getIntent().getExtras();
    if (extras != null) {
      isUnderTest = extras.getBoolean(IS_UNDER_TEST_KEY, false);
    }

    splitEngineRenderer =
        ImpSplitEngineRenderer.create(
            this, /* setupParams= */ null, XrExtensionsProvider.getXrExtensions());
    if (isUnderTest) {
      // NOTE: DO NOT COPY THIS CODE BLOCK FOR YOUR OWN APPS. This is a hack to support Scuba tests.
      // In order to support Scuba tests, create a placeholder view that can be used by scuba to get
      // the ImpApiScuba instance.
      impView = new ImpApiScubaProxyTestView(this);
      impView.setImpApiScuba(splitEngineRenderer);
      setContentView(impView);
      impView.requestFocus();
    }
  }

  @Override
  public void onDestroy() {
    super.onDestroy();
    splitEngineRenderer.destroy();
    splitEngineRenderer = null;
  }
}
