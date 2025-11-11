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

import android.content.pm.ActivityInfo;
import android.content.pm.PackageManager;
import android.os.Bundle;
import androidx.appcompat.app.AppCompatActivity;
import android.util.Log;
import com.google.ar.imp.view.ImpApi;
import com.google.ar.imp.view.ImpSurfaceView;
import com.google.ar.imp.view.SetupParams;

/** Example activity for using Impress */
public class BasicImpActivity extends AppCompatActivity {
  private static final String TAG = BasicImpActivity.class.getSimpleName();

  public static final String IS_UNDER_TEST_KEY = "IS_UNDER_TEST";

  // If using a custom jni library, then he name of the .so file passed through the AndroidManifest.
  public static final String IMPRESS_BINARY_NAME = "IMPRESS_BINARY_NAME";

  protected ImpSurfaceView impView;
  protected ImpApi impApi;
  private boolean isUnderTest = false;

  @Override
  public void onResume() {
    super.onResume();
    if (impApi != null) {
      if (!isUnderTest) {
        impApi.startFrameLoop();
      } else {
        // In end to end tests, we don't want to start the frame loop so that the test can
        // explicitly control when time advances and frames are rendered. This helps us avoid
        // flakiness in the tests. In that case, we call resume explicitly to ensure that the normal
        // lifecycle events are still triggered.
        impApi.resume();
      }
    }
  }

  @Override
  public void onPause() {
    super.onPause();
    if (impApi != null) {
      if (!isUnderTest) {
        impApi.stopFrameLoop();
      } else {
        impApi.pause();
      }
    }
  }

  @Override
  public void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);

    Bundle extras = getIntent().getExtras();
    if (extras != null) {
      isUnderTest = extras.getBoolean(IS_UNDER_TEST_KEY, false);
    }

    impView = new ImpSurfaceView(this);
    impApi = impView.createImpApiSync(getSetupParams());
    setContentView(impView);
  }

  @Override
  public void onDestroy() {
    super.onDestroy();
    if (impApi != null) {
      impApi.releaseResources();
    }
  }

  public ImpSurfaceView getImpView() {
    return impView;
  }

  private SetupParams getSetupParams() {
    String binaryName = "";

    try {
      ActivityInfo ai =
          getPackageManager().getActivityInfo(getComponentName(), PackageManager.GET_META_DATA);
      Bundle bundle = ai.metaData;
      if (bundle != null) {
        binaryName = bundle.getString(IMPRESS_BINARY_NAME);
      }
    } catch (PackageManager.NameNotFoundException e) {
      Log.e(
          this.getClass().getSimpleName(),
          "Failed to load meta-data, NameNotFound: " + e.getMessage());
    } catch (NullPointerException e) {
      Log.e(TAG, "Failed to load meta-data, NullPointer: " + e.getMessage());
    }

    if (!binaryName.isEmpty()) {
      return SetupParams.newBuilder().setCustomNativeLibrary(binaryName).build();
    } else {
      return SetupParams.getDefaultInstance();
    }
  }
}
