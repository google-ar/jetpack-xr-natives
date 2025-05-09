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

package com.google.ar.imp.core.web.testing;

import static java.util.concurrent.TimeUnit.SECONDS;

import android.app.Activity;
import android.webkit.ConsoleMessage;
import android.webkit.WebChromeClient;
import android.webkit.WebView;
import com.google.android.filament.proguard.UsedByNative;
import com.google.common.util.concurrent.SettableFuture;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeoutException;

/** Creates test webview for an E2E test of the Impress scripting system. */
public final class TestWebView {
  private static final int INIT_WEBVIEW_TIMEOUT_SECONDS = 30;
  private static final int TEST_TIMEOUT_SECONDS = 200;
  private static final String SCRIPT_PREFIX = "(() => {";
  private static final String SCRIPT_SUFFIX = "})()";

  private long nativeTestHandle;
  private WebView webView;
  private Activity activity;
  private SettableFuture<Boolean> completionFuture;

  @UsedByNative("web_view_test_wrapper.cc")
  public WebView init(Activity activity, long nativeTestHandle) {
    this.activity = activity;
    this.nativeTestHandle = nativeTestHandle;
    SettableFuture<WebView> webViewFuture = SettableFuture.create();
    activity.runOnUiThread(
        () -> {
          webViewFuture.set(initWebView(activity));
        });

    try {
      this.webView = webViewFuture.get(INIT_WEBVIEW_TIMEOUT_SECONDS, SECONDS);
    } catch (InterruptedException | ExecutionException | TimeoutException e) {
      throw new IllegalStateException(e);
    }
    return this.webView;
  }

  @UsedByNative("web_view_test_wrapper.cc")
  public void evaluateJavaScript(String script) {
    this.activity.runOnUiThread(
        () -> this.webView.evaluateJavascript(SCRIPT_PREFIX + script + SCRIPT_SUFFIX, null));
  }

  @UsedByNative("web_view_test_wrapper.cc")
  public boolean runAllTests(String script) {
    completionFuture = SettableFuture.create();
    this.evaluateJavaScript(script);
    try {
      completionFuture.get(TEST_TIMEOUT_SECONDS, SECONDS);
    } catch (TimeoutException | InterruptedException | ExecutionException e) {
      System.out.println("Exception: " + e);
      return false;
    }
    return true;
  }

  private void callDrainAllExecutors() {
    if (completionFuture.isDone()) {
      return;
    }
    activity.runOnUiThread(() -> nDrainAllExecutors(nativeTestHandle));
  }

  private WebView initWebView(Activity activity) {
    WebView wv = new WebView(activity);
    activity.setContentView(wv);
    wv.setWebChromeClient(new TestWebChromeClient());
    wv.getSettings().setJavaScriptEnabled(true);
    return wv;
  }

  /** Adds a custom WebChromeClient to forward JS console output to the native unit test. */
  private class TestWebChromeClient extends WebChromeClient {
    @Override
    public boolean onConsoleMessage(ConsoleMessage consoleMessage) {
      String type = "";
      String message = consoleMessage.message();
      switch (consoleMessage.messageLevel()) {
        case LOG:
          {
            type = "LOG";
            break;
          }
        case WARNING:
          {
            type = "WARNING";
            break;
          }
        case ERROR:
          {
            type = "ERROR";
            break;
          }
        case TIP:
        case DEBUG:
          {
            type = "DEBUG";
            break;
          }
      }
      nOnConsoleMessage(nativeTestHandle, type, message);
      if (message.startsWith("Testing complete")) {
        completionFuture.set(true);
      } else if (message.equals("call_drain_all_executors")) {
        callDrainAllExecutors();
      }

      return true;
    }
  }

  private static native void nDrainAllExecutors(long handle);

  private static native void nOnConsoleMessage(long handle, String type, String message);
}
