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

package com.google.ar.imp.core.web;

import android.util.Base64;
import android.util.Log;
import android.webkit.JavascriptInterface;
import android.webkit.WebView;
import com.google.android.filament.proguard.UsedByNative;

/** Communication bridge between JS and C++. */
@UsedByNative("web_view.cc")
final class ImpWebViewBridge {
  // LINT.IfChange
  private static final String JS_TO_NATIVE_ENTRY_POINT = "nativeEntryPoint";
  private static final String NATIVE_TO_JS_ENTRY_POINT =
      "window.javaScriptEntryPoint && window.javaScriptEntryPoint.incoming &&"
          + " window.javaScriptEntryPoint.incoming.postMessage";
  // LINT.ThenChange(
  //   //depot/google3/third_party/impress/javascript/core/scripting/src/script_bridge.ts
  // )
  private static final String JS_INJECTION_PREFIX = "(()=>{";
  private static final String JS_INJECTION_SUFFIX = "})();";

  private final long nativeHandle;
  private String injectionScript;
  private WebView webView;
  private boolean paused;

  @UsedByNative("web_view.cc")
  public ImpWebViewBridge(
      long nativeHandle, WebView webView, String injectionScript, boolean clearCache) {
    this.nativeHandle = nativeHandle;
    this.webView = webView;
    if (clearCache) {
      webView.clearCache(true);
    }
    safePost(
        () -> {
          this.webView.addJavascriptInterface(
              new ImpJavascriptInterface(), JS_TO_NATIVE_ENTRY_POINT);
          // Per the Android WebView documentation
          // (https://developer.android.com/reference/android/webkit/WebView#addJavascriptInterface(java.lang.Object,%20java.lang.String))
          // the JS interface instance will be available in the next page reload, so we are forcing
          // it to happen.
          // TODO: Try to improve the need to call this method.
          this.webView.reload();
        });
    this.paused = false;
    this.injectionScript = injectionScript;
  }

  /** Ends bridge activity. */
  public void end() {
    pause();
    this.webView = null;
  }

  /** Pauses bridge communication. */
  public void pause() {
    this.paused = true;
  }

  /** Resumes bridge communication. */
  public void resume() {
    this.paused = false;
  }

  class ImpJavascriptInterface {
    /** Exposes method to JavaScript for communication through Java to C++. */
    @JavascriptInterface
    public void postMessage(String input) {
      if (webView == null) {
        return;
      }
      if (paused) {
        // TODO: Report error to JS if paused or not started.
        return;
      }
      byte[] bytes = Base64.decode(input, Base64.DEFAULT);
      if (bytes.length != 0) {
        webView.post(() -> nPostMessage(nativeHandle, bytes));
      }
    }
  }

  @UsedByNative("web_view.cc")
  public void setInjectionScript(String injectionScript) {
    this.injectionScript = injectionScript;
  }

  @UsedByNative("web_view.cc")
  public void injectScript() {
    safePost(() -> webView.evaluateJavascript(formatInjection(injectionScript), null));
  }

  @UsedByNative("web_view.cc")
  public void postMessage(String message) {
    final String javascript = formatMessage(message);
    safePost(() -> webView.evaluateJavascript(javascript, ImpWebViewBridge::postMessageError));
  }

  private void safePost(Runnable runnable) {
    webView.post(
        () -> {
          if (this.webView == null) {
            Log.e("ImpWeb Error", "Tried to post runnable to a null WebView.");
          }
          runnable.run();
        });
  }

  private static void postMessageError(String value) {
    if (!value.equals("true")) {
      // TODO: Add error reporting from platform WebView to JS.
      Log.e("ImpWeb Error", "Failed to evaluateJavascript in postMessage call to JS.");
    }
  }

  private static String formatInjection(final String script) {
    return JS_INJECTION_PREFIX + script + JS_INJECTION_SUFFIX;
  }

  private static String formatMessage(String message) {
    return NATIVE_TO_JS_ENTRY_POINT + "(\"" + message + "\");";
  }

  static native void nPostMessage(long nativeHandle, byte[] bytes);
}
