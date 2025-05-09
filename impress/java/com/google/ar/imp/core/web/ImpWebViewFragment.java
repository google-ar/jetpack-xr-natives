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

import android.app.Activity;
import android.graphics.Color;
import android.os.Bundle;
import androidx.fragment.app.Fragment;
import androidx.fragment.app.FragmentManager;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.webkit.WebSettings;
import android.webkit.WebView;
import android.webkit.WebViewClient;
import android.widget.FrameLayout;
import com.google.android.filament.proguard.UsedByNative;

/**
 * Wraps an android.webkit.WebView instance along with an ImpWebViewBridge. The Fragment
 * capabilities are used for lifecycle handling. The WebView and bridge respond to host Activity
 * callbacks from the Fragment and properly clean up the WebView when it is destructed.
 */
public final class ImpWebViewFragment extends Fragment {
  private ImpWebViewBridge bridge;
  private WebView webView;
  private long webViewHandle;
  private float viewX;
  private float viewY;
  private int viewWidth;
  private int viewHeight;
  private String url;
  private String injectionScript;
  private boolean clearCache;

  private static final String USER_AGENT = "ImpWebView";
  private static final String URL_KEY = "url";
  private static final String NATIVE_HANDLE_KEY = "webViewHandle";
  private static final String VIEW_X_KEY = "x";
  private static final String VIEW_Y_KEY = "y";
  private static final String VIEW_WIDTH_KEY = "width";
  private static final String VIEW_HEIGHT_KEY = "height";
  private static final String INJECTION_SCRIPT_KEY = "injectionScript";

  /**
   * This function is called by the native side using the JNI to create a new ImpWebViewFragment
   * with a new WebView. The webViewHandle is a pointer to the native WebViewInternal and is used to
   * communicate with C++ and to identify the Fragment.
   */
  @UsedByNative("web_view.cc")
  public void inflate(
      FragmentHost host,
      long webViewHandle,
      float x,
      float y,
      int width,
      int height,
      String url,
      String injectionScript,
      boolean clearCache) {
    Bundle args = new Bundle();
    args.putLong(NATIVE_HANDLE_KEY, webViewHandle);
    args.putFloat(VIEW_X_KEY, x);
    args.putFloat(VIEW_Y_KEY, y);
    args.putInt(VIEW_WIDTH_KEY, width);
    args.putInt(VIEW_HEIGHT_KEY, height);
    args.putString(URL_KEY, url);
    args.putString(INJECTION_SCRIPT_KEY, injectionScript);
    this.setArguments(args);
    this.clearCache = clearCache;

    host.getSupportFragmentManager()
        .beginTransaction()
        .add(android.R.id.content, this, Long.toString(webViewHandle))
        .commit();
  }

  /** Called over JNI to remove the fragment when native web_view is destroyed. */
  @UsedByNative("web_view.cc")
  public static void deflate(FragmentHost host, long webViewHandle) {
    FragmentManager manager = host.getSupportFragmentManager();
    Fragment fragment = manager.findFragmentByTag(Long.toString(webViewHandle));
    if (fragment != null) {
      manager.beginTransaction().remove(fragment).commit();
    }
  }

  /** Loads the JNI library and saves Bundle args. */
  @Override
  public void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    Bundle args = getArguments();
    webViewHandle = args.getLong(NATIVE_HANDLE_KEY);
    viewX = args.getFloat(VIEW_X_KEY);
    viewY = args.getFloat(VIEW_Y_KEY);
    viewWidth = args.getInt(VIEW_WIDTH_KEY);
    viewHeight = args.getInt(VIEW_HEIGHT_KEY);
    url = args.getString(URL_KEY);
    injectionScript = args.getString(INJECTION_SCRIPT_KEY);
  }

  /** Initializes the WebView and the bridge. */
  @Override
  public View onCreateView(
      LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
    Activity activity = getActivity();
    FrameLayout rootView = new FrameLayout(activity);
    rootView.setLayoutParams(
        new ViewGroup.LayoutParams(
            ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT));
    webView = initWebView(activity);
    rootView.addView(webView, viewWidth, viewHeight);
    webView.setX(viewX);
    webView.setY(viewY);
    bridge = new ImpWebViewBridge(webViewHandle, webView, injectionScript, clearCache);
    return rootView;
  }

  @UsedByNative("web_view.cc")
  public void postMessage(String message) {
    this.bridge.postMessage(message);
  }

  @UsedByNative("web_view.cc")
  public void injectScript() {
    this.bridge.injectScript();
  }

  private WebView initWebView(Activity activity) {
    WebView wv = new WebView(activity);
    wv.setWebViewClient(new ImpWebViewClient());
    wv.setBackgroundColor(Color.TRANSPARENT);

    WebSettings settings = wv.getSettings();
    settings.setJavaScriptEnabled(true);
    settings.setAllowContentAccess(false);
    settings.setAllowFileAccess(false);
    settings.setAllowFileAccessFromFileURLs(false);
    settings.setAllowUniversalAccessFromFileURLs(false);
    settings.setSafeBrowsingEnabled(true);
    settings.setUserAgentString(USER_AGENT);

    wv.loadUrl(url);
    return wv;
  }

  /** Pauses webview and bridge when Activity stops. */
  @Override
  public void onPause() {
    super.onPause();
    webView.onPause();
    webView.pauseTimers();
    this.bridge.pause();
  }

  /** Resumes webview and bridge when Activity starts. */
  @Override
  public void onResume() {
    super.onResume();
    webView.onResume();
    webView.resumeTimers();
    this.bridge.resume();
  }

  /** Destroys webview and bridge. */
  @Override
  public void onDestroyView() {
    bridge.end();
    if (webView != null) {
      webView.destroy();
      webView = null;
    }
    nOnDestroyed(webViewHandle);
    super.onDestroyView();
  }

  private class ImpWebViewClient extends WebViewClient {
    /** Constructs a custom ImpWebViewClient to inject the ImpWeb API whenever a url is loaded. */
    public ImpWebViewClient() {}

    @Override
    public void onPageFinished(WebView view, String url) {
      super.onPageFinished(view, url);
      bridge.injectScript();
    }
  }

  private static native void nOnDestroyed(long handle);
}
