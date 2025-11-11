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

package com.google.ar.imp.view;

import android.content.Context;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.SurfaceView;
import androidx.annotation.Nullable;
import com.google.ar.imp.core.web.FragmentHost;
import com.google.common.util.concurrent.Futures;
import com.google.common.util.concurrent.ListenableFuture;
import java.util.concurrent.Executor;

/** A SurfaceView that wraps a ImpApi. */
public final class ImpSurfaceView extends SurfaceView implements ImpApiProvider, InputEventSource {
  private final Context context;
  private final FragmentHost host;
  @Nullable private ImpApi impApi;
  @Nullable private InputEventHandler inputEventHandler;

  public ImpSurfaceView(Context context) {
    super(context);
    this.context = context;
    this.host = null;
  }

  public ImpSurfaceView(Context context, @Nullable AttributeSet attrs) {
    super(context, attrs);
    this.context = context;
    this.host = null;
  }

  public ImpSurfaceView(Context context, FragmentHost host) {
    super(context);
    this.context = context;
    this.host = host;
  }

  @Override
  public boolean hasOverlappingRendering() {
    return false;
  }

  @Override
  public ImpApi createImpApiSync(SetupParams setupParams) {
    impApi = ImpApi.createSync(setupParams, context, this, host);
    return impApi;
  }

  @Override
  public ImpApi createImpApiSync(
      SetupParams setupParams, @Nullable FrameScheduler.Factory frameSchedulerFactory) {
    impApi = ImpApi.createSync(setupParams, context, this, host, 0, frameSchedulerFactory);
    return impApi;
  }

  @Override
  public ImpApi createImpApiSync(SetupParams setupParams, long eglContext) {
    impApi = ImpApi.createSync(setupParams, context, this, host, eglContext);
    return impApi;
  }

  @Override
  public ImpApi createImpApiAsync(
      SetupParams setupParams, Executor executor, ImpApi.PostCreatedCallback postCreatedCallback) {
    impApi = ImpApi.createAsync(setupParams, context, this, host, executor, postCreatedCallback);
    return impApi;
  }

  @Override
  public ImpApi createImpApiAsync(
      SetupParams setupParams,
      long eglContext,
      Executor executor,
      ImpApi.PostCreatedCallback postCreatedCallback) {
    impApi =
        ImpApi.createAsync(
            setupParams, context, this, host, eglContext, executor, postCreatedCallback);
    return impApi;
  }

  @Override
  public ImpApi createImpApiAsync(
      SetupParams setupParams,
      long eglContext,
      Executor executor,
      ImpApi.PostCreatedCallback postCreatedCallback,
      @Nullable FrameScheduler.Factory frameSchedulerFactory) {
    impApi =
        ImpApi.createAsync(
            setupParams,
            context,
            this,
            host,
            eglContext,
            executor,
            postCreatedCallback,
            frameSchedulerFactory);
    return impApi;
  }

  public ListenableFuture<ImpApi> createImpApiAsync(
      SetupParams setupParams, Executor lightweightExecutor, Executor backgroundExecutor) {
    ListenableFuture<ImpApi> impApiFuture =
        ImpApi.createAsync(setupParams, context, this, host, backgroundExecutor);
    return Futures.transform(
        impApiFuture,
        impApi -> {
          ImpSurfaceView.this.impApi = impApi;
          return impApi;
        },
        lightweightExecutor);
  }

  @Override
  @Nullable
  public ImpApi getImpApi() {
    return impApi;
  }

  @Override
  public void setInputEventHandler(InputEventHandler inputEventHandler) {
    this.inputEventHandler = inputEventHandler;
  }

  @Override
  public boolean onTouchEvent(MotionEvent e) {
    if (inputEventHandler != null) {
      return inputEventHandler.onTouchEvent(e, this::impOnTouchEvent);
    }
    return impOnTouchEvent(e);
  }

  @Override
  public boolean onHoverEvent(MotionEvent e) {
    if (inputEventHandler != null) {
      return inputEventHandler.onHoverEvent(e, super::onHoverEvent);
    }
    return super.onHoverEvent(e);
  }

  @Override
  protected void onLayout(boolean changed, int l, int t, int r, int b) {
    super.onLayout(changed, l, t, r, b);

    if (impApi != null) {
      impApi.setDisplayRotation(getDisplay().getRotation());
    }
    return;
  }

  @Override
  public ImpApiScuba getImpApiScuba() {
    return impApi;
  }

  private boolean impOnTouchEvent(MotionEvent event) {
    if (impApi != null) {
      return impApi.onTouchEvent(event);
    }
    return false;
  }
}
