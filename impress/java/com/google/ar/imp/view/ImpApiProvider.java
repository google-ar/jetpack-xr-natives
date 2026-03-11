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

import androidx.annotation.Nullable;
import java.util.concurrent.Executor;

/**
 * Factory for creating ImpApi. Can be mocked for testing.
 *
 * <p>TODO Rename to ImpApiFactory.
 */
public interface ImpApiProvider extends ImpApiScubaProvider {

  /**
   * Create ImpApi synchronously, which will be ready immediately.
   *
   * <p>This is required before any calls to getImpApi().
   */
  @Deprecated
  public ImpApi createImpApiSync(SetupParams setupParams);

  /**
   * Create ImpApi synchronously, which will be ready immediately. This version of the method allows
   * you to pass in a parent EGL context that will be used when creating the Impress render context.
   *
   * <p>This is required before any calls to getImpApi().
   */
  @Deprecated
  public ImpApi createImpApiSync(SetupParams setupParams, long eglContext);

  /**
   * Create ImpApi synchronously, which will be ready immediately. This version of the method allows
   * you to pass in a frame scheduler factory that will be used to create the frame scheduler.
   *
   * <p>This is required before any calls to getImpApi().
   */
  @Deprecated
  public ImpApi createImpApiSync(
      SetupParams setupParams, @Nullable FrameScheduler.Factory frameSchedulerFactory);

  /**
   * Create ImpApi asynchronously. Refer to {@link ImpApi#createAsync()} for more details.
   *
   * <p>This is required before any calls to getImpApi().
   */
  @Deprecated
  public ImpApi createImpApiAsync(
      SetupParams setupParams, Executor executor, ImpApi.PostCreatedCallback postCreatedCallback);

  /**
   * Create ImpApi asynchronously. Refer to {@link ImpApi#createAsync()} for more details. This
   * version of the method allows you to pass in a parent EGL context that will be used when
   * creating the Impress render context.
   *
   * <p>This is required before any calls to getImpApi().
   */
  @Deprecated
  public ImpApi createImpApiAsync(
      SetupParams setupParams,
      long eglContext,
      Executor executor,
      ImpApi.PostCreatedCallback postCreatedCallback);

  /**
   * Create ImpApi asynchronously. Refer to {@link ImpApi#createAsync()} for more details. This
   * version of the method allows you to pass in a parent EGL context that will be used when
   * creating the Impress render context and a frame scheduler factory that will be used to create
   * the frame scheduler.
   *
   * <p>This is required before any calls to getImpApi().
   */
  @Deprecated
  public ImpApi createImpApiAsync(
      SetupParams setupParams,
      long eglContext,
      Executor executor,
      ImpApi.PostCreatedCallback postCreatedCallback,
      @Nullable FrameScheduler.Factory frameSchedulerFactory);

  /**
   * This will be available immediately after calling either createImpApi*() methods, but async will
   * no-op until it is done.
   *
   * <p>createImpApiSync() or createImpApiAsync() need to be called before this can return a valid
   * instance.
   */
  @Nullable
  public ImpApi getImpApi();
}
