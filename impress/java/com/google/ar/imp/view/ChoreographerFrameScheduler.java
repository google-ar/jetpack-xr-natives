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

import static java.lang.Math.max;

import android.view.Choreographer;

/**
 * Implements a FrameScheduler using the Choreographer.
 *
 * <p>The Choreographer callback system provides frame pacing, so that frames are rendered at
 * regular intervals. ChoreographerFrameScheduler supports the ImpApi on the main thread or
 * background thread. ImpApi's retry frame mechanism is also supported.
 */
public final class ChoreographerFrameScheduler extends FrameScheduler {
  public static final String TAG = ChoreographerFrameScheduler.class.getSimpleName();

  /** Makes a new instance of the frame scheduler */
  public static class Factory implements FrameScheduler.Factory {
    @Override
    public FrameScheduler create(ThreadMode threadMode) {
      return new ChoreographerFrameScheduler(threadMode);
    }
  }

  /** Choreographer callback used to advance a frame while the frame loop is running. */
  private static class FrameCallback implements Choreographer.FrameCallback {
    FrameCallback(ChoreographerFrameScheduler frameScheduler) {
      this.frameScheduler = frameScheduler;
    }

    @Override
    public void doFrame(long frameTimeNanos) {
      frameScheduler.advance(frameTimeNanos);
    }

    private final ChoreographerFrameScheduler frameScheduler;
  }

  /** Handler callback used to retry a frame when it is skipped while the frame loop is running. */
  private static class RetryCallback implements Runnable {
    RetryCallback(ChoreographerFrameScheduler frameScheduler) {
      this.frameScheduler = frameScheduler;
    }

    @Override
    public void run() {
      frameScheduler.retry();
    }

    private final ChoreographerFrameScheduler frameScheduler;
  }

  // When a frame is skipped, this is the minimum amount of time to wait before retrying it.
  public static final long MINIMUM_RETRY_MS = 2;

  private FrameScheduler.FrameAdvancer frameAdvancer;

  private final ChoreographerHolder choreographerHolder;
  private final FrameCallback frameCallback;
  private final RetryCallback retryCallback;
  private long retryFrameTimeNanos = 0;
  // TODO: Consider using a GuardedBy annotation to ensure synchronicity.
  private State state = State.STOPPED;

  ChoreographerFrameScheduler(ThreadMode threadMode) {
    super(threadMode);
    this.state = State.STOPPED;
    this.choreographerHolder = new AndroidChoreographerHolder();
    this.frameCallback = new FrameCallback(this);
    this.retryCallback = new RetryCallback(this);
  }

  ChoreographerFrameScheduler(ThreadMode threadMode, ChoreographerHolder choreographerHolder) {
    super(threadMode);
    this.state = State.STOPPED;
    this.choreographerHolder = choreographerHolder;
    this.frameCallback = new FrameCallback(this);
    this.retryCallback = new RetryCallback(this);
  }

  /**
   * Initiates the frame loop and stops the previous frame loop if one was already running. The
   * FrameAdvancer will be called each frame based on the choreographer tick on the appropriate
   * thread. If the FrameAdvancer skips a frame it will be retried.
   */
  @Override
  public void startFrameLoop(FrameAdvancer frameAdvancer) {
    runOnFrameThread(
        () -> {
          stopFrameLoopImpl();
          startFrameLoopImpl(frameAdvancer);
        });
  }

  /** Stops the current frame loop if one is running. */
  @Override
  public void stopFrameLoop() {
    runOnFrameThread(this::stopFrameLoopImpl);
  }

  /** This method must be called on the frame thread. */
  private synchronized void startFrameLoopImpl(FrameAdvancer frameAdvancer) {
    if (state == State.RUNNING) {
      return;
    }

    // Choreographer.getInstance must be called on the frame thread to get the Choreographer
    // for that thread.
    choreographerHolder.postFrameCallback(frameCallback);
    this.frameAdvancer = frameAdvancer;
    state = State.RUNNING;
  }

  /** This method must be called on the frame thread. */
  private synchronized void stopFrameLoopImpl() {
    if (state == State.STOPPED) {
      return;
    }

    frameAdvancer = null;
    frameThread.getHandler().removeCallbacks(retryCallback);
    // Choreographer.getInstance must be called on the frame thread to get the Choreographer
    // for that thread.
    choreographerHolder.removeFrameCallback(frameCallback);
    state = State.STOPPED;
  }

  private void advance(long frameTimeNanos) {
    if (getState() != State.RUNNING) {
      return;
    }
    checkFrameThread();
    if (frameAdvancer == null) {
      throw new IllegalStateException("frameAdvancer cannot be null");
    }

    long nextFrameTimeNanos = frameTimeNanos;
    if (retryFrameTimeNanos != 0) {
      // A previous frame is still pending retry.  Since it is already scheduled, and frames should
      // not be processed out of order, swap s.t. the retry timestamp is processed now, and the new
      // timestamp is processed next.
      nextFrameTimeNanos = retryFrameTimeNanos;
      retryFrameTimeNanos = frameTimeNanos;
    }

    long result = frameAdvancer.advanceFrame(nextFrameTimeNanos);

    if (result < 0) {
      // A result of -1 indicates that advancing & rendering succeeded, in which case we should
      // wait until the next choreographer tick to advance again.
    } else if (retryFrameTimeNanos != 0) {
      // Don't retry; we were attempting to process an old frame and failed.  The swapped retry will
      // begin attempts at frameTimeNanos.
    } else {
      // Render failed without a pending retry; re-attempt this frame.
      retryFrameTimeNanos = frameTimeNanos;
      long retryMillis = max(result, MINIMUM_RETRY_MS);
      frameThread.getHandler().postDelayed(retryCallback, retryMillis);
    }
    Choreographer.getInstance().postFrameCallback(frameCallback);
  }

  private void retry() {
    if (getState() != State.RUNNING) {
      return;
    }
    checkFrameThread();
    if (frameAdvancer == null) {
      throw new IllegalStateException("frameAdvancer cannot be null");
    }

    if (retryFrameTimeNanos == 0) {
      // We only hold one retryFrameTimeNanos but can be hypothetically scheduled twice,
      // by a retry failure followed by advance failure.
      return;
    }
    long result = frameAdvancer.advanceFrame(retryFrameTimeNanos);
    if (result < 0) {
      // A result of -1 indicates that advancing & rendering succeeded, in which case we should
      // wait until the next choreographer tick to advance again.
      retryFrameTimeNanos = 0;
    } else {
      // Schedule a retry.
      long retryMillis = max(result, MINIMUM_RETRY_MS);
      frameThread.getHandler().postDelayed(retryCallback, retryMillis);
    }
  }

  public synchronized State getState() {
    return state;
  }
}
