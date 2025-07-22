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

package com.google.ar.imp.core.media;

import android.content.Context;
import android.net.Uri;
import android.view.Surface;
import androidx.annotation.IntDef;
import androidx.annotation.Nullable;
import androidx.media3.common.C;
import androidx.media3.common.Format;
import androidx.media3.common.MediaItem;
import androidx.media3.common.PlaybackParameters;
import androidx.media3.common.Player;
import androidx.media3.exoplayer.ExoPlayer;
import androidx.media3.exoplayer.SeekParameters;
import com.google.android.filament.proguard.UsedByNative;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.UUID;

/** Wrapper for ExoPlayer to handle internal state logic for Imp specific callback listeners. */
@UsedByNative("android_exoplayer.h")
final class ImpExoPlayer {

  static final int IMP_STATE_BUFFERING = 0;
  static final int IMP_STATE_BUFFERING_DONE = 1;

  // Listener that tracks the state of the player which we can propagate back to the c++ side.
  private class Listener implements Player.Listener {
    private static final int UNKNOWN = 0;
    private static final int BUFFERING = 1 << 0;
    private static final int SEEKING = 1 << 1;

    private int state;

    public Listener() {
      state = UNKNOWN;
    }

    @Override
    public void onEvents(Player player, Player.Events events) {
      if (events.contains(Player.EVENT_PLAYBACK_STATE_CHANGED)) {
        if (player.getPlaybackState() == Player.STATE_BUFFERING) {
          state |= BUFFERING;
          if (listener != null) {
            listener.onBuffering(IMP_STATE_BUFFERING);
          }
        }
        if (player.getPlaybackState() == Player.STATE_READY) {
          if (listener != null) {
            listener.onReady();
          }
          if ((state & BUFFERING) != 0) {
            state ^= BUFFERING;
            if (listener != null) {
              listener.onBuffering(IMP_STATE_BUFFERING_DONE);
            }
          }
          if ((state & SEEKING) != 0) {
            state ^= SEEKING;
            if (listener != null) {
              listener.onSeekComplete();
            }
          }
        }
        if (player.getPlaybackState() == Player.STATE_ENDED) {
          if (listener != null) {
            listener.onPlaybackComplete();
          }
        }
      }
    }

    @Override
    public void onPositionDiscontinuity(
        Player.PositionInfo oldPosition,
        Player.PositionInfo newPosition,
        @Player.DiscontinuityReason int reason) {
      if (reason == Player.DISCONTINUITY_REASON_SEEK) {
        state |= SEEKING;
      }
    }
  }

  @IntDef({SEEK_QUICK, SEEK_PRECISE})
  @interface SeekType {}

  static final int SEEK_QUICK = 0;
  static final int SEEK_PRECISE = 1;

  private final ExoPlayer player;
  @Nullable private ImpExoPlayerListener listener;

  @UsedByNative("android_exoplayer.h")
  public ImpExoPlayer(Context context) {
    ExoPlayer.Builder builder = new ExoPlayer.Builder(context);
    player = builder.build();
    player.addListener(new Listener());
  }

  @UsedByNative("android_exoplayer.h")
  public void release() {
    player.release();
  }

  @UsedByNative("android_exoplayer.h")
  public void setVideoSurface(Surface surface) {
    player.setVideoSurface(surface);
  }

  @UsedByNative("android_exoplayer.h")
  public void setMediaItem(String videoUrl) {
    Uri uri = Uri.parse(videoUrl);
    MediaItem mediaItem = MediaItem.fromUri(uri);
    player.setMediaItem(mediaItem);
  }

  @UsedByNative("android_exoplayer.h")
  public void setProtectedMediaItem(String videoUrl, String licenseUrl, String drmSchemeUuid) {
    Uri uri = Uri.parse(videoUrl);
    UUID drmScheme = UUID.fromString(drmSchemeUuid);

    MediaItem mediaItem =
        new MediaItem.Builder()
            .setUri(uri)
            .setDrmConfiguration(
                new MediaItem.DrmConfiguration.Builder(drmScheme).setLicenseUri(licenseUrl).build())
            .build();
    player.setMediaItem(mediaItem);
  }

  @UsedByNative("android_exoplayer.h")
  public void setListener(ImpExoPlayerListener listener) {
    this.listener = listener;
  }

  @UsedByNative("android_exoplayer.h")
  public void play() {
    player.play();
  }

  @UsedByNative("android_exoplayer.h")
  public void pause() {
    player.pause();
  }

  @UsedByNative("android_exoplayer.h")
  public void stop() {
    player.stop();
  }

  @UsedByNative("android_exoplayer.h")
  public void prepare() {
    player.prepare();
  }

  @UsedByNative("android_exoplayer.h")
  public void setPlaybackSpeed(float speed) {
    PlaybackParameters params = new PlaybackParameters(speed);
    player.setPlaybackParameters(params);
  }

  @UsedByNative("android_exoplayer.h")
  public void setLooping(boolean loop) {
    player.setRepeatMode(loop ? Player.REPEAT_MODE_ONE : Player.REPEAT_MODE_OFF);
  }

  @UsedByNative("android_exoplayer.h")
  public void seekTo(long milliseconds, @SeekType int seekType) {
    SeekParameters seekParam =
        seekType == SEEK_QUICK ? SeekParameters.PREVIOUS_SYNC : SeekParameters.CLOSEST_SYNC;
    player.setSeekParameters(seekParam);
    player.seekTo(milliseconds);
  }

  @UsedByNative("android_exoplayer.h")
  public void setVolume(float volume) {
    player.setVolume(volume);
  }

  @UsedByNative("android_exoplayer.h")
  public boolean isPlaying() {
    return player.isPlaying();
  }

  @UsedByNative("android_exoplayer.h")
  public boolean isLooping() {
    return player.getRepeatMode() == Player.REPEAT_MODE_ONE;
  }

  @UsedByNative("android_exoplayer.h")
  public long getDuration() {
    long duration = player.getDuration();
    return duration == C.TIME_UNSET ? -1 : duration;
  }

  @UsedByNative("android_exoplayer.h")
  public long getCurrentPosition() {
    return player.getCurrentPosition();
  }

  @UsedByNative("android_exoplayer.h")
  public int getVideoWidth() {
    return player.getVideoSize().width;
  }

  @UsedByNative("android_exoplayer.h")
  public int getVideoHeight() {
    return player.getVideoSize().height;
  }

  @UsedByNative("android_exoplayer.h")
  public int getStereoMode() {
    return player.getVideoFormat().stereoMode;
  }

  @UsedByNative("android_exoplayer.h")
  public int getColorStandard() {
    Format format = player.getVideoFormat();
    if (format != null && format.colorInfo != null) {
      return format.colorInfo.colorSpace;
    }
    return -1;
  }

  @UsedByNative("android_exoplayer.h")
  public int getColorTransfer() {
    Format format = player.getVideoFormat();
    if (format != null && format.colorInfo != null) {
      return format.colorInfo.colorTransfer;
    }
    return -1;
  }

  @UsedByNative("android_exoplayer.h")
  public int getColorRange() {
    Format format = player.getVideoFormat();
    if (format != null && format.colorInfo != null) {
      return format.colorInfo.colorRange;
    }
    return -1;
  }

  @UsedByNative("android_exoplayer.h")
  public int getLumaBitDepth() {
    Format format = player.getVideoFormat();
    if (format != null && format.colorInfo != null) {
      return format.colorInfo.lumaBitdepth;
    }
    // Default to 8 bits for unset chroma bitdepth (SDR/BT.709).
    return 8;
  }

  @UsedByNative("android_exoplayer.h")
  public int getChromaBitDepth() {
    Format format = player.getVideoFormat();
    if (format != null && format.colorInfo != null) {
      return format.colorInfo.chromaBitdepth;
    }
    // Default to 8 bits for unset chroma bitdepth (SDR/BT.709).
    return 8;
  }

  @UsedByNative("android_exoplayer.h")
  public int getMaxContentLightLevel() {
    Format format = player.getVideoFormat();
    if (format != null && format.colorInfo != null && format.colorInfo.hdrStaticInfo != null) {
      byte[] hdrStaticInfo = format.colorInfo.hdrStaticInfo;
      if (hdrStaticInfo.length >= 25) {
        ByteBuffer buffer = ByteBuffer.wrap(hdrStaticInfo);
        buffer.order(ByteOrder.LITTLE_ENDIAN);
        buffer.position(23);
        return buffer.getShort();
      }
    }
    return 0;
  }
}
