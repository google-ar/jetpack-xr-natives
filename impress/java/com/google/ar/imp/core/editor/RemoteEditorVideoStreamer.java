/*
 * Copyright 2026 Google LLC
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

package com.google.ar.imp.core.editor;

import android.media.MediaCodec;
import android.media.MediaCodecInfo;
import android.media.MediaFormat;
import android.os.Build.VERSION;
import android.os.Build.VERSION_CODES;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.Base64;
import android.util.Log;
import android.view.Surface;
import androidx.annotation.Nullable;
import androidx.annotation.VisibleForTesting;
import java.io.IOException;
import java.nio.ByteBuffer;
import org.json.JSONException;
import org.json.JSONObject;

/**
 * Handles MediaCodec initialization and encoding of the Impress editor UI.
 *
 * <p>This class sets up a persistent input surface for the native renderer to draw onto. It then
 * encodes this surface content into an AVC (H.264) video stream using {@link MediaCodec}.
 *
 * <p>It manages its own {@link RemoteEditorWebSocketServer} instance to broadcast encoded video
 * frames and configuration data to connected clients, and to receive control messages such as
 * resolution change requests.
 */
public class RemoteEditorVideoStreamer implements RemoteEditorWebSocketServer.WebSocketListener {
  private static final String TAG = RemoteEditorVideoStreamer.class.getSimpleName();

  private static final int VIDEO_HEIGHT_720P = 720;
  private static final int VIDEO_WIDTH_1080P = 1920;
  private static final int VIDEO_HEIGHT_1080P = 1080;
  private static final int VIDEO_HEIGHT_1440P = 1440;
  private static final int VIDEO_WIDTH_4K = 3840;
  private static final int VIDEO_HEIGHT_4K = 2160;
  private static final int DEFAULT_FPS = 30;
  private static final int BITRATE_720P = 5000000; // 5Mbps
  private static final int BITRATE_1080P = 8000000; // 8Mbps
  private static final int BITRATE_1440P = 16000000; // 16Mbps
  private static final int BITRATE_2160P = 45000000; // 45Mbps
  private static final int I_FRAME_INTERVAL = 2;
  private static final long REPEAT_FRAME_DELAY_US = 1000000L; // 1 second
  private static final int NAL_UNIT_HEADER_LENGTH = 4;
  private static final byte FRAME_TYPE_KEY = 1;
  private static final byte FRAME_TYPE_INTER = 2;
  private static final byte AVCC_CONFIGURATION_VERSION = 1;
  private static final byte AVCC_LENGTH_SIZE_MINUS_ONE = (byte) 0xFF;
  private static final byte AVCC_NUM_SPS_1 = (byte) 0xE1; // 1 SPS | 0xE0 reserved
  private static final byte AVCC_NUM_PPS_1 = 1;

  private int fps = DEFAULT_FPS;
  private int bitrate = BITRATE_1080P;
  private int bitrateMode = MediaCodecInfo.EncoderCapabilities.BITRATE_MODE_VBR;
  private long nativeServerWrapperPtr;

  @Nullable private volatile MediaCodec mediaCodec;
  private volatile JSONObject config = new JSONObject();
  @Nullable private String codecString;
  @Nullable private String description;
  @Nullable private String lastConfigSent;
  private int videoWidth = VIDEO_WIDTH_1080P;
  private int videoHeight = VIDEO_HEIGHT_1080P;
  private volatile RemoteEditorWebSocketServer server;
  private final Surface persistentInputSurface;
  private NativeInterface nativeInterface =
      new NativeInterface() {
        @Override
        public void setEditorUiRenderSurface(
            long nativeServerWrapperPtr, Surface surface, int width, int height) {
          nativeSetEditorUiRenderSurface(nativeServerWrapperPtr, surface, width, height);
        }

        @Override
        public void releaseEditorUiRenderSurface(long nativeServerWrapperPtr) {
          nativeReleaseEditorUiRenderSurface(nativeServerWrapperPtr);
        }
      };

  /** Constructs a new RemoteEditorVideoStreamer. */
  RemoteEditorVideoStreamer() {
    this.persistentInputSurface = MediaCodec.createPersistentInputSurface();
  }

  @Override
  public void onBinaryMessage(ByteBuffer blob) {
    // Ignore binary messages from UI Stream clients.
  }

  @Override
  public void onConnected(org.java_websocket.WebSocket conn) {
    if (config.length() > 0) {
      conn.send(config.toString());
      requestKeyFrame();
    }
  }

  @Override
  public void onDisconnected() {}

  @Override
  public void onStringMessage(String message) {
    try {
      JSONObject json = new JSONObject(message);
      if (json.optString("type").equals("resolution")) {
        int width = json.getInt("width");
        int height = json.getInt("height");

        new Handler(Looper.getMainLooper()).post(() -> changeResolution(width, height));
      }
    } catch (JSONException e) {
      Log.e(TAG, "Error parsing WebSocket message", e);
    } catch (RuntimeException e) {
      Log.e(TAG, "Unexpected error handling WebSocket message", e);
    }
  }

  private void changeResolution(int width, int height) {
    // We do NOT call nativeReleaseEditorUiRenderSurface here because we are reusing the persistent
    // surface. Instead, we stop the current codec and start a new one with the new resolution,
    // attaching it to the same persistent surface.
    stopCodec();

    // Ensure dimensions are even numbers (requirement for many video encoders)
    width &= ~1;
    height &= ~1;

    setVideoSize(width, height);

    try {
      // The server could be null if stopStreaming() was called after this message was queued
      // but before it was processed. If streaming is inactive, ignore the resolution change.
      if (server != null) {
        startStreaming(server.getPort(), nativeServerWrapperPtr, fps);
      }

      // The MediaCodec.Callback will automatically send the new config when onOutputFormatChanged
      // fires.
      // We just request a key frame to ensure we get a clean start once configured.
      requestKeyFrame();

    } catch (IOException e) {
      Log.e(TAG, "Failed to restart MediaCodec with new resolution", e);
    }
  }

  /**
   * Starts the video streamer with the default framerate.
   *
   * @param port The port to listen on.
   * @param nativeServerWrapperPtr The pointer to the native RemoteEditorServer C++ wrapper.
   * @throws IOException If the MediaCodec cannot be created or configured.
   * @throws IllegalStateException If {@code startStreaming()} is called on an already-started
   *     instance before {@code stopStreaming()} has been called.
   */
  void startStreaming(int port, long nativeServerWrapperPtr) throws IOException {
    startStreaming(port, nativeServerWrapperPtr, DEFAULT_FPS);
  }

  /**
   * Starts the video streamer with a specific framerate.
   *
   * @param port The port to listen on.
   * @param nativeServerWrapperPtr The pointer to the native RemoteEditorServer C++ wrapper.
   * @param fps The target frames per second.
   * @throws IOException If the MediaCodec cannot be created or configured.
   * @throws IllegalStateException If {@code startStreaming()} is called on an already-started
   *     instance before {@code stopStreaming()} has been called.
   */
  void startStreaming(int port, long nativeServerWrapperPtr, int fps) throws IOException {
    if (mediaCodec != null) {
      throw new IllegalStateException(
          "RemoteEditorVideoStreamer is already started. Call stopStreaming() before starting"
              + " again.");
    }
    if (!persistentInputSurface.isValid()) {
      throw new IllegalStateException(
          "RemoteEditorVideoStreamer has been released and cannot be restarted.");
    }

    startServer(port);

    this.nativeServerWrapperPtr = nativeServerWrapperPtr;
    nativeInterface.setEditorUiRenderSurface(
        nativeServerWrapperPtr, persistentInputSurface, videoWidth, videoHeight);
    this.fps = fps;
    mediaCodec = MediaCodec.createEncoderByType(MediaFormat.MIMETYPE_VIDEO_AVC);
    MediaCodecInfo.EncoderCapabilities caps =
        mediaCodec
            .getCodecInfo()
            .getCapabilitiesForType(MediaFormat.MIMETYPE_VIDEO_AVC)
            .getEncoderCapabilities();
    if (bitrateMode == MediaCodecInfo.EncoderCapabilities.BITRATE_MODE_VBR
        && !caps.isBitrateModeSupported(MediaCodecInfo.EncoderCapabilities.BITRATE_MODE_VBR)) {
      Log.w(TAG, "VBR bitrate mode not supported, falling back to CBR.");
      bitrateMode = MediaCodecInfo.EncoderCapabilities.BITRATE_MODE_CBR;
    }
    // Double check that the selected bitrate mode is supported.
    if (!caps.isBitrateModeSupported(bitrateMode)) {
      Log.w(
          TAG,
          "Bitrate mode " + bitrateMode + " is not supported. Trying to find a supported mode.");
      if (caps.isBitrateModeSupported(MediaCodecInfo.EncoderCapabilities.BITRATE_MODE_CBR)) {
        bitrateMode = MediaCodecInfo.EncoderCapabilities.BITRATE_MODE_CBR;
      } else if (caps.isBitrateModeSupported(MediaCodecInfo.EncoderCapabilities.BITRATE_MODE_VBR)) {
        bitrateMode = MediaCodecInfo.EncoderCapabilities.BITRATE_MODE_VBR;
      }
    }
    MediaFormat format =
        MediaFormat.createVideoFormat(MediaFormat.MIMETYPE_VIDEO_AVC, videoWidth, videoHeight);
    format.setInteger(
        MediaFormat.KEY_COLOR_FORMAT, MediaCodecInfo.CodecCapabilities.COLOR_FormatSurface);
    format.setInteger(MediaFormat.KEY_BIT_RATE, bitrate);
    format.setInteger(MediaFormat.KEY_FRAME_RATE, fps);
    format.setInteger(MediaFormat.KEY_I_FRAME_INTERVAL, I_FRAME_INTERVAL);
    format.setInteger(MediaFormat.KEY_BITRATE_MODE, bitrateMode);
    // Only repeat frame if we haven't seen a new one in 1 second.
    format.setLong(MediaFormat.KEY_REPEAT_PREVIOUS_FRAME_AFTER, REPEAT_FRAME_DELAY_US);
    // Limit the encoder output framerate to the target FPS, as the input surface
    // might be fed at the display refresh rate (e.g. 60, 90, 120Hz).
    if (VERSION.SDK_INT >= VERSION_CODES.Q) {
      format.setFloat(MediaFormat.KEY_MAX_FPS_TO_ENCODER, fps);
    }

    Log.i(TAG, "Configuring MediaCodec with format: " + format);
    try {
      mediaCodec.configure(format, null, null, MediaCodec.CONFIGURE_FLAG_ENCODE);
      // Attach the persistent surface to this new codec instance.
      mediaCodec.setInputSurface(persistentInputSurface);
    } catch (IllegalArgumentException | IllegalStateException e) {
      Log.e(TAG, "MediaCodec configure failed", e);
      throw e;
    }
    mediaCodec.setCallback(
        new MediaCodec.Callback() {
          @Override
          public void onInputBufferAvailable(MediaCodec codec, int index) {}

          @Override
          public void onOutputBufferAvailable(
              MediaCodec codec, int index, MediaCodec.BufferInfo info) {
            ByteBuffer outputBuffer = codec.getOutputBuffer(index);

            if ((info.flags & MediaCodec.BUFFER_FLAG_CODEC_CONFIG) != 0) {
              codec.releaseOutputBuffer(index, false);
              return;
            }

            int nalUnitLength = info.size - NAL_UNIT_HEADER_LENGTH;
            if (nalUnitLength < 0) {
              codec.releaseOutputBuffer(index, false);
              return;
            }

            // Combine frame type, length, and NAL unit into a single buffer to minimize
            // allocations.
            int totalSize = 1 + NAL_UNIT_HEADER_LENGTH + nalUnitLength;
            byte[] packet = new byte[totalSize];
            ByteBuffer packetBuffer = ByteBuffer.wrap(packet);

            byte frameType =
                (info.flags & MediaCodec.BUFFER_FLAG_KEY_FRAME) != 0
                    ? FRAME_TYPE_KEY
                    : FRAME_TYPE_INTER;
            packetBuffer.put(frameType);
            packetBuffer.putInt(nalUnitLength);

            outputBuffer.position(info.offset + NAL_UNIT_HEADER_LENGTH);
            outputBuffer.limit(info.offset + info.size);
            packetBuffer.put(outputBuffer);

            if (server != null) {
              server.broadcast(packet);
            }
            codec.releaseOutputBuffer(index, false);
          }

          @Override
          public void onError(MediaCodec codec, MediaCodec.CodecException e) {
            Log.e(TAG, "MediaCodec Error", e);
          }

          @Override
          public void onOutputFormatChanged(MediaCodec codec, MediaFormat format) {
            Log.i(TAG, "Encoder output format changed: " + format);
            ByteBuffer spsBuffer = format.getByteBuffer("csd-0");
            ByteBuffer ppsBuffer = format.getByteBuffer("csd-1");

            if (spsBuffer == null || ppsBuffer == null) {
              return;
            }

            byte[] sps = new byte[spsBuffer.remaining() - NAL_UNIT_HEADER_LENGTH];
            spsBuffer.position(spsBuffer.position() + NAL_UNIT_HEADER_LENGTH);
            spsBuffer.get(sps);

            byte[] pps = new byte[ppsBuffer.remaining() - NAL_UNIT_HEADER_LENGTH];
            ppsBuffer.position(ppsBuffer.position() + NAL_UNIT_HEADER_LENGTH);
            ppsBuffer.get(pps);

            codecString =
                String.format("avc1.%02X%02X%02X", sps[1] & 0xFF, sps[2] & 0xFF, sps[3] & 0xFF);
            byte[] avcc = buildAvcc(sps, pps);
            description = Base64.encodeToString(avcc, Base64.NO_WRAP);
            buildAndSendConfig(0);
          }
        });

    if (mediaCodec != null) {
      try {
        mediaCodec.start();
        requestKeyFrame();
      } catch (IllegalStateException e) {
        Log.e(TAG, "MediaCodec start failed", e);
        throw e;
      }
    }
  }

  private void requestKeyFrame() {
    if (mediaCodec != null) {
      Bundle params = new Bundle();
      params.putInt(MediaCodec.PARAMETER_KEY_REQUEST_SYNC_FRAME, 0);
      mediaCodec.setParameters(params);
    }
  }

  private synchronized void buildAndSendConfig(int rotation) {
    if (codecString == null || description == null) {
      return; // Not ready yet.
    }
    int deviceRotationDegrees = 0;
    switch (rotation) {
      case Surface.ROTATION_0 -> deviceRotationDegrees = 0;
      case Surface.ROTATION_90 -> deviceRotationDegrees = 90;
      case Surface.ROTATION_180 -> deviceRotationDegrees = 180;
      case Surface.ROTATION_270 -> deviceRotationDegrees = 270;
      default -> {}
    }
    int rotationDegrees = (-deviceRotationDegrees + 360) % 360;

    try {
      JSONObject newConfig = new JSONObject();
      newConfig.put("type", "config");
      newConfig.put("codec", codecString);
      newConfig.put("codedWidth", videoWidth);
      newConfig.put("codedHeight", videoHeight);
      newConfig.put("description", description);
      newConfig.put("rotation", rotationDegrees);
      newConfig.put("fps", fps);
      newConfig.put("bitrate", bitrate);
      newConfig.put("bitrateMode", bitrateMode);
      newConfig.put("iFrameInterval", I_FRAME_INTERVAL);

      if (!newConfig.toString().equals(lastConfigSent)) {
        config = newConfig;
        Log.i(TAG, "Sending config: " + config);
        // The server could be null if stopStreaming() was called after this callback was
        // queued but before it was processed. If streaming is inactive, we shouldn't
        // broadcast the config.
        if (server != null) {
          server.broadcast(config.toString());
        }
        lastConfigSent = config.toString();
        if (mediaCodec != null) {
          Bundle params = new Bundle();
          params.putInt(MediaCodec.PARAMETER_KEY_REQUEST_SYNC_FRAME, 0);
          mediaCodec.setParameters(params);
        }
      }
    } catch (JSONException e) {
      Log.e(TAG, "Error building or sending config", e);
    }
  }

  private void setVideoSize(int width, int height) {
    this.videoWidth = width;
    this.videoHeight = height;

    if (height == VIDEO_HEIGHT_720P) {
      this.bitrate = BITRATE_720P;
    } else if (height == VIDEO_HEIGHT_1080P) {
      this.bitrate = BITRATE_1080P;
    } else if (height == VIDEO_HEIGHT_1440P) {
      this.bitrate = BITRATE_1440P;
    } else if (height == VIDEO_HEIGHT_4K) {
      this.bitrate = BITRATE_2160P;
    } else {
      this.bitrate = BITRATE_1080P;
    }
  }

  /** Stops the video encoder. The streamer can be restarted after calling this. */
  void stopStreaming() {
    stopCodec();
    stopServer();
    if (nativeServerWrapperPtr != 0) {
      nativeInterface.releaseEditorUiRenderSurface(nativeServerWrapperPtr);
      nativeServerWrapperPtr = 0;
    }
  }

  /**
   * Final cleanup of the streamer's persistent resources. Once called, the streamer cannot be
   * restarted.
   */
  public void release() {
    stopStreaming();
    if (persistentInputSurface != null) {
      persistentInputSurface.release();
    }
  }

  private void startServer(int port) {
    if (server == null || server.getPort() != port) {
      stopServer();
      server = new RemoteEditorWebSocketServer(port);
      server.addListener(this);
      server.startServer();
    } else if (!server.getConnections().isEmpty()) {
      requestKeyFrame(); // Client might still be connected during a restart
    }
  }

  private void stopServer() {
    if (server != null) {
      server.removeListener(this);
      try {
        server.stop();
      } catch (IOException | InterruptedException e) {
        if (e instanceof InterruptedException) {
          Thread.currentThread().interrupt();
        }
        Log.e(TAG, "Error stopping video streaming WebSocket server", e);
      }
      server = null;
    }
  }

  private void stopCodec() {
    if (mediaCodec != null) {
      mediaCodec.stop();
      mediaCodec.release();
      mediaCodec = null;
    }
  }

  private byte[] buildAvcc(byte[] sps, byte[] pps) {
    // Build AVCDecoderConfigurationRecord structure as defined in ISO 14496-15, 5.3.3.1.
    // avcRecordSize = 1 byte for configurationVersion
    // + 3 bytes for AVCProfileIndication, profile_compatibility, AVCLevelIndication
    // + 1 byte for lengthSizeMinusOne
    // + 1 byte for numOfSequenceParameterSets
    // + 2 bytes for sequenceParameterSetLength
    // + sps.length (Sequence Parameter Set)
    // + 1 byte for numOfPictureParameterSets
    // + 2 bytes for pictureParameterSetLength
    // + pps.length (Picture Parameter Set)
    int avcRecordSize = 1 + 3 + 1 + 1 + 2 + sps.length + 1 + 2 + pps.length;
    ByteBuffer avcc = ByteBuffer.allocate(avcRecordSize);

    avcc.put(AVCC_CONFIGURATION_VERSION);
    avcc.put(sps[1]); // AVCProfileIndication
    avcc.put(sps[2]); // profile_compatibility
    avcc.put(sps[3]); // AVCLevelIndication
    avcc.put(AVCC_LENGTH_SIZE_MINUS_ONE);

    avcc.put(AVCC_NUM_SPS_1);
    avcc.putShort((short) sps.length);
    avcc.put(sps);

    avcc.put(AVCC_NUM_PPS_1);
    avcc.putShort((short) pps.length);
    avcc.put(pps);

    return avcc.array();
  }

  
  void setNativeInterface(NativeInterface nativeInterface) {
    this.nativeInterface = nativeInterface;
  }

  
  interface NativeInterface {
    void setEditorUiRenderSurface(
        long nativeServerWrapperPtr, Surface surface, int width, int height);

    void releaseEditorUiRenderSurface(long nativeServerWrapperPtr);
  }

  // LINT.IfChange(nativeSetEditorUiRenderSurface)
  private native void nativeSetEditorUiRenderSurface(
      long nativeServerWrapperPtr, Surface surface, int width, int height);

  // LINT.ThenChange(//depot/google3/third_party/impress/core/editor/remote_editor/remote_editor_video_streamer_jni.cc:nativeSetEditorUiRenderSurface)

  // LINT.IfChange(nativeReleaseEditorUiRenderSurface)
  private native void nativeReleaseEditorUiRenderSurface(long nativeServerWrapperPtr);
  // LINT.ThenChange(//depot/google3/third_party/impress/core/editor/remote_editor/remote_editor_video_streamer_jni.cc:nativeReleaseEditorUiRenderSurface)
}
