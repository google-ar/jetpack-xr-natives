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

import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import androidx.annotation.Nullable;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.LinkedBlockingQueue;
import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;

/**
 * A generic WebSocket server used by the Impress remote editor.
 *
 * <p>This server provides a communication channel for remote clients (e.g., a web-based UI). It
 * notifies registered {@link WebSocketListener}s of incoming messages and connection events,
 * allowing components to handle the application logic. All callbacks to registered listeners are
 * guaranteed to be executed on the Android main thread.
 *
 * <ul>
 *   <li><b>Incoming Messages:</b> When a message is received from a connected client, it is
 *       dispatched to all registered {@link WebSocketListener}s.
 *   <li><b>Sending Responses:</b> Messages can be sent to all connected clients using {@link
 *       #broadcast(String)}, {@link #broadcast(ByteBuffer)}, or {@link #broadcast(byte[])}, or to a
 *       specific client using {@code WebSocket.send()}.
 * </ul>
 */
public class RemoteEditorWebSocketServer extends WebSocketServer {
  private static final String TAG = RemoteEditorWebSocketServer.class.getSimpleName();
  private static final int MIN_POOL_BUFFER_SIZE = 1024;
  private static final int MAX_POOL_BUFFER_SIZE = 1024 * 1024;
  private static final int MAX_BUFFERS_PER_BUCKET = 10;
  private final Handler mainThreadHandler = new Handler(Looper.getMainLooper());
  private final List<WebSocketListener> listeners = new CopyOnWriteArrayList<>();

  /**
   * A pool of reusable byte buffers to reduce allocations during message handling. Uses
   * power-of-two sized buckets to pool buffers of similar sizes.
   */
  private final ConcurrentHashMap<Integer, LinkedBlockingQueue<byte[]>> bufferPool =
      new ConcurrentHashMap<>();

  /**
   * Borrows a buffer from the pool that is at least minSize bytes. If no suitable buffer is in the
   * pool, a new one is allocated. Buffers are organized into buckets based on power-of-two sizes to
   * minimize waste.
   */
  private byte[] borrowBuffer(int minSize) {
    // If the message is too large, allocate it directly to avoid pooling huge buffers.
    if (minSize > MAX_POOL_BUFFER_SIZE) {
      Log.w(TAG, "Large message size: " + minSize + ", allocating without pooling.");
      return new byte[minSize];
    }
    // Round minSize up to the next power of 2 to determine bucket size, starting from
    // MIN_POOL_BUFFER_SIZE. This keeps the number of buckets low and allows reuse of slightly
    // larger buffers.
    int bucketSize = MIN_POOL_BUFFER_SIZE;
    while (bucketSize < minSize) {
      bucketSize <<= 1;
    }

    // Get or create the queue for this bucket size.
    LinkedBlockingQueue<byte[]> bucket =
        bufferPool.computeIfAbsent(
            bucketSize, k -> new LinkedBlockingQueue<>(MAX_BUFFERS_PER_BUCKET));

    // Try to retrieve a buffer from the pool, or allocate a new one if the pool is empty.
    byte[] buffer = bucket.poll();
    return buffer != null ? buffer : new byte[bucketSize];
  }

  /** Releases a buffer back to the pool it was borrowed from. */
  private void releaseBuffer(byte[] buffer) {
    LinkedBlockingQueue<byte[]> bucket = bufferPool.get(buffer.length);
    if (bucket != null) {
      bucket.offer(buffer);
    }
  }

  /**
   * Interface for listening to WebSocket events. All callbacks are guaranteed to be executed on the
   * Android main thread.
   *
   * <p><b>Important:</b> Because callbacks are on the main thread, implementations must be fast and
   * non-blocking to avoid causing ANRs. If a listener needs to perform a long-running task in
   * response to a message, it must dispatch that work to its own background thread.
   */
  public interface WebSocketListener {
    /** Called when a new client connects to the WebSocket server. */
    void onConnected(WebSocket conn);

    /** Called when a connected client disconnects. */
    void onDisconnected(WebSocket conn);

    /**
     * Called when a string message is received from the client.
     *
     * @param message The string message received from the client.
     */
    void onStringMessage(String message);

    /**
     * Called when a binary message is received from the client.
     *
     * @param blob The binary message data received from the client.
     *     <p><b>Warning:</b> The underlying buffer for the {@code blob} ByteBuffer is pooled and
     *     reused for performance. The data in this buffer is only guaranteed to be valid for the
     *     duration of this callback. If you need to access the message data asynchronously or
     *     retain it after this method returns, you MUST copy the required data out of the buffer
     *     (e.g., by calling {@code blob.get(myByteArray)}).
     */
    void onBinaryMessage(ByteBuffer blob);
  }

  public RemoteEditorWebSocketServer(int port) {
    super(new InetSocketAddress(port));
    setReuseAddr(true);
  }

  public void addListener(WebSocketListener listener) {
    if (!listeners.contains(listener)) {
      listeners.add(listener);
    }
  }

  public void removeListener(WebSocketListener listener) {
    listeners.remove(listener);
  }

  public void startServer() {
    start();
  }

  @Override
  public void onOpen(WebSocket conn, ClientHandshake handshake) {
    Log.i(TAG, "New connection from [address = " + conn.getRemoteSocketAddress() + "]");
    mainThreadHandler.post(
        () -> {
          for (WebSocketListener listener : listeners) {
            listener.onConnected(conn);
          }
        });
  }

  @Override
  public void onClose(WebSocket conn, int code, String reason, boolean remote) {
    Log.i(
        TAG,
        "Connection closed from "
            + conn.getRemoteSocketAddress()
            + " with exit code "
            + code
            + " reason: "
            + reason);
    mainThreadHandler.post(
        () -> {
          for (WebSocketListener listener : listeners) {
            listener.onDisconnected(conn);
          }
        });
  }

  @Override
  public void onMessage(WebSocket conn, ByteBuffer blob) {
    int length = blob.remaining();
    // Borrow a buffer from the pool instead of allocating a new byte[] for each message.
    byte[] bytes = borrowBuffer(length);
    // Copy buffer contents since blob may be reused by network thread.
    blob.get(bytes, 0, length);

    mainThreadHandler.post(
        () -> {
          try {
            ByteBuffer message = ByteBuffer.wrap(bytes, 0, length);
            for (WebSocketListener listener : listeners) {
              // Reset position to 0 and limit to message length before each listener,
              // as listeners might consume or modify the buffer's position.
              message.position(0);
              message.limit(length); // Reset position and limit before each listener
              listener.onBinaryMessage(message);
            }
          } finally {
            // Ensure the buffer is returned to the pool even if a listener throws an exception.
            releaseBuffer(bytes);
          }
        });
  }

  @Override
  public void onMessage(WebSocket conn, String message) {
    mainThreadHandler.post(
        () -> {
          for (WebSocketListener listener : listeners) {
            listener.onStringMessage(message);
          }
        });
  }

  @Override
  public void onError(@Nullable WebSocket conn, Exception ex) {
    if (conn != null) {
      Log.e(TAG, "An error occurred on connection " + conn.getRemoteSocketAddress() + ":" + ex);
    } else {
      Log.e(TAG, "An error occurred: " + ex);
    }
  }

  @Override
  public void onStart() {
    Log.i(TAG, "WebSocket server started on [port = " + getPort() + "]");
  }
}
