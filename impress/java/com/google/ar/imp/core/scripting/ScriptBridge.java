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

package com.google.ar.imp.core.scripting;

import com.google.android.filament.proguard.UsedByNative;
import com.google.ar.imp.core.NodeHandleMessageOuterClass.NodeHandleMessage;
import com.google.ar.imp.core.scripting.Bridge.MessageToNative;
import com.google.ar.imp.core.scripting.Bridge.MessageToScript;
import com.google.ar.imp.core.scripting.Events.EventListenerAddRequest;
import com.google.ar.imp.core.scripting.Events.EventListenerAddResponse;
import com.google.ar.imp.core.scripting.Events.EventListenerMessage;
import com.google.ar.imp.core.scripting.Events.EventListenerRemoveRequest;
import com.google.common.util.concurrent.ListenableFuture;
import com.google.common.util.concurrent.SettableFuture;
import com.google.protobuf.Any;
import com.google.protobuf.MessageLite;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutionException;
import java.util.function.Consumer;

/** Handles MessageToNative and MessageToScript transport between Java and C++. */
public final class ScriptBridge implements ScriptEndpoint, ApiBridge {
  private final long scriptMessageHandlerProviderHandle;

  /**
   * A pending request for an async API call.
   *
   * <p>This class is abstract to allow for both MessageLite (response) and empty (void) responses.
   */
  private abstract static class PendingRequest<T> {
    final Class<T> type;
    final SettableFuture<T> future;

    PendingRequest(Class<T> type) {
      this.type = type;
      this.future = SettableFuture.create();
    }

    ListenableFuture<T> getFuture() {
      return future;
    }

    void setException(Exception e) {
      future.setException(e);
    }

    /** Resolves the pending request with the given message from native. */
    void resolve(MessageToScript messageToScript, Object out) {
      if (!messageToScript.getError().isEmpty()) {
        setException(new ApiException(messageToScript.getError()));
      } else {
        future.set(getResponse(messageToScript, out));
      }
    }

    abstract T getResponse(MessageToScript messageToScript, Object out);
  }

  /** A pending request for an async API call with a MessageLite response. */
  private static class PendingRequestWithMessageResponse<T extends MessageLite>
      extends PendingRequest<T> {
    PendingRequestWithMessageResponse(Class<T> type) {
      super(type);
    }

    @Override
    T getResponse(MessageToScript messageToScript, Object out) {
      if (out != null) {
        throw new ApiException("Out parameter is not supported with a Message response.");
      }
      return MessageUtils.getContent(type, messageToScript);
    }
  }

  /** A pending request for an async API call with an Object response. */
  private static class PendingRequestWithObjectResponse<T> extends PendingRequest<T> {
    PendingRequestWithObjectResponse(Class<T> type) {
      super(type);
    }

    @Override
    @SuppressWarnings(
        "unchecked") // The cast is checked in the method, which throws an ApiException if the
    // request returned an incompatible type.
    T getResponse(MessageToScript messageToScript, Object out) {
      if (!type.isAssignableFrom(out.getClass())) {
        throw new ApiException("Out parameter is not assignable to the response type.");
      }
      return (T) out;
    }
  }

  /** A pending request for an async API call with a void response. */
  private static class EmptyPendingRequest extends PendingRequest<Void> {
    EmptyPendingRequest() {
      super(Void.class);
    }

    @Override
    Void getResponse(MessageToScript messageToScript, Object out) {
      return null;
    }
  }

  private static class DefaultEventDispatcher implements EventDispatcher {
    @Override
    public <EventT extends MessageLite> void onEvent(EventListener<EventT> listener, Any event) {
      listener.onEvent(event);
    }
  }

  /** set if there is a custom event dispatcher. */
  private EventDispatcher eventDispatcher = new DefaultEventDispatcher();

  /** A map for routing async responses to PendingRequests based on request id. */
  private final Map<Integer, PendingRequest<?>> pendingRequests = new HashMap<>();

  /** A map for routing events to registered EventListeners by proto typeUrl. */
  private final Map<Integer, EventListener<?>> eventListeners = new HashMap<>();

  // Incrementing promise ID (int matches native type).
  private int nextPromiseId = 1;

  /** Creates a new ScriptBridge with the given view host handle for posting messages. */
  public ScriptBridge(long scriptMessageHandlerProviderHandle) {
    this.scriptMessageHandlerProviderHandle = scriptMessageHandlerProviderHandle;
  }

  @Override
  public <RequestT extends MessageLite, ResponseT> ResponseT sendRequest(
      ApiRequest<RequestT> request, Class<ResponseT> responseType) {
    ListenableFuture<ResponseT> future = sendRequestAsync(request, responseType);
    try {
      if (!future.isDone()) {
        throw new ApiException(
            "Synchronous request returned a future that was not immediately ready.");
      }
      return future.get();
    } catch (ExecutionException e) {
      if (e.getCause() instanceof ApiException) {
        // Avoid double wrapping the exception with ApiException.
        throw (ApiException) e.getCause();
      } else {
        throw new ApiException(e.getMessage(), e);
      }
    } catch (InterruptedException e) {
      throw new ApiException(e.getMessage(), e);
    }
  }

  @Override
  public <RequestT extends MessageLite> void sendRequest(ApiRequest<RequestT> request) {
    ListenableFuture<Void> future = sendRequestAsync(request);
    try {
      if (!future.isDone()) {
        throw new ApiException(
            "Synchronous request returned a future that was not immediately ready.");
      }
      Void unused = future.get();
    } catch (ExecutionException e) {
      if (e.getCause() instanceof ApiException) {
        // Avoid double wrapping the exception with ApiException.
        throw (ApiException) e.getCause();
      } else {
        throw new ApiException(e.getMessage(), e);
      }
    } catch (InterruptedException e) {
      throw new ApiException(e.getMessage(), e);
    }
  }

  @Override
  public <RequestT extends MessageLite, ResponseT> ListenableFuture<ResponseT> sendRequestAsync(
      ApiRequest<RequestT> request, Class<ResponseT> responseType) {
    if (responseType == null) {
      throw new IllegalArgumentException("Response type is null.");
    }
    // Since this method can handle both MessageLite and Object responses, check if the
    // response type is a MessageLite to determine which PendingRequest to use.
    // If the response type is a MessageLite, we can use the PendingRequestWithMessageResponse which
    // will parse the MessageToScript into the specific MessageLite type.
    // Otherwise, we use the PendingRequestWithObjectResponse which will pass the out parameter from
    // native to Java.
    if (MessageLite.class.isAssignableFrom(responseType)) {
      // We just ensured that the response type is a MessageLite subtype, so it's safe to presume an
      // `extends MessageLite` constraint for the below casts.
      @SuppressWarnings("unchecked") // Safe because isAssignableFrom is checked above.
      Class<? extends MessageLite> messageLiteType = (Class<? extends MessageLite>) responseType;
      @SuppressWarnings("unchecked") // Safe because isAssignableFrom is checked above.
      PendingRequest<ResponseT> pendingRequest =
          (PendingRequest<ResponseT>) new PendingRequestWithMessageResponse<>(messageLiteType);
      return sendRequestAsyncInternal(request, pendingRequest);
    } else {
      return sendRequestAsyncInternal(
          request, new PendingRequestWithObjectResponse<>(responseType));
    }
  }

  @Override
  public <RequestT extends MessageLite> ListenableFuture<Void> sendRequestAsync(
      ApiRequest<RequestT> request) {
    return sendRequestAsyncInternal(request, new EmptyPendingRequest());
  }

  private <RequestT extends MessageLite, ResponseT>
      ListenableFuture<ResponseT> sendRequestAsyncInternal(
          ApiRequest<RequestT> request, PendingRequest<ResponseT> pendingRequest) {
    SettableFuture<ResponseT> future = SettableFuture.create();
    int pendingRequestId = nextPromiseId++;
    pendingRequests.put(pendingRequestId, pendingRequest);
    try {
      Any content = MessageUtils.toAny(request.requestTypeUrl(), request.request());
      MessageToNative messageToNative =
          MessageToNative.newBuilder().setMessageId(pendingRequestId).setContent(content).build();

      byte[] byteArray = messageToNative.toByteArray();
      if (byteArray.length == 0) {
        throw new ApiException("Failed to serialize request.");
      }
      nPostMessage(this, scriptMessageHandlerProviderHandle, byteArray, request.args());
    } catch (ApiException ex) {
      // An async request can fail immediately, in which case we should set the exception on the
      // future and discard the pending request since we don't expect a message with that ID to be
      // sent in the future.
      pendingRequest.setException(ex);
      pendingRequests.remove(pendingRequestId);
    }
    return pendingRequest.getFuture();
  }

  @Override
  public void setEventDispatcher(EventDispatcher customEventDispatcher) {
    eventDispatcher = customEventDispatcher;
  }

  @Override
  public void removeEventDispatcher() {
    eventDispatcher = new DefaultEventDispatcher();
  }

  @Override
  public <EventT extends MessageLite> EventListenerHandle addEventListener(
      String eventTypeUrl,
      Class<EventT> eventType,
      NodeHandleMessage target,
      Consumer<EventT> listener) {
    EventListenerAddRequest.Builder eventListenerAddRequestBuilder =
        EventListenerAddRequest.newBuilder();
    if (target != null) {
      eventListenerAddRequestBuilder.setTarget(target);
    }
    eventListenerAddRequestBuilder.setEventTypeUrl(
        MessageUtils.getFullyQualifiedProtoTypeUrl(eventTypeUrl));

    EventListenerAddRequest eventListenerAddRequest = eventListenerAddRequestBuilder.build();

    EventListenerAddResponse response =
        sendRequest(
            "imp.scripting.EventListenerAddRequest",
            eventListenerAddRequest,
            EventListenerAddResponse.class);

    Integer id = response.getListenerId();
    EventListenerHandle handle = new EventListenerHandle(this, eventTypeUrl, id);
    eventListeners.put(id, EventListener.create(eventType, listener));
    return handle;
  }

  @Override
  public void removeEventListener(String eventTypeUrl, int id) {
    EventListenerRemoveRequest eventListenerRemoveRequest =
        EventListenerRemoveRequest.newBuilder().setListenerId(id).build();
    sendRequest("imp.scripting.EventListenerRemoveRequest", eventListenerRemoveRequest);
    eventListeners.remove(id);
  }

  /** Posts a MessageToScript from native to the Java scripting interface. */
  @Override
  @UsedByNative("scripting_bridge_jni.cc")
  public void postMessage(byte[] message, Object out) {
    MessageToScript messageToScript = MessageUtils.parseMessageToScript(message);
    Integer id = messageToScript.getMessageId();
    if (id != 0) {
      PendingRequest<?> pendingRequest = pendingRequests.get(id);
      if (pendingRequest != null) {
        pendingRequests.remove(id);
        pendingRequest.resolve(messageToScript, out);
      }
    } else {
      // If this is not a pending request, it must be an "event", i.e. a message type that comes
      // from
      // C++ without a corresponding API request.
      Any content = messageToScript.getContent();
      if (content
          .getTypeUrl()
          .equals(
              MessageUtils.getFullyQualifiedProtoTypeUrl("imp.scripting.EventListenerMessage"))) {
        EventListenerMessage eventListenerMessage =
            MessageUtils.getContent(EventListenerMessage.class, content);
        EventListener<?> listener = eventListeners.get(eventListenerMessage.getListenerId());
        if (listener != null) {
          eventDispatcher.onEvent(listener, eventListenerMessage.getEvent());
        }
      } else {
        // This is some other message that is not currently handled.
        throw new ApiException("Unhandled response - type: " + content.getTypeUrl());
      }
    }
  }

  // LINT.IfChange(scripting)
  private static native void nPostMessage(
      Object self, long scriptMessageHandlerProviderHandle, byte[] request, List<Object> args);
  // LINT.ThenChange(
  //
  // //depot/google3/third_party/impress/core/scripting/web/android/jni/scripting_bridge_jni.cc:scripting
  // )
}
