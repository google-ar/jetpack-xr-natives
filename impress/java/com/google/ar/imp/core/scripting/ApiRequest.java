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

import com.google.protobuf.MessageLite;
import java.util.List;

/** A struct for storing the parameters needed to execute a scripting request. */
public final class ApiRequest<RequestT extends MessageLite> {
  private final String requestTypeUrl;
  private final RequestT request;
  private final List<Object> args;
  private final List<Object> out;

  public ApiRequest(String requestTypeUrl, RequestT request) {
    this.requestTypeUrl = requestTypeUrl;
    this.request = request;
    this.args = null;
    this.out = null;
  }

  public ApiRequest(String requestTypeUrl, RequestT request, List<Object> args) {
    this.requestTypeUrl = requestTypeUrl;
    this.request = request;
    this.args = args;
    this.out = null;
  }

  public ApiRequest(String requestTypeUrl, RequestT request, List<Object> args, List<Object> out) {
    this.requestTypeUrl = requestTypeUrl;
    this.request = request;
    this.args = args;
    this.out = out;
  }

  public String requestTypeUrl() {
    return requestTypeUrl;
  }

  public RequestT request() {
    return request;
  }

  public List<Object> args() {
    return args;
  }

  public List<Object> out() {
    return out;
  }
}
