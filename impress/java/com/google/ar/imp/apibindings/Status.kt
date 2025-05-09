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

package com.google.ar.imp.apibindings

// LINT.IfChange(status_conversion)

/** Sealed class for the status of a JNI call to the native side of the Impress API. */
public sealed class Status<out T> {
  /** Status of a successful JNI call. */
  public object Success : Status<Nothing>()

  /** Status of a successful JNI call that returns an Int value. */
  public data class SuccessWithIntValue(public val data: Int) : Status<Int>()

  /** Status of a successful JNI call that returns a Long value. */
  public data class SuccessWithLongValue(public val data: Long) : Status<Long>()

  /** Status of a failed JNI call. */
  public sealed class Error(public val message: String) : Status<Nothing>() {
    /** Error when a look up fails. */
    public data class NotFound(public val details: String) : Error(details)

    /** Error when an argument is invalid. */
    public data class InvalidArgument(public val details: String) : Error(details)

    /** Error when a native call fails. */
    public data class Internal(public val details: String) : Error(details)
  }
}

// LINT.ThenChange(//depot/google3/third_party/impress/apibindings/impress_api.cc:status_conversion)
