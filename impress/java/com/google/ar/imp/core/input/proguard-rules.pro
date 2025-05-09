# Copyright 2024 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Add project specific ProGuard rules here.
# You can control the set of applied configuration files using the
# proguardFiles setting in build.gradle.
#
# For more details, see
#   http://developer.android.com/guide/developing/tools/proguard.html

# JNI is an entry point that's hard to keep track of, so there's
# an annotation to mark fields and methods used by native code.

# Keep the annotations that proguard needs to process.
-keep class com.google.android.filament.proguard.UsedBy*

# Just because native code accesses members of a class, does not mean that the
# class itself needs to be annotated - only annotate classes that are
# referenced themselves in native code.
-keep @com.google.android.filament.proguard.UsedBy* class * {
  <init>();
}
-keepclassmembers class * {
  @com.google.android.filament.proguard.UsedBy* *;
}
-keep class com.google.ar.imp.core.input.**{*;}
