# Jetpack XR Natives

Jetpack XR Natives is a collection of native libraries that are used by the
[Jetpack XR SDK](https://developer.android.com/develop/xr/jetpack-xr-sdk).

## How to Build

This library requires [Bazel](https://bazel.build/) version 8.0.0 to build. Once
installed, this library can be built using the following commands:

```shell
bazel build //openxr:libandroidx.xr.arcore.openxr.so --config=android_arm64-v8a

bazel build //openxr:libandroidx.xr.arcore.openxr.so --config=android_x86_64

bazel build //openxr:libandroidx.xr.arcore.openxr.so --config=android_armeabi-v7a

bazel build //openxr:libandroidx.xr.arcore.openxr.so --config=android_x86_32
```