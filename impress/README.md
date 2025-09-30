# Impress API Bindings
Impress is a framework used by Jetpack XR for building 3D and XR experiences.
Impress includes the building blocks of a 3D/XR experience including both rendering and UX logic.

## How to Build

To build the Impress .jar:
```
bazel build //apibindings:impress_no_native_lib_jar
```

To build the Impress .so:
```
bazel build //apibindings:impress_api_jni --config=android_arm64-v8a
bazel build //apibindings:impress_api_jni --config=android_x86_64
bazel build //apibindings:impress_api_jni --config=android_armeabi-v7a
bazel build //apibindings:impress_api_jni --config=android_x86_32
```
