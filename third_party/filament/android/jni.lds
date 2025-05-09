VERS_1.0 {
  # Export JNI symbols and ANativeActivity_onCreate.
  # Android's NativeActivity expects ANativeActivity_onCreate as its entrypoint.
  global:
    Java_*;
    *JNI_OnLoad*;
    JNI_OnUnload;
    ANativeActivity_onCreate;
    *filament*BufferObject*;
    *filament*Camera*;
    *filament*Color*;
    *filament*Exposure*;
    *filament*Skybox*;
    *filament*Engine*;
    *filament*RenderableManager*;
    *filament*Aabb*;
    *filament*IndirectLight*;
    *filament*LightManager*;
    *filament*Renderer*;
    *filament*RenderTarget*;
    *filament*Scene*;
    *filament*Transform*;
    *filament*Material*;
    *filament*IndexBuffer*;
    *filament*VertexBuffer*;
    *filament*View*;
    *filament*Texture*;
    *filament*geometry*;

  # Hide everything else.
  local:
    *;
};
