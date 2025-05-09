runit() {
  if $*
  then return 0
  else
    echo ">> FAILED: " "$*"
    return 2
  fi
}
blazetestit() {
  runit blaze test -c opt $*
}
blazebuildit() {
  runit blaze build -c opt $*
}
sceneformtest() {
  blazetestit third_party/arcore/ar/sceneform/viewer:viewer_basic_test &&
  blazetestit third_party/arcore/ar/sceneform/viewer:viewer_gltf_test &&
  blazetestit third_party/impress/core/loader/provider:provider_gltf_test
}
viewertest() {
  blazebuildit --config=wasm third_party/arcore/ar/sceneform/viewer:viewerwasm_web &&
  blazebuildit --config=arcore_linux_x64 third_party/arcore/ar/sceneform/loader:libloader-jni &&
  blazebuildit third_party/arcore/ar/sceneform/viewer:viewer &&
  blazebuildit --config=arcore_darwin_x64 third_party/arcore/ar/sceneform/viewer:viewer
}
othertests() {
  blazetestit //third_party/draco/src/draco/visualization:draco_to_filament_mesh_converter_test &&
  blazetestit //third_party/draco/src/draco/visualization:filament_mesh_reader_test &&
  blazetestit //third_party/draco/src/draco/visualization:snapshot_saver_test &&
  blazetestit //third_party/draco/src/draco/visualization:filament_environment_test &&
  blazetestit //robotics/simulation/cloudviz/python:utils_test &&
  blazebuildit //third_party/draco/src/draco/visualization:filament_platform &&
  blazebuildit //third_party/draco/src/draco/visualization:filament_platform_sdl &&
  blazebuildit //third_party/draco/src/draco/visualization:filament_platform_swiftshader &&
  blazebuildit //third_party/draco/src/draco/visualization:filament_platform_gpu_offscreen &&
  blazetestit //third_party/draco/src/draco/experimental/simplification/modules:parameterize_mesh_using_vector_fields_module_test &&
  blazetestit //third_party/draco/src/draco/mesh:mesh_material_transfer_test &&
  blazetestit //third_party/draco/src/draco/simplification/modules:cut_and_parameterize_mesh_using_uv_atlas_module_test &&
  blazetestit //third_party/draco/src/draco/simplification/modules:decimate_mesh_with_distortion_module_test &&
  blazetestit //third_party/draco/src/draco/simplification/modules:measure_distortion_module_test &&
  blazetestit //third_party/draco/src/draco/simplification/modules:optimize_texture_mapping_using_distortion_module_test &&
  blazetestit //third_party/draco/src/draco/simplification/modules:parameterize_mesh_using_perception_lscm_module_test &&
  blazetestit //third_party/draco/src/draco/simplification/modules:transfer_materials_module_test &&
  blazetestit //third_party/draco/src/draco/tools/distortion_meter:mesh_distortion_helper_test  &&
  blazebuildit //third_party/draco/src/draco/tools:draco_distortion_meter &&
  blazetestit //third_party/draco/src/draco/tools:draco_simplifier  &&
  blazetestit //third_party/draco/src/draco/tools:draco_visualizer &&
  blazetestit //third_party/draco/src/draco/visualization:draco_scene_to_filament_mesh_converter_test &&
  blazetestit //third_party/draco/src/draco/visualization:draco_to_filament_mesh_converter_test &&
  blazetestit //third_party/draco/src/draco/visualization:filament_environment_test  &&
  blazetestit //third_party/draco/src/draco/visualization:filament_mesh_reader_test &&
  blazetestit //third_party/arcore/javatests/com/google/ar/sceneform/modelviewer:ModelViewerScubaTest_mh   --test_output=streamed --notest_loasd --nocache_test_results
}
allofit() {
  sceneformtest &&
  viewertest &&
  othertests &&
  echo ">> ＼(^o^)／ "
}
echo "Filament Auditing Script"
allofit || echo ">> (-_-メ)"
