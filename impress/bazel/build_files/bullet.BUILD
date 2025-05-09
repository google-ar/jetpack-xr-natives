# Copyright 2025 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Description:
# The Bullet library for rigid body collisions.

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-layering_check",
        "-parse_headers",
    ],
)

licenses(["notice"])

exports_files(["LICENSE"])

# Provide a c_opts that is specific to only .c files.
# This is required for blaze build -c opt.
c_only_copts = [
    "-fexceptions",
    "-Wno-array-parameter",
    "-Wno-bitfield-constant-conversion",
    "-Wno-unused-variable",
    "-Wno-parentheses",
    "-Wno-self-assign",
    "-Wno-string-conversion",
    "-Wno-uninitialized",
    "-Wno-int-to-pointer-cast",
    "-Wno-error=frame-larger-than=",
    "-Wno-unused-value",
    "-Wno-unused-function",
    "-Wno-implicit-fallthrough",
    "-DUSE_MINICL",
    # Disable acquisition and release of GL objects, since we don't have
    # implementations for clEnqueueAcquireGLObjects() and
    # clEnqueueReleaseGLObjects().
    "-DNO_GL_OBJECT_SHARING",
]

copts = c_only_copts + [
    "-Wno-overloaded-virtual",
    "-Wno-non-virtual-dtor",
    "-Wno-reorder",
]

cc_library(
    name = "BulletInverseDynamics",
    srcs = glob([
        "src/BulletInverseDynamics/**/*.cpp",
        "src/BulletInverseDynamics/*.cpp",
    ]) + [
        "src/BulletInverseDynamics/IDConfig.hpp",
        "src/BulletInverseDynamics/details/MultiBodyTreeImpl.hpp",
        "src/BulletInverseDynamics/details/MultiBodyTreeInitCache.hpp",
    ],
    hdrs = glob([
        "src/BulletInverseDynamics/**/*.h",
        "src/BulletInverseDynamics/*.h",
    ]) + [
        "src/BulletInverseDynamics/IDMath.hpp",
        "src/BulletInverseDynamics/MultiBodyTree.hpp",
    ],
    copts = copts,
    features = ["-use_header_modules"],  # Incompatible with -fexception.
    includes = ["src"],
    deps = [
        ":BulletDynamics",
        ":LinearMath",
        ":loose_headers",
    ],
)

cc_library(
    name = "BulletInverseDynamicsDoublePrecision",
    srcs = glob([
        "src/BulletInverseDynamics/**/*.cpp",
        "src/BulletInverseDynamics/*.cpp",
    ]) + [
        "src/BulletInverseDynamics/IDConfig.hpp",
        "src/BulletInverseDynamics/details/MultiBodyTreeImpl.hpp",
        "src/BulletInverseDynamics/details/MultiBodyTreeInitCache.hpp",
    ],
    hdrs = glob([
        "src/BulletInverseDynamics/**/*.h",
        "src/BulletInverseDynamics/*.h",
    ]) + [
        "src/BulletInverseDynamics/IDMath.hpp",
        "src/BulletInverseDynamics/MultiBodyTree.hpp",
    ],
    copts = copts,
    features = ["-use_header_modules"],  # Incompatible with -fexception.
    includes = ["src"],
    deps = [
        ":BulletDynamicsDoublePrecision",
        ":LinearMathDoublePrecision",
    ],
)

cc_library(
    name = "BulletInverseDynamicsBuiltin",
    srcs = glob([
        "src/BulletInverseDynamics/**/*.cpp",
        "src/BulletInverseDynamics/*.cpp",
    ]) + [
        "src/BulletInverseDynamics/IDConfig.hpp",
        "src/BulletInverseDynamics/details/MultiBodyTreeImpl.hpp",
        "src/BulletInverseDynamics/details/MultiBodyTreeInitCache.hpp",
    ],
    hdrs = glob([
        "src/BulletInverseDynamics/**/*.h",
        "src/BulletInverseDynamics/*.h",
    ]) + [
        "src/BulletInverseDynamics/IDMath.hpp",
        "src/BulletInverseDynamics/MultiBodyTree.hpp",
    ],
    defines = [
        "BT_CUSTOM_INVERSE_DYNAMICS_CONFIG_H=IDConfigBuiltin.hpp",
    ],
    deps = [
        ":loose_headers",
    ],
)

cc_library(
    name = "BulletSoftBody",
    srcs = glob([
        "src/BulletSoftBody/**/*.cpp",
        "src/BulletSoftBody/*.cpp",
    ]),
    hdrs = glob([
        "src/BulletSoftBody/**/*.h",
        "src/BulletSoftBody/*.h",
    ]),
    copts = copts,
    features = ["-use_header_modules"],  # Incompatible with -fexception.
    includes = ["src"],
    deps = [
        ":BulletCollision",
        ":BulletDynamics",
        ":LinearMath",
    ],
)

cc_library(
    name = "BulletSoftBodyDoublePrecision",
    srcs = glob([
        "src/BulletSoftBody/**/*.cpp",
        "src/BulletSoftBody/*.cpp",
    ]),
    hdrs = glob([
        "src/BulletSoftBody/**/*.h",
        "src/BulletSoftBody/*.h",
    ]),
    copts = copts,
    features = ["-use_header_modules"],  # Incompatible with -fexception.
    includes = ["src"],
    deps = [
        ":BulletCollisionDoublePrecision",
        ":BulletDynamicsDoublePrecision",
        ":LinearMathDoublePrecision",
    ],
)

cc_library(
    name = "BulletCollisionWithSphereBoxCollisions",
    srcs = glob([
        "src/BulletCollision/**/*.cpp",
        "src/BulletCollision/*.cpp",
    ]),
    hdrs = glob([
        "src/BulletCollision/**/*.h",
        "src/BulletCollision/*.h",
    ]) + ["src/btBulletCollisionCommon.h"],
    copts = copts + [
        "-DUSE_BUGGY_SPHERE_BOX_ALGORITHM",
    ],
    features = ["-use_header_modules"],  # Incompatible with -fexception.
    includes = ["src"],
    deps = [":LinearMath"],
)

cc_library(
    name = "BulletCollisionDoublePrecisionWithSphereBoxCollisions",
    srcs = glob([
        "src/BulletCollision/**/*.cpp",
        "src/BulletCollision/*.cpp",
    ]),
    hdrs = glob([
        "src/BulletCollision/**/*.h",
        "src/BulletCollision/*.h",
    ]) + ["src/btBulletCollisionCommon.h"],
    copts = copts + [
        "-DUSE_BUGGY_SPHERE_BOX_ALGORITHM",
    ],
    features = ["-use_header_modules"],  # Incompatible with -fexception.
    includes = ["src"],
    deps = [":LinearMathDoublePrecision"],
)

cc_library(
    name = "BulletCollision",
    srcs = glob([
        "src/BulletCollision/**/*.cpp",
        "src/BulletCollision/*.cpp",
    ]),
    hdrs = glob([
        "src/BulletCollision/**/*.h",
        "src/BulletCollision/*.h",
    ]) + ["src/btBulletCollisionCommon.h"],
    copts = copts,
    features = ["-use_header_modules"],  # Incompatible with -fexception.
    includes = ["src"],
    deps = [":LinearMath"],
)

cc_library(
    name = "BulletCollisionDoublePrecision",
    srcs = glob([
        "src/BulletCollision/**/*.cpp",
        "src/BulletCollision/*.cpp",
    ]),
    hdrs = glob([
        "src/BulletCollision/**/*.h",
        "src/BulletCollision/*.h",
    ]) + ["src/btBulletCollisionCommon.h"],
    copts = copts,
    features = ["-use_header_modules"],  # Incompatible with -fexception.
    includes = ["src"],
    deps = [":LinearMathDoublePrecision"],
)

cc_library(
    name = "BulletDynamics",
    srcs = glob([
        "src/BulletDynamics/**/*.cpp",
        "src/BulletDynamics/*.cpp",
    ]),
    hdrs = glob([
        "src/BulletDynamics/**/*.h",
        "src/BulletDynamics/*.h",
    ]) + [
        "src/btBulletDynamicsCommon.h",
    ],
    copts = copts,
    features = ["-use_header_modules"],  # Incompatible with -fexception.
    includes = ["src"],
    deps = [":BulletCollision"],
)

cc_library(
    name = "BulletDynamicsDoublePrecision",
    srcs = glob([
        "src/BulletDynamics/**/*.cpp",
        "src/BulletDynamics/*.cpp",
    ]),
    hdrs = glob([
        "src/BulletDynamics/**/*.h",
        "src/BulletDynamics/*.h",
    ]) + [
        "src/btBulletDynamicsCommon.h",
    ],
    copts = copts,
    features = ["-use_header_modules"],  # Incompatible with -fexception.
    includes = ["src"],
    deps = [":BulletCollisionDoublePrecision"],
)

cc_library(
    name = "LinearMath",
    srcs = glob(["src/LinearMath/*.cpp"]),
    hdrs = glob([
        "src/LinearMath/*.h",
        "src/**/*.h",
    ]),
    copts = copts,
    defines = [
        "G3_TINYXML2",
    ],
    features = ["-use_header_modules"],  # Incompatible with -fexception.
    includes = ["src"],
)

cc_library(
    name = "LinearMathDoublePrecision",
    srcs = glob(["src/LinearMath/**/*.cpp"]),
    hdrs = glob(["src/LinearMath/*.h"]),
    copts = copts,
    defines = [
        "BT_THREADSAFE",
        "G3_TINYXML2",
        "BT_USE_DOUBLE_PRECISION",
    ],
    features = ["-use_header_modules"],  # Incompatible with -fexception.
    includes = ["src"],
    deps = [":loose_headers"],
)

cc_library(
    name = "Bullet3Common",
    srcs = glob(["src/Bullet3Common/*.cpp"]),
    hdrs = glob(["src/Bullet3Common/*.h"]),
    copts = copts,
    features = ["-use_header_modules"],  # Incompatible with -fexception.
    includes = ["src"],
)

filegroup(
    name = "all_hdrs",
    srcs = glob([
        "src/**/*.h",
    ]),
)

filegroup(
    name = "all_srcs",
    srcs = glob([
        "src/**/*.cpp",
        "src/**/*.cl",
    ]),
)

filegroup(
    name = "data",
    srcs = glob([
        "data/**/*",
    ]),
    data = glob(["data/**"]),
)

filegroup(
    name = "pybullet_data",
    srcs = glob([
        "examples/pybullet/gym/pybullet_data/**/*",
    ]),
)

cc_library(
    name = "BulletExamplesCommonInterfaces",
    srcs = glob(
        [
        ],
        exclude = [
        ],
    ),
    hdrs = glob(
        [
            "examples/CommonInterfaces/*.h",
        ],
    ),
    copts = copts,
    features = ["-use_header_modules"],  # Incompatible with -fexception.
    includes = [
        "examples/CommonInterfaces",
        "src",
    ],
    deps = [
    ],
)

cc_library(
    name = "OpenGLWindow_DynamicLoadAll",
    srcs = glob(
        [
            "examples/OpenGLWindow/*.cpp",
        ],
        exclude = [
            "examples/OpenGLWindow/Win32*.cpp",
        ],
    ) + [
        "examples/OpenGLWindow/GLInstancingRenderer.h",
        "examples/OpenGLWindow/GLPrimitiveRenderer.h",
        "examples/OpenGLWindow/GLRenderToTexture.h",
        "examples/OpenGLWindow/LoadShader.h",
        "examples/OpenGLWindow/SimpleOpenGL2App.h",
        "examples/OpenGLWindow/SimpleOpenGL2Renderer.h",
        "examples/OpenGLWindow/TwFonts.h",
        "examples/OpenGLWindow/X11OpenGLWindow.h",
        "examples/OpenGLWindow/fontstash.h",
        "examples/OpenGLWindow/opengl_fontstashcallbacks.h",
    ],
    hdrs = [
        "examples/OpenGLWindow/SimpleCamera.h",
        "examples/OpenGLWindow/SimpleOpenGL3App.h",
    ],
    copts = copts + [
        "-DGLEW_STATIC",
        "-DGLEW_INIT_OPENGL11_FUNCTIONS=1",
        "-DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1",
        "-DDYNAMIC_LOAD_X11_FUNCTIONS",
    ],
    features = ["-use_header_modules"],  # Incompatible with -fexception.
    includes = [
        "examples/OpenGLWindow",
        "examples/ThirdPartyLibs/optionalX11",
        "src",
    ],
    deps = [
        ":Bullet3Common",
        ":LinearMath",
        ":glad",
        ":loose_headers",
        "@stblib",
    ],
)

cc_library(
    name = "OpenGLWindowDoublePrecision_DynamicLoadAll",
    srcs = glob(
        [
            "examples/OpenGLWindow/*.cpp",
        ],
        exclude = [
            "examples/OpenGLWindow/Win32*.cpp",
            "examples/OpenGLWindow/EGLOpenGLWindow.cpp",
        ],
    ) + [
        "examples/OpenGLWindow/GLInstancingRenderer.h",
        "examples/OpenGLWindow/GLPrimitiveRenderer.h",
        "examples/OpenGLWindow/GLRenderToTexture.h",
        "examples/OpenGLWindow/LoadShader.h",
        "examples/OpenGLWindow/SimpleOpenGL2App.h",
        "examples/OpenGLWindow/SimpleOpenGL2Renderer.h",
        "examples/OpenGLWindow/TwFonts.h",
        "examples/OpenGLWindow/X11OpenGLWindow.h",
        "examples/OpenGLWindow/fontstash.h",
        "examples/OpenGLWindow/opengl_fontstashcallbacks.h",
    ],
    hdrs = [
        "examples/OpenGLWindow/SimpleCamera.h",
        "examples/OpenGLWindow/SimpleOpenGL3App.h",
    ],
    copts = copts + [
        "-DGLEW_STATIC",
        "-DGLEW_INIT_OPENGL11_FUNCTIONS=1",
        "-DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1",
        "-DDYNAMIC_LOAD_X11_FUNCTIONS",
        "-DBT_USE_EGL",  # Needed here to support loading EGL window backend through arguments
    ],
    features = ["-use_header_modules"],  # Incompatible with -fexception.
    includes = [
        "examples/OpenGLWindow",
        "examples/ThirdPartyLibs/optionalX11",
        "src",
    ],
    deps = [
        ":Bullet3Common",
        ":LinearMathDoublePrecision",
        ":glad",
        ":loose_headers",
        "@stblib",
    ],
)

cc_binary(
    name = "SimpleOpenGL3_DynamicLoadAll",
    srcs = ["examples/SimpleOpenGL3/main.cpp"],
    copts = copts + [
        "-DGLEW_STATIC",
        "-DGLEW_INIT_OPENGL11_FUNCTIONS=1",
        "-DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1",
        "-DDYNAMIC_LOAD_X11_FUNCTIONS",
    ],
    features = ["-use_header_modules"],  # Incompatible with -fexception.
    includes = [
        "examples",
        "examples/OpenGLWindow",
        "examples/ThirdPartyLibs/Glew",
        "examples/ThirdPartyLibs/optionalX11",
        "src",
    ],
    linkopts = ["-ldl"],
    deps = [
        ":Bullet3Common",
        ":OpenGLWindow_DynamicLoadAll",
    ],
)

cc_binary(
    name = "SimpleOpenGL3",
    srcs = ["examples/SimpleOpenGL3/main.cpp"],
    copts = [
        "-DGLEW_STATIC",
        "-DGLEW_INIT_OPENGL11_FUNCTIONS=1",
        "-DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1",
        "-DDYNAMIC_LOAD_X11_FUNCTIONS",
    ],
    features = ["-use_header_modules"],  # Incompatible with -fexception.
    includes = ["examples"],
    deps = [
        ":OpenGLWindow_DynamicLoadAll",
    ],
)

cc_binary(
    name = "ExampleBrowser",
    srcs = [
        "Extras/Serialize/BulletFileLoader/bChunk.cpp",
        "Extras/Serialize/BulletFileLoader/bDNA.cpp",
        "Extras/Serialize/BulletFileLoader/bFile.cpp",
        "Extras/Serialize/BulletFileLoader/btBulletFile.cpp",
        "Extras/Serialize/BulletWorldImporter/btBulletWorldImporter.cpp",
        "Extras/Serialize/BulletWorldImporter/btMultiBodyWorldImporter.cpp",
        "Extras/Serialize/BulletWorldImporter/btWorldImporter.cpp",
        "examples/BasicDemo/BasicExample.cpp",
        "examples/Benchmarks/BenchmarkDemo.cpp",
        "examples/BulletRobotics/FixJointBoxes.cpp",
        "examples/Collision/CollisionSdkC_Api.cpp",
        "examples/Collision/CollisionTutorialBullet2.cpp",
        "examples/Collision/Internal/Bullet2CollisionSdk.cpp",
        "examples/Collision/Internal/RealTimeBullet3CollisionSdk.cpp",
        "examples/Constraints/ConstraintDemo.cpp",
        "examples/Constraints/ConstraintPhysicsSetup.cpp",
        "examples/Constraints/Dof6Spring2Setup.cpp",
        "examples/Constraints/TestHingeTorque.cpp",
        "examples/DeformableDemo/ClothFriction.cpp",
        "examples/DeformableDemo/Collide.cpp",
        "examples/DeformableDemo/DeformableClothAnchor.cpp",
        "examples/DeformableDemo/DeformableContact.cpp",
        "examples/DeformableDemo/DeformableMultibody.cpp",
        "examples/DeformableDemo/DeformableRigid.cpp",
        "examples/DeformableDemo/DeformableSelfCollision.cpp",
        "examples/DeformableDemo/GraspDeformable.cpp",
        "examples/DeformableDemo/LargeDeformation.cpp",
        "examples/DeformableDemo/LoadDeformed.cpp",
        "examples/DeformableDemo/MultibodyClothAnchor.cpp",
        "examples/DeformableDemo/Pinch.cpp",
        "examples/DeformableDemo/PinchFriction.cpp",
        "examples/DeformableDemo/SplitImpulse.cpp",
        "examples/DeformableDemo/VolumetricDeformable.cpp",
        "examples/DynamicControlDemo/MotorDemo.cpp",
        "examples/Evolution/NN3DWalkers.cpp",
        "examples/ExampleBrowser/ExampleEntries.cpp",
        "examples/ExampleBrowser/GwenGUISupport/GraphingTexture.cpp",
        "examples/ExampleBrowser/GwenGUISupport/GwenParameterInterface.cpp",
        "examples/ExampleBrowser/GwenGUISupport/GwenProfileWindow.cpp",
        "examples/ExampleBrowser/GwenGUISupport/GwenTextureWindow.cpp",
        "examples/ExampleBrowser/GwenGUISupport/gwenUserInterface.cpp",
        "examples/ExampleBrowser/InProcessExampleBrowser.cpp",
        "examples/ExampleBrowser/main.cpp",
        "examples/ExtendedTutorials/Bridge.cpp",
        "examples/ExtendedTutorials/Chain.cpp",
        "examples/ExtendedTutorials/CompoundBoxes.cpp",
        "examples/ExtendedTutorials/InclinedPlane.cpp",
        "examples/ExtendedTutorials/MultiPendulum.cpp",
        "examples/ExtendedTutorials/MultipleBoxes.cpp",
        "examples/ExtendedTutorials/NewtonsCradle.cpp",
        "examples/ExtendedTutorials/NewtonsRopeCradle.cpp",
        "examples/ExtendedTutorials/RigidBodyFromObj.cpp",
        "examples/ExtendedTutorials/SimpleBox.cpp",
        "examples/ExtendedTutorials/SimpleCloth.cpp",
        "examples/ExtendedTutorials/SimpleJoint.cpp",
        "examples/ForkLift/ForkLiftDemo.cpp",
        "examples/FractureDemo/FractureDemo.cpp",
        "examples/FractureDemo/btFractureBody.cpp",
        "examples/FractureDemo/btFractureBody.h",
        "examples/FractureDemo/btFractureDynamicsWorld.cpp",
        "examples/GyroscopicDemo/GyroscopicSetup.cpp",
        "examples/Heightfield/HeightfieldExample.cpp",
        "examples/Importers/ImportBsp/BspConverter.cpp",
        "examples/Importers/ImportBsp/BspConverter.h",
        "examples/Importers/ImportBsp/BspLoader.cpp",
        "examples/Importers/ImportBsp/ImportBspExample.cpp",
        "examples/Importers/ImportBullet/SerializeSetup.cpp",
        "examples/Importers/ImportColladaDemo/ImportColladaSetup.cpp",
        "examples/Importers/ImportColladaDemo/LoadMeshFromCollada.cpp",
        "examples/Importers/ImportMJCFDemo/BulletMJCFImporter.cpp",
        "examples/Importers/ImportMJCFDemo/ImportMJCFSetup.cpp",
        "examples/Importers/ImportMeshUtility/b3ImportMeshUtility.cpp",
        "examples/Importers/ImportObjDemo/ImportObjExample.cpp",
        "examples/Importers/ImportObjDemo/LoadMeshFromObj.cpp",
        "examples/Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp",
        "examples/Importers/ImportSDFDemo/ImportSDFSetup.cpp",
        "examples/Importers/ImportSTLDemo/ImportSTLSetup.cpp",
        "examples/Importers/ImportURDFDemo/BulletUrdfImporter.cpp",
        "examples/Importers/ImportURDFDemo/ImportURDFSetup.cpp",
        "examples/Importers/ImportURDFDemo/MyMultiBodyCreator.cpp",
        "examples/Importers/ImportURDFDemo/URDF2Bullet.cpp",
        "examples/Importers/ImportURDFDemo/UrdfParser.cpp",
        "examples/Importers/ImportURDFDemo/urdfStringSplit.cpp",
        "examples/InverseDynamics/InverseDynamicsExample.cpp",
        "examples/InverseKinematics/InverseKinematicsExample.cpp",
        "examples/MultiBody/InvertedPendulumPDControl.cpp",
        "examples/MultiBody/KinematicMultiBodyExample.cpp",
        "examples/MultiBody/MultiBodyConstraintFeedback.cpp",
        "examples/MultiBody/MultiBodySoftContact.cpp",
        "examples/MultiBody/MultiDofDemo.cpp",
        "examples/MultiBody/Pendulum.cpp",
        "examples/MultiBody/SerialChains.cpp",
        "examples/MultiBody/TestJointTorqueSetup.cpp",
        "examples/MultiThreadedDemo/CommonRigidBodyMTBase.cpp",
        "examples/MultiThreadedDemo/MultiThreadedDemo.cpp",
        "examples/MultiThreading/MultiThreadingExample.cpp",
        "examples/MultiThreading/b3PosixThreadSupport.cpp",
        "examples/MultiThreading/b3ThreadSupportInterface.cpp",
        "examples/MultiThreading/b3Win32ThreadSupport.cpp",
        "examples/Planar2D/Planar2D.cpp",
        "examples/Raycast/RaytestDemo.cpp",
        "examples/ReducedDeformableDemo/ConservationTest.cpp",
        "examples/ReducedDeformableDemo/FreeFall.cpp",
        "examples/ReducedDeformableDemo/FrictionSlope.cpp",
        "examples/ReducedDeformableDemo/ModeVisualizer.cpp",
        "examples/ReducedDeformableDemo/ReducedBenchmark.cpp",
        "examples/ReducedDeformableDemo/ReducedCollide.cpp",
        "examples/ReducedDeformableDemo/ReducedGrasp.cpp",
        "examples/ReducedDeformableDemo/ReducedMotorGrasp.cpp",
        "examples/ReducedDeformableDemo/Springboard.cpp",
        "examples/RenderingExamples/CoordinateSystemDemo.cpp",
        "examples/RenderingExamples/DynamicTexturedCubeDemo.cpp",
        "examples/RenderingExamples/RaytracerSetup.cpp",
        "examples/RenderingExamples/RenderInstancingDemo.cpp",
        "examples/RenderingExamples/TimeSeriesCanvas.cpp",
        "examples/RenderingExamples/TimeSeriesExample.cpp",
        "examples/RenderingExamples/TimeSeriesFontData.cpp",
        "examples/RenderingExamples/TinyRendererSetup.cpp",
        "examples/RenderingExamples/TinyVRGui.cpp",
        "examples/RigidBody/KinematicRigidBodyExample.cpp",
        "examples/RigidBody/RigidBodySoftContact.cpp",
        "examples/RobotSimulator/b3RobotSimulatorClientAPI.cpp",
        "examples/RoboticsLearning/GripperGraspExample.cpp",
        "examples/RoboticsLearning/KukaGraspExample.cpp",
        "examples/RoboticsLearning/R2D2GraspExample.cpp",
        "examples/RollingFrictionDemo/RollingFrictionDemo.cpp",
        "examples/SharedMemory/GraphicsClientExample.cpp",
        "examples/SharedMemory/GraphicsServerExample.cpp",
        "examples/SharedMemory/IKTrajectoryHelper.cpp",
        "examples/SharedMemory/InProcessMemory.cpp",
        "examples/SharedMemory/PhysicsClient.cpp",
        "examples/SharedMemory/PhysicsClientC_API.cpp",
        "examples/SharedMemory/PhysicsClientExample.cpp",
        "examples/SharedMemory/PhysicsClientSharedMemory.cpp",
        "examples/SharedMemory/PhysicsClientSharedMemory_C_API.cpp",
        "examples/SharedMemory/PhysicsDirect.cpp",
        "examples/SharedMemory/PhysicsDirectC_API.cpp",
        "examples/SharedMemory/PhysicsLoopBack.cpp",
        "examples/SharedMemory/PhysicsLoopBackC_API.cpp",
        "examples/SharedMemory/PhysicsServer.cpp",
        "examples/SharedMemory/PhysicsServerCommandProcessor.cpp",
        "examples/SharedMemory/PhysicsServerExample.cpp",
        "examples/SharedMemory/PhysicsServerExampleBullet2.cpp",
        "examples/SharedMemory/PhysicsServerSharedMemory.cpp",
        "examples/SharedMemory/PosixSharedMemory.cpp",
        "examples/SharedMemory/RemoteGUIHelper.cpp",
        "examples/SharedMemory/SharedMemoryInProcessPhysicsC_API.cpp",
        "examples/SharedMemory/Win32SharedMemory.cpp",
        "examples/SharedMemory/b3PluginManager.cpp",
        "examples/SharedMemory/b3RobotSimulatorClientAPI_NoDirect.cpp",
        "examples/SharedMemory/b3RobotSimulatorClientAPI_NoGUI.cpp",
        "examples/SharedMemory/plugins/collisionFilterPlugin/collisionFilterPlugin.cpp",
        "examples/SharedMemory/plugins/pdControlPlugin/pdControlPlugin.cpp",
        "examples/SharedMemory/plugins/tinyRendererPlugin/TinyRendererVisualShapeConverter.cpp",
        "examples/SharedMemory/plugins/tinyRendererPlugin/tinyRendererPlugin.cpp",
        "examples/ThirdPartyLibs/BussIK/Jacobian.cpp",
        "examples/ThirdPartyLibs/BussIK/LinearR2.cpp",
        "examples/ThirdPartyLibs/BussIK/LinearR3.cpp",
        "examples/ThirdPartyLibs/BussIK/LinearR4.cpp",
        "examples/ThirdPartyLibs/BussIK/MatrixRmn.cpp",
        "examples/ThirdPartyLibs/BussIK/Misc.cpp",
        "examples/ThirdPartyLibs/BussIK/Node.cpp",
        "examples/ThirdPartyLibs/BussIK/Tree.cpp",
        "examples/ThirdPartyLibs/BussIK/VectorRn.cpp",
        "examples/ThirdPartyLibs/Gwen/Renderers/OpenGL_DebugFont.cpp",
        "examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp",
        "examples/TinyRenderer/TinyRenderer.cpp",
        "examples/TinyRenderer/geometry.cpp",
        "examples/TinyRenderer/model.cpp",
        "examples/TinyRenderer/our_gl.cpp",
        "examples/TinyRenderer/tgaimage.cpp",
        "examples/Tutorial/Dof6ConstraintTutorial.cpp",
        "examples/Tutorial/Tutorial.cpp",
        "examples/Vehicles/Hinge2Vehicle.cpp",
        "examples/VoronoiFracture/VoronoiFractureDemo.cpp",
        "examples/VoronoiFracture/VoronoiFractureDemo.h",
        "examples/VoronoiFracture/btConvexConvexMprAlgorithm.cpp",
        "src/Bullet3Common/b3AlignedAllocator.h",
        "src/Bullet3Common/b3AlignedObjectArray.h",
        "src/Bullet3Common/b3Logging.h",
        "src/Bullet3Common/b3Scalar.h",
        "src/Bullet3Common/shared/b3PlatformDefinitions.h",
        "src/BulletCollision/BroadphaseCollision/btBroadphaseProxy.h",
        "src/BulletCollision/CollisionDispatch/btCollisionObject.h",
        "src/BulletCollision/NarrowPhaseCollision/btManifoldPoint.h",
        "src/BulletDynamics/ConstraintSolver/btConstraintSolver.h",
        "src/BulletDynamics/ConstraintSolver/btContactSolverInfo.h",
        "src/BulletDynamics/ConstraintSolver/btJacobianEntry.h",
        "src/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h",
        "src/BulletDynamics/ConstraintSolver/btSolverBody.h",
        "src/BulletDynamics/ConstraintSolver/btSolverConstraint.h",
        "src/BulletDynamics/ConstraintSolver/btTypedConstraint.h",
        "src/BulletDynamics/Dynamics/btRigidBody.h",
        "src/BulletDynamics/MLCPSolvers/btLemkeAlgorithm.h",
        "src/BulletDynamics/MLCPSolvers/btLemkeSolver.h",
        "src/BulletDynamics/MLCPSolvers/btMLCPSolver.h",
        "src/BulletDynamics/MLCPSolvers/btMLCPSolverInterface.h",
        "src/BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h",
        "src/BulletInverseDynamics/IDConfigBuiltin.hpp",
        "src/BulletInverseDynamics/IDConfigEigen.hpp",
        "src/BulletInverseDynamics/IDErrorMessages.hpp",
        "src/BulletInverseDynamics/details/IDEigenInterface.hpp",
        "src/BulletInverseDynamics/details/IDLinearMathInterface.hpp",
        "src/BulletInverseDynamics/details/IDMatVec.hpp",
    ],
    copts = copts + [
        "-Wno-non-virtual-dtor",
        "-Wno-delete-non-virtual-dtor",
        "-Wno-unused-variable",
        "-Wno-missing-braces",
        "-Wno-format",
        "-Wno-format-security",
        "-Wno-c++11-narrowing",
        "-Wno-narrowing",
        "-Wframe-larger-than=534760",
        "-DGWEN_COMPILE_STATIC",
        "-DDONT_USE_GLUT",
    ],
    data = glob(["data/**"]),
    features = ["-use_header_modules"],  # Incompatible with -fexception.
    includes = [
        "examples",
        "examples/CommonInterfaces",
        "examples/ExampleBrowser",
        "examples/SharedMemory",
    ],
    deps = [
        ":BulletGwen",
        ":ExampleBrowserLib",
        ":OpenGLWindow",
        ":enet",
        "@stblib",
    ],
)

cc_binary(
    name = "ExampleBrowser_DynamicLoadAll",
    srcs = [
        "Extras/Serialize/BulletFileLoader/bChunk.cpp",
        "Extras/Serialize/BulletFileLoader/bDNA.cpp",
        "Extras/Serialize/BulletFileLoader/bFile.cpp",
        "Extras/Serialize/BulletFileLoader/btBulletFile.cpp",
        "Extras/Serialize/BulletWorldImporter/btBulletWorldImporter.cpp",
        "Extras/Serialize/BulletWorldImporter/btMultiBodyWorldImporter.cpp",
        "Extras/Serialize/BulletWorldImporter/btWorldImporter.cpp",
        "examples/BasicDemo/BasicExample.cpp",
        "examples/Benchmarks/BenchmarkDemo.cpp",
        "examples/BulletRobotics/FixJointBoxes.cpp",
        "examples/Collision/CollisionSdkC_Api.cpp",
        "examples/Collision/CollisionTutorialBullet2.cpp",
        "examples/Collision/Internal/Bullet2CollisionSdk.cpp",
        "examples/Collision/Internal/RealTimeBullet3CollisionSdk.cpp",
        "examples/Constraints/ConstraintDemo.cpp",
        "examples/Constraints/ConstraintPhysicsSetup.cpp",
        "examples/Constraints/Dof6Spring2Setup.cpp",
        "examples/Constraints/TestHingeTorque.cpp",
        "examples/DeformableDemo/ClothFriction.cpp",
        "examples/DeformableDemo/Collide.cpp",
        "examples/DeformableDemo/DeformableClothAnchor.cpp",
        "examples/DeformableDemo/DeformableContact.cpp",
        "examples/DeformableDemo/DeformableMultibody.cpp",
        "examples/DeformableDemo/DeformableRigid.cpp",
        "examples/DeformableDemo/DeformableSelfCollision.cpp",
        "examples/DeformableDemo/GraspDeformable.cpp",
        "examples/DeformableDemo/LargeDeformation.cpp",
        "examples/DeformableDemo/LoadDeformed.cpp",
        "examples/DeformableDemo/MultibodyClothAnchor.cpp",
        "examples/DeformableDemo/Pinch.cpp",
        "examples/DeformableDemo/PinchFriction.cpp",
        "examples/DeformableDemo/SplitImpulse.cpp",
        "examples/DeformableDemo/VolumetricDeformable.cpp",
        "examples/DynamicControlDemo/MotorDemo.cpp",
        "examples/Evolution/NN3DWalkers.cpp",
        "examples/ExampleBrowser/ExampleEntries.cpp",
        "examples/ExampleBrowser/GwenGUISupport/GraphingTexture.cpp",
        "examples/ExampleBrowser/GwenGUISupport/GwenParameterInterface.cpp",
        "examples/ExampleBrowser/GwenGUISupport/GwenProfileWindow.cpp",
        "examples/ExampleBrowser/GwenGUISupport/GwenTextureWindow.cpp",
        "examples/ExampleBrowser/GwenGUISupport/gwenUserInterface.cpp",
        "examples/ExampleBrowser/InProcessExampleBrowser.cpp",
        "examples/ExampleBrowser/main.cpp",
        "examples/ExtendedTutorials/Bridge.cpp",
        "examples/ExtendedTutorials/Chain.cpp",
        "examples/ExtendedTutorials/CompoundBoxes.cpp",
        "examples/ExtendedTutorials/InclinedPlane.cpp",
        "examples/ExtendedTutorials/MultiPendulum.cpp",
        "examples/ExtendedTutorials/MultipleBoxes.cpp",
        "examples/ExtendedTutorials/NewtonsCradle.cpp",
        "examples/ExtendedTutorials/NewtonsRopeCradle.cpp",
        "examples/ExtendedTutorials/RigidBodyFromObj.cpp",
        "examples/ExtendedTutorials/SimpleBox.cpp",
        "examples/ExtendedTutorials/SimpleCloth.cpp",
        "examples/ExtendedTutorials/SimpleJoint.cpp",
        "examples/ForkLift/ForkLiftDemo.cpp",
        "examples/FractureDemo/FractureDemo.cpp",
        "examples/FractureDemo/btFractureBody.cpp",
        "examples/FractureDemo/btFractureDynamicsWorld.cpp",
        "examples/GyroscopicDemo/GyroscopicSetup.cpp",
        "examples/Heightfield/HeightfieldExample.cpp",
        "examples/Importers/ImportBsp/BspConverter.cpp",
        "examples/Importers/ImportBsp/BspLoader.cpp",
        "examples/Importers/ImportBsp/ImportBspExample.cpp",
        "examples/Importers/ImportBullet/SerializeSetup.cpp",
        "examples/Importers/ImportColladaDemo/ImportColladaSetup.cpp",
        "examples/Importers/ImportColladaDemo/LoadMeshFromCollada.cpp",
        "examples/Importers/ImportMJCFDemo/BulletMJCFImporter.cpp",
        "examples/Importers/ImportMJCFDemo/ImportMJCFSetup.cpp",
        "examples/Importers/ImportMeshUtility/b3ImportMeshUtility.cpp",
        "examples/Importers/ImportObjDemo/ImportObjExample.cpp",
        "examples/Importers/ImportObjDemo/LoadMeshFromObj.cpp",
        "examples/Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp",
        "examples/Importers/ImportSDFDemo/ImportSDFSetup.cpp",
        "examples/Importers/ImportSTLDemo/ImportSTLSetup.cpp",
        "examples/Importers/ImportURDFDemo/BulletUrdfImporter.cpp",
        "examples/Importers/ImportURDFDemo/ImportURDFSetup.cpp",
        "examples/Importers/ImportURDFDemo/MyMultiBodyCreator.cpp",
        "examples/Importers/ImportURDFDemo/URDF2Bullet.cpp",
        "examples/Importers/ImportURDFDemo/UrdfParser.cpp",
        "examples/Importers/ImportURDFDemo/urdfStringSplit.cpp",
        "examples/InverseDynamics/InverseDynamicsExample.cpp",
        "examples/InverseKinematics/InverseKinematicsExample.cpp",
        "examples/MultiBody/InvertedPendulumPDControl.cpp",
        "examples/MultiBody/KinematicMultiBodyExample.cpp",
        "examples/MultiBody/MultiBodyConstraintFeedback.cpp",
        "examples/MultiBody/MultiBodySoftContact.cpp",
        "examples/MultiBody/MultiDofDemo.cpp",
        "examples/MultiBody/Pendulum.cpp",
        "examples/MultiBody/SerialChains.cpp",
        "examples/MultiBody/TestJointTorqueSetup.cpp",
        "examples/MultiThreadedDemo/CommonRigidBodyMTBase.cpp",
        "examples/MultiThreadedDemo/MultiThreadedDemo.cpp",
        "examples/MultiThreading/MultiThreadingExample.cpp",
        "examples/MultiThreading/b3PosixThreadSupport.cpp",
        "examples/MultiThreading/b3ThreadSupportInterface.cpp",
        "examples/MultiThreading/b3Win32ThreadSupport.cpp",
        "examples/Planar2D/Planar2D.cpp",
        "examples/Raycast/RaytestDemo.cpp",
        "examples/ReducedDeformableDemo/ConservationTest.cpp",
        "examples/ReducedDeformableDemo/FreeFall.cpp",
        "examples/ReducedDeformableDemo/FrictionSlope.cpp",
        "examples/ReducedDeformableDemo/ModeVisualizer.cpp",
        "examples/ReducedDeformableDemo/ReducedBenchmark.cpp",
        "examples/ReducedDeformableDemo/ReducedCollide.cpp",
        "examples/ReducedDeformableDemo/ReducedGrasp.cpp",
        "examples/ReducedDeformableDemo/ReducedMotorGrasp.cpp",
        "examples/ReducedDeformableDemo/Springboard.cpp",
        "examples/RenderingExamples/CoordinateSystemDemo.cpp",
        "examples/RenderingExamples/DynamicTexturedCubeDemo.cpp",
        "examples/RenderingExamples/RaytracerSetup.cpp",
        "examples/RenderingExamples/RenderInstancingDemo.cpp",
        "examples/RenderingExamples/TimeSeriesCanvas.cpp",
        "examples/RenderingExamples/TimeSeriesExample.cpp",
        "examples/RenderingExamples/TimeSeriesFontData.cpp",
        "examples/RenderingExamples/TinyRendererSetup.cpp",
        "examples/RenderingExamples/TinyVRGui.cpp",
        "examples/RigidBody/KinematicRigidBodyExample.cpp",
        "examples/RigidBody/RigidBodySoftContact.cpp",
        "examples/RobotSimulator/b3RobotSimulatorClientAPI.cpp",
        "examples/RoboticsLearning/GripperGraspExample.cpp",
        "examples/RoboticsLearning/KukaGraspExample.cpp",
        "examples/RoboticsLearning/R2D2GraspExample.cpp",
        "examples/RollingFrictionDemo/RollingFrictionDemo.cpp",
        "examples/SharedMemory/GraphicsClientExample.cpp",
        "examples/SharedMemory/GraphicsServerExample.cpp",
        "examples/SharedMemory/IKTrajectoryHelper.cpp",
        "examples/SharedMemory/InProcessMemory.cpp",
        "examples/SharedMemory/PhysicsClient.cpp",
        "examples/SharedMemory/PhysicsClientC_API.cpp",
        "examples/SharedMemory/PhysicsClientExample.cpp",
        "examples/SharedMemory/PhysicsClientSharedMemory.cpp",
        "examples/SharedMemory/PhysicsClientSharedMemory_C_API.cpp",
        "examples/SharedMemory/PhysicsDirect.cpp",
        "examples/SharedMemory/PhysicsDirectC_API.cpp",
        "examples/SharedMemory/PhysicsLoopBack.cpp",
        "examples/SharedMemory/PhysicsLoopBackC_API.cpp",
        "examples/SharedMemory/PhysicsServer.cpp",
        "examples/SharedMemory/PhysicsServerCommandProcessor.cpp",
        "examples/SharedMemory/PhysicsServerExample.cpp",
        "examples/SharedMemory/PhysicsServerExampleBullet2.cpp",
        "examples/SharedMemory/PhysicsServerSharedMemory.cpp",
        "examples/SharedMemory/PosixSharedMemory.cpp",
        "examples/SharedMemory/RemoteGUIHelper.cpp",
        "examples/SharedMemory/SharedMemoryInProcessPhysicsC_API.cpp",
        "examples/SharedMemory/Win32SharedMemory.cpp",
        "examples/SharedMemory/b3PluginManager.cpp",
        "examples/SharedMemory/b3RobotSimulatorClientAPI_NoDirect.cpp",
        "examples/SharedMemory/b3RobotSimulatorClientAPI_NoGUI.cpp",
        "examples/SharedMemory/plugins/collisionFilterPlugin/collisionFilterPlugin.cpp",
        "examples/SharedMemory/plugins/pdControlPlugin/pdControlPlugin.cpp",
        "examples/SharedMemory/plugins/tinyRendererPlugin/TinyRendererVisualShapeConverter.cpp",
        "examples/SharedMemory/plugins/tinyRendererPlugin/tinyRendererPlugin.cpp",
        "examples/ThirdPartyLibs/BussIK/Jacobian.cpp",
        "examples/ThirdPartyLibs/BussIK/LinearR2.cpp",
        "examples/ThirdPartyLibs/BussIK/LinearR3.cpp",
        "examples/ThirdPartyLibs/BussIK/LinearR4.cpp",
        "examples/ThirdPartyLibs/BussIK/MatrixRmn.cpp",
        "examples/ThirdPartyLibs/BussIK/Misc.cpp",
        "examples/ThirdPartyLibs/BussIK/Node.cpp",
        "examples/ThirdPartyLibs/BussIK/Tree.cpp",
        "examples/ThirdPartyLibs/BussIK/VectorRn.cpp",
        "examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp",
        "examples/TinyRenderer/TinyRenderer.cpp",
        "examples/TinyRenderer/geometry.cpp",
        "examples/TinyRenderer/model.cpp",
        "examples/TinyRenderer/our_gl.cpp",
        "examples/TinyRenderer/tgaimage.cpp",
        "examples/Tutorial/Dof6ConstraintTutorial.cpp",
        "examples/Tutorial/Tutorial.cpp",
        "examples/Vehicles/Hinge2Vehicle.cpp",
        "examples/VoronoiFracture/VoronoiFractureDemo.cpp",
        "examples/VoronoiFracture/btConvexConvexMprAlgorithm.cpp",
    ],
    copts = copts + [
        "-Wno-non-virtual-dtor",
        "-Wno-delete-non-virtual-dtor",
        "-Wno-unused-variable",
        "-Wno-missing-braces",
        "-Wno-format",
        "-Wno-format-security",
        "-Wno-c++11-narrowing",
        "-Wno-narrowing",
        "-Wframe-larger-than=534760",
        "-DGWEN_COMPILE_STATIC",
        "-DDONT_USE_GLUT",
        "-DGLEW_STATIC",
        "-DGLEW_INIT_OPENGL11_FUNCTIONS=1",
        "-DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1",
        "-DDYNAMIC_LOAD_X11_FUNCTIONS",
    ],
    data = glob(["data/*"]),
    features = ["-use_header_modules"],  # Incompatible with -fexception.
    includes = [
        "examples",
        "examples/CommonInterfaces",
        "examples/ExampleBrowser",
        "examples/SharedMemory",
        "examples/ThirdPartyLibs/Glew",
        "examples/ThirdPartyLibs/optionalX11",
    ],
    linkopts = ["-ldl"],
    deps = [
        ":BulletGwen_DynamicLoadAll",
        ":ExampleBrowserLib_DynamicLoadAll",
        ":OpenGLWindow_DynamicLoadAll",
        ":enet",
        ":loose_headers",
        "@stblib",
    ],
)

cc_binary(
    name = "ExampleBrowserDoublePrecision_DynamicLoadAll",
    srcs = [
        "Extras/Serialize/BulletFileLoader/bChunk.cpp",
        "Extras/Serialize/BulletFileLoader/bDNA.cpp",
        "Extras/Serialize/BulletFileLoader/bFile.cpp",
        "Extras/Serialize/BulletFileLoader/btBulletFile.cpp",
        "Extras/Serialize/BulletWorldImporter/btBulletWorldImporter.cpp",
        "Extras/Serialize/BulletWorldImporter/btMultiBodyWorldImporter.cpp",
        "Extras/Serialize/BulletWorldImporter/btWorldImporter.cpp",
        "examples/BasicDemo/BasicExample.cpp",
        "examples/Benchmarks/BenchmarkDemo.cpp",
        "examples/BulletRobotics/FixJointBoxes.cpp",
        "examples/Collision/CollisionSdkC_Api.cpp",
        "examples/Collision/CollisionTutorialBullet2.cpp",
        "examples/Collision/Internal/Bullet2CollisionSdk.cpp",
        "examples/Collision/Internal/RealTimeBullet3CollisionSdk.cpp",
        "examples/Constraints/ConstraintDemo.cpp",
        "examples/Constraints/ConstraintPhysicsSetup.cpp",
        "examples/Constraints/Dof6Spring2Setup.cpp",
        "examples/Constraints/TestHingeTorque.cpp",
        "examples/DeformableDemo/ClothFriction.cpp",
        "examples/DeformableDemo/Collide.cpp",
        "examples/DeformableDemo/DeformableClothAnchor.cpp",
        "examples/DeformableDemo/DeformableContact.cpp",
        "examples/DeformableDemo/DeformableMultibody.cpp",
        "examples/DeformableDemo/DeformableRigid.cpp",
        "examples/DeformableDemo/DeformableSelfCollision.cpp",
        "examples/DeformableDemo/GraspDeformable.cpp",
        "examples/DeformableDemo/LargeDeformation.cpp",
        "examples/DeformableDemo/LoadDeformed.cpp",
        "examples/DeformableDemo/MultibodyClothAnchor.cpp",
        "examples/DeformableDemo/Pinch.cpp",
        "examples/DeformableDemo/PinchFriction.cpp",
        "examples/DeformableDemo/SplitImpulse.cpp",
        "examples/DeformableDemo/VolumetricDeformable.cpp",
        "examples/DynamicControlDemo/MotorDemo.cpp",
        "examples/Evolution/NN3DWalkers.cpp",
        "examples/ExampleBrowser/ExampleEntries.cpp",
        "examples/ExampleBrowser/GwenGUISupport/GraphingTexture.cpp",
        "examples/ExampleBrowser/GwenGUISupport/GwenParameterInterface.cpp",
        "examples/ExampleBrowser/GwenGUISupport/GwenProfileWindow.cpp",
        "examples/ExampleBrowser/GwenGUISupport/GwenTextureWindow.cpp",
        "examples/ExampleBrowser/GwenGUISupport/gwenUserInterface.cpp",
        "examples/ExampleBrowser/InProcessExampleBrowser.cpp",
        "examples/ExampleBrowser/main.cpp",
        "examples/ExtendedTutorials/Bridge.cpp",
        "examples/ExtendedTutorials/Chain.cpp",
        "examples/ExtendedTutorials/CompoundBoxes.cpp",
        "examples/ExtendedTutorials/InclinedPlane.cpp",
        "examples/ExtendedTutorials/MultiPendulum.cpp",
        "examples/ExtendedTutorials/MultipleBoxes.cpp",
        "examples/ExtendedTutorials/NewtonsCradle.cpp",
        "examples/ExtendedTutorials/NewtonsRopeCradle.cpp",
        "examples/ExtendedTutorials/RigidBodyFromObj.cpp",
        "examples/ExtendedTutorials/SimpleBox.cpp",
        "examples/ExtendedTutorials/SimpleCloth.cpp",
        "examples/ExtendedTutorials/SimpleJoint.cpp",
        "examples/ForkLift/ForkLiftDemo.cpp",
        "examples/FractureDemo/FractureDemo.cpp",
        "examples/FractureDemo/btFractureBody.cpp",
        "examples/FractureDemo/btFractureDynamicsWorld.cpp",
        "examples/GyroscopicDemo/GyroscopicSetup.cpp",
        "examples/Heightfield/HeightfieldExample.cpp",
        "examples/Importers/ImportBsp/BspConverter.cpp",
        "examples/Importers/ImportBsp/BspLoader.cpp",
        "examples/Importers/ImportBsp/ImportBspExample.cpp",
        "examples/Importers/ImportBullet/SerializeSetup.cpp",
        "examples/Importers/ImportColladaDemo/ImportColladaSetup.cpp",
        "examples/Importers/ImportColladaDemo/LoadMeshFromCollada.cpp",
        "examples/Importers/ImportMJCFDemo/BulletMJCFImporter.cpp",
        "examples/Importers/ImportMJCFDemo/ImportMJCFSetup.cpp",
        "examples/Importers/ImportMeshUtility/b3ImportMeshUtility.cpp",
        "examples/Importers/ImportObjDemo/ImportObjExample.cpp",
        "examples/Importers/ImportObjDemo/LoadMeshFromObj.cpp",
        "examples/Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp",
        "examples/Importers/ImportSDFDemo/ImportSDFSetup.cpp",
        "examples/Importers/ImportSTLDemo/ImportSTLSetup.cpp",
        "examples/Importers/ImportURDFDemo/BulletUrdfImporter.cpp",
        "examples/Importers/ImportURDFDemo/ImportURDFSetup.cpp",
        "examples/Importers/ImportURDFDemo/MyMultiBodyCreator.cpp",
        "examples/Importers/ImportURDFDemo/URDF2Bullet.cpp",
        "examples/Importers/ImportURDFDemo/UrdfParser.cpp",
        "examples/Importers/ImportURDFDemo/urdfStringSplit.cpp",
        "examples/InverseDynamics/InverseDynamicsExample.cpp",
        "examples/InverseKinematics/InverseKinematicsExample.cpp",
        "examples/MultiBody/InvertedPendulumPDControl.cpp",
        "examples/MultiBody/KinematicMultiBodyExample.cpp",
        "examples/MultiBody/MultiBodyConstraintFeedback.cpp",
        "examples/MultiBody/MultiBodySoftContact.cpp",
        "examples/MultiBody/MultiDofDemo.cpp",
        "examples/MultiBody/Pendulum.cpp",
        "examples/MultiBody/SerialChains.cpp",
        "examples/MultiBody/TestJointTorqueSetup.cpp",
        "examples/MultiThreadedDemo/CommonRigidBodyMTBase.cpp",
        "examples/MultiThreadedDemo/MultiThreadedDemo.cpp",
        "examples/MultiThreading/MultiThreadingExample.cpp",
        "examples/MultiThreading/b3PosixThreadSupport.cpp",
        "examples/MultiThreading/b3ThreadSupportInterface.cpp",
        "examples/MultiThreading/b3Win32ThreadSupport.cpp",
        "examples/Planar2D/Planar2D.cpp",
        "examples/Raycast/RaytestDemo.cpp",
        "examples/ReducedDeformableDemo/ConservationTest.cpp",
        "examples/ReducedDeformableDemo/FreeFall.cpp",
        "examples/ReducedDeformableDemo/FrictionSlope.cpp",
        "examples/ReducedDeformableDemo/ModeVisualizer.cpp",
        "examples/ReducedDeformableDemo/ReducedBenchmark.cpp",
        "examples/ReducedDeformableDemo/ReducedCollide.cpp",
        "examples/ReducedDeformableDemo/ReducedGrasp.cpp",
        "examples/ReducedDeformableDemo/ReducedMotorGrasp.cpp",
        "examples/ReducedDeformableDemo/Springboard.cpp",
        "examples/RenderingExamples/CoordinateSystemDemo.cpp",
        "examples/RenderingExamples/DynamicTexturedCubeDemo.cpp",
        "examples/RenderingExamples/RaytracerSetup.cpp",
        "examples/RenderingExamples/RenderInstancingDemo.cpp",
        "examples/RenderingExamples/TimeSeriesCanvas.cpp",
        "examples/RenderingExamples/TimeSeriesExample.cpp",
        "examples/RenderingExamples/TimeSeriesFontData.cpp",
        "examples/RenderingExamples/TinyRendererSetup.cpp",
        "examples/RenderingExamples/TinyVRGui.cpp",
        "examples/RigidBody/KinematicRigidBodyExample.cpp",
        "examples/RigidBody/RigidBodySoftContact.cpp",
        "examples/RobotSimulator/b3RobotSimulatorClientAPI.cpp",
        "examples/RoboticsLearning/GripperGraspExample.cpp",
        "examples/RoboticsLearning/KukaGraspExample.cpp",
        "examples/RoboticsLearning/R2D2GraspExample.cpp",
        "examples/RollingFrictionDemo/RollingFrictionDemo.cpp",
        "examples/SharedMemory/GraphicsClientExample.cpp",
        "examples/SharedMemory/GraphicsServerExample.cpp",
        "examples/SharedMemory/IKTrajectoryHelper.cpp",
        "examples/SharedMemory/InProcessMemory.cpp",
        "examples/SharedMemory/PhysicsClient.cpp",
        "examples/SharedMemory/PhysicsClientC_API.cpp",
        "examples/SharedMemory/PhysicsClientExample.cpp",
        "examples/SharedMemory/PhysicsClientSharedMemory.cpp",
        "examples/SharedMemory/PhysicsClientSharedMemory_C_API.cpp",
        "examples/SharedMemory/PhysicsDirect.cpp",
        "examples/SharedMemory/PhysicsDirectC_API.cpp",
        "examples/SharedMemory/PhysicsLoopBack.cpp",
        "examples/SharedMemory/PhysicsLoopBackC_API.cpp",
        "examples/SharedMemory/PhysicsServer.cpp",
        "examples/SharedMemory/PhysicsServerCommandProcessor.cpp",
        "examples/SharedMemory/PhysicsServerExample.cpp",
        "examples/SharedMemory/PhysicsServerExampleBullet2.cpp",
        "examples/SharedMemory/PhysicsServerSharedMemory.cpp",
        "examples/SharedMemory/PosixSharedMemory.cpp",
        "examples/SharedMemory/RemoteGUIHelper.cpp",
        "examples/SharedMemory/SharedMemoryInProcessPhysicsC_API.cpp",
        "examples/SharedMemory/Win32SharedMemory.cpp",
        "examples/SharedMemory/b3PluginManager.cpp",
        "examples/SharedMemory/b3RobotSimulatorClientAPI_NoDirect.cpp",
        "examples/SharedMemory/b3RobotSimulatorClientAPI_NoGUI.cpp",
        "examples/SharedMemory/plugins/collisionFilterPlugin/collisionFilterPlugin.cpp",
        "examples/SharedMemory/plugins/pdControlPlugin/pdControlPlugin.cpp",
        "examples/SharedMemory/plugins/tinyRendererPlugin/TinyRendererVisualShapeConverter.cpp",
        "examples/SharedMemory/plugins/tinyRendererPlugin/tinyRendererPlugin.cpp",
        "examples/ThirdPartyLibs/BussIK/Jacobian.cpp",
        "examples/ThirdPartyLibs/BussIK/LinearR2.cpp",
        "examples/ThirdPartyLibs/BussIK/LinearR3.cpp",
        "examples/ThirdPartyLibs/BussIK/LinearR4.cpp",
        "examples/ThirdPartyLibs/BussIK/MatrixRmn.cpp",
        "examples/ThirdPartyLibs/BussIK/Misc.cpp",
        "examples/ThirdPartyLibs/BussIK/Node.cpp",
        "examples/ThirdPartyLibs/BussIK/Tree.cpp",
        "examples/ThirdPartyLibs/BussIK/VectorRn.cpp",
        "examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp",
        "examples/TinyRenderer/TinyRenderer.cpp",
        "examples/TinyRenderer/geometry.cpp",
        "examples/TinyRenderer/model.cpp",
        "examples/TinyRenderer/our_gl.cpp",
        "examples/TinyRenderer/tgaimage.cpp",
        "examples/Tutorial/Dof6ConstraintTutorial.cpp",
        "examples/Tutorial/Tutorial.cpp",
        "examples/Vehicles/Hinge2Vehicle.cpp",
        "examples/VoronoiFracture/VoronoiFractureDemo.cpp",
        "examples/VoronoiFracture/btConvexConvexMprAlgorithm.cpp",
    ],
    copts = copts + [
        "-Wno-non-virtual-dtor",
        "-Wno-delete-non-virtual-dtor",
        "-Wno-unused-variable",
        "-Wno-missing-braces",
        "-Wno-format",
        "-Wno-format-security",
        "-Wno-c++11-narrowing",
        "-Wno-narrowing",
        "-Wframe-larger-than=534760",
        "-DGWEN_COMPILE_STATIC",
        "-DDONT_USE_GLUT",
        "-DGLEW_STATIC",
        "-DGLEW_INIT_OPENGL11_FUNCTIONS=1",
        "-DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1",
        "-DDYNAMIC_LOAD_X11_FUNCTIONS",
    ],
    data = glob(["data/*"]),
    features = ["-use_header_modules"],  # Incompatible with -fexception.
    includes = [
        "examples",
        "examples/CommonInterfaces",
        "examples/ExampleBrowser",
        "examples/SharedMemory",
        "examples/ThirdPartyLibs/Glew",
        "examples/ThirdPartyLibs/optionalX11",
    ],
    linkopts = ["-ldl"],
    deps = [
        ":BulletGwen_DynamicLoadAll",
        ":ExampleBrowserLibDoublePrecision_DynamicLoadAll",
        ":OpenGLWindow_DynamicLoadAll",
        ":enet",
        ":loose_headers",
        "@stblib",
    ],
)

cc_library(
    name = "BulletUtils",
    srcs = [
        "examples/Utils/ChromeTraceUtil.cpp",
        "examples/Utils/RobotLoggingUtil.cpp",
        "examples/Utils/b3Clock.cpp",
        "examples/Utils/b3ResourcePath.cpp",
    ],
    hdrs = [
        "examples/Utils/RobotLoggingUtil.h",
        "examples/Utils/b3Clock.h",
        "examples/Utils/b3ERPCFMHelper.hpp",
        "examples/Utils/b3ReferenceFrameHelper.hpp",
        "examples/Utils/b3ResourcePath.h",
    ],
    copts = copts + [
        "-Wno-non-virtual-dtor",
        "-Wno-unused-variable",
        "-Wno-missing-braces",
        "-Wno-format",
        "-Wno-format-security",
        "-Wno-c++11-narrowing",
        "-Wno-narrowing",
        "-Wframe-larger-than=131184",
        "-DGWEN_COMPILE_STATIC",
        "-DDONT_USE_GLUT",
        "-DHAS_SOCKLEN_T",
    ],
    features = ["-use_header_modules"],  # Incompatible with -fexception.
    includes = [
        "examples",
        "examples/CommonInterfaces",
        "examples/ExampleBrowser",
        "examples/ThirdPartyLibs",
        "examples/ThirdPartyLibs/enet/include",
    ],
    deps = [
        ":LinearMath",
        ":loose_headers",
    ],
)

cc_library(
    name = "BulletUtilsDoublePrecision",
    srcs = [
        "examples/Utils/ChromeTraceUtil.cpp",
        "examples/Utils/RobotLoggingUtil.cpp",
        "examples/Utils/b3Clock.cpp",
        "examples/Utils/b3ResourcePath.cpp",
    ],
    hdrs = [
        "examples/Utils/RobotLoggingUtil.h",
        "examples/Utils/b3Clock.h",
        "examples/Utils/b3ERPCFMHelper.hpp",
        "examples/Utils/b3ReferenceFrameHelper.hpp",
        "examples/Utils/b3ResourcePath.h",
    ],
    copts = copts + [
        "-Wno-non-virtual-dtor",
        "-Wno-unused-variable",
        "-Wno-missing-braces",
        "-Wno-format",
        "-Wno-format-security",
        "-Wno-c++11-narrowing",
        "-Wno-narrowing",
        "-Wframe-larger-than=131184",
        "-DGWEN_COMPILE_STATIC",
        "-DDONT_USE_GLUT",
        "-DHAS_SOCKLEN_T",
    ],
    features = ["-use_header_modules"],  # Incompatible with -fexception.
    includes = [
        "examples",
        "examples/CommonInterfaces",
        "examples/ExampleBrowser",
        "examples/ThirdPartyLibs",
        "examples/ThirdPartyLibs/enet/include",
    ],
    deps = [
        ":LinearMathDoublePrecision",
        ":loose_headers",
    ],
)

cc_library(
    name = "enet",
    srcs = glob(
        [
            "examples/ThirdPartyLibs/enet/*.c",
        ],
    ),
    copts = c_only_copts + [
        "-DHAS_SOCKLEN_T",
    ],
    features = ["-use_header_modules"],  # Incompatible with -fexception.
    includes = [
        "examples/ThirdPartyLibs/enet/include",
    ],
    deps = [":loose_headers"],
)

cc_library(
    name = "base64",
    srcs = glob(
        [
            "examples/ThirdPartyLibs/cpp_base64/base64.cpp",
        ],
    ),
    hdrs = ["examples/ThirdPartyLibs/cpp_base64/include/cpp_base64/base64.h"],
    features = ["-use_header_modules"],  # Incompatible with -fexception.
    includes = [
        "examples/ThirdPartyLibs/cpp_base64/include",
        "examples/ThirdPartyLibs/cpp_base64/include/cpp_base64",
    ],
)

cc_library(
    name = "crossguid",
    srcs =
        [
            "examples/ThirdPartyLibs/crossguid/guid.cpp",
        ],
    hdrs = ["examples/ThirdPartyLibs/crossguid/crossguid/guid.hpp"],
    defines = ["GUID_LIBUUID"],
    features = ["-use_header_modules"],  # Incompatible with -fexception.
    includes = [
        "examples/ThirdPartyLibs/crossguid",
        "examples/ThirdPartyLibs/crossguid/crossguid",
    ],
    deps = ["//third_party/e2fsprogs:uuid"],
)

cc_library(
    name = "BulletInverseDynamicsUtils",
    srcs = glob([
        "Extras/InverseDynamics/*.cpp",
    ]),
    hdrs = glob([
        "Extras/InverseDynamics/*.hpp",
    ]),
    copts = [
    ],
    includes = [
    ],
    deps = [
        ":Bullet3Common",
        ":BulletCollision",
        ":BulletDynamics",
        ":BulletInverseDynamics",
        ":LinearMath",
        ":loose_headers",
    ],
)

cc_library(
    name = "BulletInverseDynamicsUtilsDoublePrecision",
    srcs = glob([
        "Extras/InverseDynamics/*.cpp",
    ]),
    hdrs = glob([
        "Extras/InverseDynamics/*.hpp",
    ]),
    copts = copts,
    features = ["-use_header_modules"],  # Incompatible with -fexception.
    includes = [
    ],
    deps = [
        ":Bullet3Common",
        ":BulletCollisionDoublePrecision",
        ":BulletDynamicsDoublePrecision",
        ":BulletInverseDynamicsDoublePrecision",
        ":LinearMathDoublePrecision",
        ":loose_headers",
    ],
)

cc_library(
    name = "BulletInverseDynamicsUtilsEigen",
    srcs = glob(
        [
            "Extras/InverseDynamics/*.cpp",
        ],
        exclude = [
            "Extras/InverseDynamics/invdyn_bullet_comparison.cpp",
        ],
    ),
    hdrs = glob([
        "Extras/InverseDynamics/*.hpp",
    ]),
    defines = [
        "BT_CUSTOM_INVERSE_DYNAMICS_CONFIG_H=IDConfigEigen.hpp",
    ],
    includes = [
    ],
    deps = [
        ":BulletDynamicsDoublePrecision",
        ":BulletInverseDynamicsEigen",
        ":LinearMathDoublePrecision",
        ":loose_headers",
        "//third_party/eigen3",
    ],
)

cc_library(
    name = "BulletInverseDynamicsUtilsBuiltin",
    srcs = [
        "Extras/InverseDynamics/CloneTreeCreator.cpp",
        "Extras/InverseDynamics/CoilCreator.cpp",
        "Extras/InverseDynamics/DillCreator.cpp",
        "Extras/InverseDynamics/IDRandomUtil.cpp",
        "Extras/InverseDynamics/MultiBodyNameMap.cpp",
        "Extras/InverseDynamics/MultiBodyTreeCreator.cpp",
        "Extras/InverseDynamics/MultiBodyTreeDebugGraph.cpp",
        "Extras/InverseDynamics/RandomTreeCreator.cpp",
        "Extras/InverseDynamics/SimpleTreeCreator.cpp",
        "Extras/InverseDynamics/User2InternalIndex.cpp",
    ],
    hdrs = glob([
        "Extras/InverseDynamics/*.hpp",
    ]),
    defines = [
        "BT_CUSTOM_INVERSE_DYNAMICS_CONFIG_H=IDConfigBuiltin.hpp",
    ],
    includes = [
    ],
    deps = [
        ":BulletDynamics",
        ":BulletInverseDynamicsBuiltin",
        ":LinearMath",
        ":loose_headers",
    ],
)

cc_library(
    name = "BulletGwen",
    srcs = glob(
        [
            "examples/ThirdPartyLibs/Gwen/*.cpp",
            "examples/ThirdPartyLibs/Gwen/**/*.cpp",
        ],
        exclude = [
            "examples/ThirdPartyLibs/Gwen/Renderers/OpenGL_DebugFont.cpp",
        ],
    ),
    hdrs = glob([
        "examples/ThirdPartyLibs/Gwen/*.h",
        "examples/ThirdPartyLibs/Gwen/**/*.h",
    ]),
    copts = [
        "-Wno-tautological-constant-out-of-range-compare",
        "-Wno-c++11-narrowing",
        "-Wno-narrowing",
        "-Wframe-larger-than=16416",
    ],
    includes = [
        "examples",
        "examples/ThirdPartyLibs",
    ],
    deps = [
        "@third_party//gl/native:GL",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "BulletGwen_DynamicLoadAll",
    srcs = glob(
        [
            "examples/ThirdPartyLibs/Gwen/*.cpp",
            "examples/ThirdPartyLibs/Gwen/**/*.cpp",
        ],
        exclude = [
            "examples/ThirdPartyLibs/Gwen/Renderers/OpenGL_DebugFont.cpp",
        ],
    ),
    hdrs = glob([
        "examples/ThirdPartyLibs/Gwen/*.h",
        "examples/ThirdPartyLibs/Gwen/**/*.h",
    ]),
    copts = [
        "-Wno-tautological-constant-out-of-range-compare",
        "-Wno-c++11-narrowing",
        "-Wno-narrowing",
        "-Wframe-larger-than=16416",
        "-DGWEN_COMPILE_STATIC",
        "-DDONT_USE_GLUT",
        "-DGLEW_STATIC",
        "-DGLEW_INIT_OPENGL11_FUNCTIONS=1",
        "-DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1",
        "-DDYNAMIC_LOAD_X11_FUNCTIONS",
    ],
    includes = [
        "examples",
        "examples/ThirdPartyLibs",
    ],
    deps = [
        "@com_google_absl//absl/strings",
    ],
)



# Temporary library. Please do not use it in your projects.
cc_library(
    name = "temporary_shape_converter",
    srcs = ["examples/ExampleBrowser/CollisionShape2TriangleMesh.cpp"],
    hdrs = [
        "examples/ExampleBrowser/CollisionShape2TriangleMesh.h",
        "examples/OpenGLWindow/GLInstanceGraphicsShape.h",
    ],
    copts = copts,
    features = ["-use_header_modules"],  # Incompatible with -fexception.
    deps = [
        ":Bullet3Common",
        ":BulletCollision",
    ],
)

cc_library(
    name = "BulletRobotics_NoGUI",
    srcs = [
        "examples/SharedMemory/PhysicsClientSharedMemory_C_API.cpp",
        "examples/SharedMemory/b3RobotSimulatorClientAPI_NoGUI.cpp",
    ],
    hdrs = [
        "examples/SharedMemory/SharedMemoryPublic.h",
        "examples/SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h",
        "examples/SharedMemory/b3RobotSimulatorClientAPI_NoGUI.h",
    ],
    deps = [
        ":PhysicsServerDirect",
    ],
)

cc_library(
    name = "BulletRoboticsDoublePrecision_NoGUI",
    srcs = [
        "examples/SharedMemory/PhysicsClientSharedMemory_C_API.cpp",
        "examples/SharedMemory/b3RobotSimulatorClientAPI_NoGUI.cpp",
    ],
    hdrs = [
        "examples/SharedMemory/SharedMemoryPublic.h",
        "examples/SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h",
        "examples/SharedMemory/b3RobotSimulatorClientAPI_NoGUI.h",
    ],
    deps = [
        ":PhysicsServerDirectDoublePrecision",
    ],
)

cc_library(
    name = "BulletRobotics",
    srcs = [
        "examples/RobotSimulator/b3RobotSimulatorClientAPI.cpp",
    ],
    hdrs = [
        "examples/RobotSimulator/b3RobotSimulatorClientAPI.h",
        "examples/SharedMemory/b3RobotSimulatorClientAPI_InternalData.h",
    ],
    deps = [
        ":BulletRobotics_NoGUI",
        ":PhysicsServerClientGUI_DynamicLoadAll",
    ],
)

cc_library(
    name = "BulletRoboticsDoublePrecision",
    srcs = [
        "examples/RobotSimulator/b3RobotSimulatorClientAPI.cpp",
    ],
    hdrs = [
        "examples/RobotSimulator/b3RobotSimulatorClientAPI.h",
        "examples/SharedMemory/SharedMemoryPublic.h",
        "examples/SharedMemory/b3RobotSimulatorClientAPI_InternalData.h",
    ],
    deps = [
        ":BulletRoboticsDoublePrecision_NoGUI",
        ":PhysicsServerClientGUIDoublePrecisionGRPC_DynamicLoadAll",
    ],
)

# For a more maintainable build this target should not exist and the headers
# should  be split into the existing cc_library targets, but this change was
# automatically  done so that we can remove long standing issues and complexity
# in the build system. It's up to the OWNERS of this package to get rid of it or
# not. The use of the textual_hdrs attribute is discouraged, use hdrs instead.
# Here it is used to avoid header parsing errors in packages where the feature
# parse_headers was enabled since loose headers were not being parsed. See
# (broken link) for more details.
cc_library(
    name = "loose_headers",
    compatible_with = [
        "//buildenv/target:all",
        "//buildenv/target:non_prod",
    ],
    tags = ["avoid_dep"],
    textual_hdrs = [
        "Extras/Serialize/BulletFileLoader/autogenerated/bullet.h",
        "Extras/Serialize/BulletWorldImporter/btBulletWorldImporter.h",
        "Extras/Serialize/BulletWorldImporter/btMultiBodyWorldImporter.h",
        "Extras/Serialize/BulletWorldImporter/btWorldImporter.h",
        "src/BulletCollision/BroadphaseCollision/btBroadphaseProxy.h",
        "src/BulletCollision/CollisionDispatch/btCollisionObject.h",
        "src/BulletCollision/NarrowPhaseCollision/btManifoldPoint.h",
        "src/BulletDynamics/Dynamics/btRigidBody.h",
        "src/BulletDynamics/MLCPSolvers/btLemkeAlgorithm.h",
        "src/BulletDynamics/MLCPSolvers/btLemkeSolver.h",
        "src/BulletDynamics/MLCPSolvers/btMLCPSolver.h",
        "src/BulletDynamics/MLCPSolvers/btMLCPSolverInterface.h",
        "src/BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h",
        "src/BulletInverseDynamics/IDConfig.hpp",
        "src/BulletInverseDynamics/IDConfigBuiltin.hpp",
        "src/BulletInverseDynamics/IDConfigEigen.hpp",
        "src/BulletInverseDynamics/IDErrorMessages.hpp",
        "src/BulletInverseDynamics/details/IDEigenInterface.hpp",
        "src/BulletInverseDynamics/details/IDLinearMathInterface.hpp",
        "src/BulletInverseDynamics/details/IDMatVec.hpp",
        "src/LinearMath/TaskScheduler/btThreadSupportInterface.h",
    ] + glob([
        "examples/**/*.h",
        "examples/**/*.inl",
        "src/Bullet3Collision/**/*.h",
        "src/Bullet3Common/**/*.h",
        "src/BulletDynamics/ConstraintSolver/*.h",
        "examples/SharedMemory/plugins/stablePDPlugin/*.h",
        "examples/OpenGLWindow/*.h",
        "examples/Importers/ImportURDFDemo/*.h",
        "Extras/Serialize/BulletFileLoader/*.h",
        "examples/ThirdPartyLibs/enet/include/enet/*.h",
        "src/LinearMath/*.h",
        "examples/ExampleBrowser/*.h",
        "examples/CommonInterfaces/*.h",
        "examples/ExampleBrowser/GwenGUISupport/*.h",
        "examples/ThirdPartyLibs/BussIK/*.h",
    ]),
    visibility = [":__pkg__"],
)
