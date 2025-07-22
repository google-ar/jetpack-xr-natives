// Copyright 2024 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "core/common/debug_draw.h"

#include <sys/types.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <memory>
#include <set>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/base/thread_annotations.h"
#include "absl/hash/hash.h"
#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/synchronization/mutex.h"
// Generated to pack-in material data.
#include "absl/types/span.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Box.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/IndexBuffer.h"
#include "filament/filament/include/filament/RenderableManager.h"
#include "filament/filament/include/filament/TransformManager.h"
#include "filament/filament/include/filament/VertexBuffer.h"
#include "filament/libs/filabridge/include/filament/MaterialEnums.h"
#include "filament/libs/math/include/math/TVecHelpers.h"
#include "filament/libs/math/include/math/vec4.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "filament/libs/utils/include/utils/EntityManager.h"
#include "core/common/debug_draw_resources.h"
#include "core/common/filament_engine_helpers.h"
#include "core/common/filament_helpers.h"
#include "core/common/resource_helpers.h"
#include "core/geometry/shapes/rect.h"
#include "core/math/vec.h"
#include "core/model/mesh/mesh_description.h"
#include "core/model/mesh/mesh_index_data.h"
#include "core/model/mesh/mesh_vertex_and_index_data.h"
#include "core/model/mesh/mesh_vertex_data.h"
#include "core/model/mesh/vertex_format.h"
#include "robin_map/include/tsl/robin_map.h"

namespace imp {
namespace debug_draw {

using ::filament::math::float3;
using ::filament::math::ubyte4;

namespace {

static constexpr float3 kBoxFractions[] = {
    {0, 0, 1},  // 0. left bottom far
    {1, 0, 1},  // 1. right bottom far
    {0, 1, 1},  // 2. left top far
    {1, 1, 1},  // 3. right top far
    {0, 0, 0},  // 4. left bottom near
    {1, 0, 0},  // 5. right bottom near
    {0, 1, 0},  // 6. left top near
    {1, 1, 0},  // 7. right top near
};

static constexpr uint16_t kBoxLineIndices[] = {
    0, 1, 1, 3, 3, 2, 2, 0,  // far
    4, 5, 5, 7, 7, 6, 6, 4,  // near
    0, 4, 1, 5, 3, 7, 2, 6,
};

static constexpr uint32_t kBoxFaceIndices[] = {
    2, 0, 1, 2, 1, 3,  // far
    6, 4, 5, 6, 5, 7,  // near
    2, 0, 4, 2, 4, 6,  // left
    3, 1, 5, 3, 5, 7,  // right
    0, 4, 5, 0, 5, 1,  // bottom
    2, 6, 7, 2, 7, 3,  // top
};

static constexpr uint16_t kQuadLineIndices[] = {
    0, 1, 1, 2, 2, 3, 3, 0,
};

constexpr float kMinBoundingBoxExtents = 0.001f;

constexpr int kSphereStackCount = 6;
constexpr int kSphereSectorCount = 10;
constexpr int kCapsuleStackCount = 6;  // must be even
constexpr int kCapsuleSectorCount = 10;
constexpr int kCylinderSectorCount = 10;
constexpr int kConeSectorCount = 10;

filament::Box CalculateBounds(const PositionBuffer& positions) {
  filament::Box bb = NilBounds();
  for (const auto& position : positions) {
    bb.unionSelf(filament::Box{position, float3{kMinBoundingBoxExtents}});
  }
  return bb;
}

class Details final {
 public:
  explicit Details(filament::Engine* engine, filament::Scene* scene,
                   filament::Material* material,
                   filament::Material* screenspace_material)
      : engine_(engine),
        scene_(scene),
        material_(material),
        screenspace_material_(screenspace_material) {
    if (material_ == nullptr || screenspace_material_ == nullptr) {
      RegisterPackagedResources(debug_draw_resources_create());
    }

    if (material_ == nullptr) {
      absl::StatusOr<filament::Material*> packaged_material =
          LoadPackagedMaterial(engine_, "debug_draw_unlit.cmat");
      
      material_ = *packaged_material;
      assert(material_ != nullptr);
    }
    material_instance_ = material_->createInstance();
    assert(material_instance_ != nullptr);

    if (screenspace_material_ == nullptr) {
      absl::StatusOr<filament::Material*> packaged_material =
          LoadPackagedMaterial(engine_, "debug_draw_unlit_screenspace.cmat");
      
      screenspace_material_ = *packaged_material;
      assert(screenspace_material_ != nullptr);
    }
    screenspace_material_instance_ = screenspace_material_->createInstance();
    assert(screenspace_material_instance_ != nullptr);
  }

  ~Details() {
    filament::RenderableManager& rm = engine_->getRenderableManager();
    for (const auto& it : active_instances_) {
      scene_->remove(it.entity);
      rm.destroy(it.entity);
    }
    engine_->destroy(screenspace_material_instance_);
    engine_->destroy(material_instance_);
    engine_->destroy(material_);
  }

  void Advance() {
    ++current_frame_number_;

    EraseExpiredRenderables();
    CreateNewlySubmittedRenderables();
  }

  // Submit buffer builders associated with a given entity, appending them to
  // the buffer builder list associated with that entity and having the same
  // frame duration.
  void Submit(utils::Entity entity, uint32_t duration_frames,
              std::vector<Geometry> geometry_snippets,
              VertexSpace vertex_space = VertexSpace::kModelView) {
    duration_frames = std::max(duration_frames, 1u);
    const ParentInstance parent_instance{entity, duration_frames, vertex_space};

    // This could happen on any thread and needs to be guarded against Advance
    // swapping the buffer builder map for processing.
    absl::MutexLock lock(&lock_);

    GeometrySnippetsMap& geometry_snippets_map =
        geometry_snippets_maps_[active_geometry_snippets_map_];
    if (geometry_snippets_map.count(parent_instance)) {
      auto& existing_geometry_snippets =
          geometry_snippets_map[parent_instance].geometries;
      existing_geometry_snippets.insert(existing_geometry_snippets.end(),
                                        geometry_snippets.begin(),
                                        geometry_snippets.end());
      geometry_snippets_map[parent_instance].vertex_space = vertex_space;
    } else {
      geometry_snippets_map.emplace(
          parent_instance,
          GeometrySnippets{std::move(geometry_snippets), vertex_space});
    }
  }

 private:
  void EraseExpiredRenderables() {
    utils::EntityManager& em = utils::EntityManager::get();
    filament::TransformManager& tm = engine_->getTransformManager();
    filament::RenderableManager& rm = engine_->getRenderableManager();

    // Iterate over everything that expired before the current frame.
    for (auto it = active_instances_.begin();
         it != active_instances_.upper_bound(
                   {utils::Entity{}, current_frame_number_});
         ++it) {
      // Fully remove from Filament.
      scene_->remove(it->entity);
      rm.destroy(it->entity);
      tm.destroy(it->entity);
      em.destroy(it->entity);
      engine_->destroy(it->vertex_buffer);
      for (filament::IndexBuffer* ib : it->index_buffers) {
        engine_->destroy(ib);
      }
    }

    active_instances_.erase(active_instances_.begin(),
                            active_instances_.lower_bound(
                                {utils::Entity{}, current_frame_number_}));
  }

  void CreateNewlySubmittedRenderables() {
    // Swap active vertex buffer builder map.
    uint32_t map_index_to_process;
    {
      absl::MutexLock lock(&lock_);
      map_index_to_process = active_geometry_snippets_map_;
      active_geometry_snippets_map_ ^= 1u;
    }

    // Create and activate new entities.  geometry_snippets_maps_ has already
    // coalesced geometry snippets into vectors associated with a single entity
    // and expiration time in the future.
    for (auto geometry_snippets_map :
         geometry_snippets_maps_[map_index_to_process]) {
      GeometrySnippets& geometry_snippets = geometry_snippets_map.second;
      tsl::robin_map<filament::backend::PrimitiveType, IndexBuffer>
          coalesced_index_buffers_map;
      PositionBuffer coalesced_positions;
      ColorBuffer coalesced_colors;

      // Coalesce position, vertex, and index buffers into a single buffer.
      absl::c_for_each(
          geometry_snippets.geometries,
          [&coalesced_index_buffers_map, &coalesced_positions,
           &coalesced_colors](Geometry geometry_snippet) {
            auto result = coalesced_index_buffers_map.emplace(
                geometry_snippet.type, IndexBuffer{});
            const uint32_t initial_index = coalesced_positions.size();
            absl::c_move(geometry_snippet.positions,
                         std::back_inserter(coalesced_positions));
            absl::c_move(geometry_snippet.colors,
                         std::back_inserter(coalesced_colors));
            absl::c_transform(geometry_snippet.indices,
                              std::back_inserter(result.first.value()),
                              [initial_index](const uint16_t index) {
                                return index + initial_index;
                              });
          });

      // Calculate the bounding box of all vertices being added, and find the
      // number of renderables which will be between 0 and 3 inclusive,
      // depending on how many types of geometry were added.
      uint32_t renderable_count = coalesced_index_buffers_map.size();

      // Build the submeshes and submit them to the renderable manager.
      if (renderable_count > 0) {
        // Create the new instance and add it to the scene and parent transform
        // if specified.
        Instance instance{CreateChildEntity(geometry_snippets_map.first.entity),
                          current_frame_number_ +
                              geometry_snippets_map.first.duration_frames};

        filament::Box bb = CalculateBounds(coalesced_positions);

        filament::RenderableManager::Builder builder{renderable_count};
        builder.receiveShadows(false);
        builder.castShadows(false);

        instance.vertex_buffer = CreateFilamentVertexBuffer(
            std::move(coalesced_positions), std::move(coalesced_colors));

        uint32_t renderable_index = 0;
        for (auto it = coalesced_index_buffers_map.begin();
             it != coalesced_index_buffers_map.end(); ++it) {
          filament::IndexBuffer* index_buffer =
              CreateFilamentIndexBuffer(std::move(it.value()));

          builder.geometry(renderable_index, it.key(), instance.vertex_buffer,
                           index_buffer);

          if (geometry_snippets.vertex_space == VertexSpace::kModelView) {
            builder.boundingBox(bb).priority(7);
            builder.material(renderable_index, material_instance_);
          } else if (geometry_snippets.vertex_space == VertexSpace::kScreen) {
            builder.material(renderable_index, screenspace_material_instance_);
            builder.culling(false);
          } else {
            IMP_LOG(imp::FATAL) << "Geometry's vertex space must be either "
                          "VertexSpace::kModelView or VertexSpace::kScreen";
          }

          instance.index_buffers.emplace_back(index_buffer);
          ++renderable_index;
        }

        builder.build(*engine_, instance.entity);

        // Add the current instance to the active set so it will be tracked and
        // cleaned up once it has expired.
        active_instances_.emplace(instance);
      }
    }

    // Clear the just-processed map for reuse.
    geometry_snippets_maps_[map_index_to_process].clear();
  }

  // Creates an entity in the current scene with transform component and
  // parented to parent_entity, or unparented if parent_entity has no transform
  // component.  Returned entity should be removed from the scene, have its
  // transform destroyed (filament::TransformManager::destroy), and be destroyed
  // by the entity manager (filament::EntityManager::destroy).
  utils::Entity CreateChildEntity(utils::Entity parent_entity) {
    utils::EntityManager& em = utils::EntityManager::get();
    filament::TransformManager& tm = engine_->getTransformManager();

    utils::Entity entity = em.create();
    scene_->addEntity(entity);

    if (tm.hasComponent(parent_entity)) {
      tm.create(entity, tm.getInstance(parent_entity));
    } else {
      tm.create(entity);
    }

    return entity;
  }

  // Creates and returns a filament vertex buffer.  Returned vertex buffer
  // should be cleaned up with filament::Engine::Destroy.
  filament::VertexBuffer* CreateFilamentVertexBuffer(PositionBuffer positions,
                                                     ColorBuffer colors) {
    const int vertex_count = positions.size();

    constexpr uint8_t kPositionBufferIndex = 0u;
    constexpr uint8_t kColorBufferIndex = 1u;

    filament::VertexBuffer* vertex_buffer =
        filament::VertexBuffer::Builder()
            .vertexCount(vertex_count)
            .bufferCount(/*bufferCount=*/2)
            .attribute(filament::VertexAttribute::POSITION,
                       kPositionBufferIndex,
                       filament::backend::ElementType::FLOAT3)
            .attribute(filament::VertexAttribute::COLOR, kColorBufferIndex,
                       filament::backend::ElementType::UBYTE4)
            .normalized(filament::VertexAttribute::COLOR)
            .build(*engine_);

    auto positions_pointer =
        std::make_unique<PositionBuffer>(std::move(positions));
    filament::VertexBuffer::BufferDescriptor position_buffer{
        positions_pointer->data(), vertex_count * sizeof(float3),
        // Callback to cleanup the raw buffer when it is no longer needed by
        // Filament.
        [](void* buffer, size_t size, void* user) {
          std::unique_ptr<PositionBuffer>{static_cast<PositionBuffer*>(user)};
        },
        static_cast<void*>(positions_pointer.release())};
    vertex_buffer->setBufferAt(*engine_, kPositionBufferIndex,
                               std::move(position_buffer));

    auto colors_pointer = std::make_unique<ColorBuffer>(std::move(colors));
    filament::VertexBuffer::BufferDescriptor color_buffer{
        colors_pointer->data(), vertex_count * sizeof(ubyte4),
        // Callback to cleanup the raw buffer when it is no longer needed by
        // Filament.
        [](void* buffer, size_t size, void* user) {
          std::unique_ptr<ColorBuffer>{static_cast<ColorBuffer*>(user)};
        },
        static_cast<void*>(colors_pointer.release())};
    vertex_buffer->setBufferAt(*engine_, kColorBufferIndex,
                               std::move(color_buffer));

    return vertex_buffer;
  }

  // Creates and returns a filament index buffer.  Returned index buffer
  // should be cleaned up with filament::Engine::Destroy.
  filament::IndexBuffer* CreateFilamentIndexBuffer(IndexBuffer indices) {
    auto indices_pointer = std::make_unique<IndexBuffer>(std::move(indices));
    filament::IndexBuffer* ib =
        filament::IndexBuffer::Builder()
            .indexCount(indices_pointer->size())
            .bufferType(filament::IndexBuffer::IndexType::USHORT)
            .build(*engine_);

    filament::IndexBuffer::BufferDescriptor ibDescriptor{
        indices_pointer->data(), indices_pointer->size() * sizeof(uint16_t),
        // Callback to cleanup the raw buffer when it is no longer needed by
        // Filament.
        [](void* buffer, size_t size, void* user) {
          std::unique_ptr<IndexBuffer>{static_cast<IndexBuffer*>(user)};
        },
        static_cast<void*>(indices_pointer.release())};
    ib->setBuffer(*engine_, std::move(ibDescriptor));

    return ib;
  }

  // ParentInstance tracks the parent entity and duration associated with a
  // given draw call.  We use this to coalesce debug geometry into a single
  // renderable per entity with a given duration each frame.
  struct ParentInstance {
    utils::Entity entity;
    uint32_t duration_frames;
    VertexSpace vertex_space;

    constexpr bool operator==(const ParentInstance& rhs) const {
      return entity == rhs.entity && duration_frames == rhs.duration_frames &&
             vertex_space == rhs.vertex_space;
    }

    template <typename H>
    friend H AbslHashValue(H h, const ParentInstance& i) {
      return H::combine(std::move(h), utils::Entity::Hasher()(i.entity),
                        i.duration_frames, i.vertex_space);
    }
  };

  struct ParentInstanceHash {
    size_t operator()(ParentInstance const& instance) const {
      return absl::Hash<ParentInstance>()(instance);
    }
  };

  // An instance of debug geometry that is currently active and being rendered.
  // We keep this in a set so that we can easily remove geometry as it expires.
  struct Instance {
    utils::Entity entity;
    uint32_t expiration_frame_number;
    filament::VertexBuffer* vertex_buffer;
    std::vector<filament::IndexBuffer*> index_buffers;

    constexpr bool operator<(const Instance& rhs) const {
      return expiration_frame_number < rhs.expiration_frame_number ||
             ((expiration_frame_number == rhs.expiration_frame_number) &&
              (entity < rhs.entity));
    }
  };

  // Persistent Filament objects.
  filament::Engine* engine_;
  filament::Scene* scene_;
  filament::Material* material_;
  filament::MaterialInstance* material_instance_;
  filament::Material* screenspace_material_;
  filament::MaterialInstance* screenspace_material_instance_;

  // Frame number for calculating when expiration should occur
  uint32_t current_frame_number_ = 0u;

  struct GeometrySnippets {
    std::vector<Geometry> geometries;
    VertexSpace vertex_space;
  };

  // These maps are filled out when local entity activations go out of scope,
  // and processed during Advance.  We swap back and forth between the two
  // vertex_buffer_builder_maps_ so debug geometry submission doesn't block on
  // processing.
  using GeometrySnippetsMap =
      tsl::robin_map<ParentInstance, GeometrySnippets, ParentInstanceHash>;
  GeometrySnippetsMap geometry_snippets_maps_[2];
  absl::Mutex lock_;
  uint32_t active_geometry_snippets_map_ ABSL_GUARDED_BY(lock_) = 0u;

  // The set of currently active instances which is ordered ascending by
  // expiration time.
  std::set<Instance> active_instances_;
};

// Gets the pointer to the global instance that keeps the debug rendering state
// so that users do not have to pass contexts around in order to use debug
// drawing.
Details** GetDetailsInstanceCreator() {
  static Details* instance;
  return &instance;
}

Details* GetDetailsInstancePointer() { return *GetDetailsInstanceCreator(); }

}  // namespace

float2 ShiftPointOrthogonally(float2 prev, float2 point, float2 next,
                              float distance) {
  // Normalize the segments.
  float2 in_v = SafeNormalized((point - prev));
  float2 out_v = SafeNormalized((next - point));
  // Compute segment lengths.
  float in_length = length2(in_v);
  float out_length = length2(out_v);
  // Handle the edge-cases.
  if (in_length == 0 && out_length == 0) {
    // All three points overlap, no way to apply offset.
    return point;
  } else if (in_length == 0) {
    in_v = out_v;
  } else if (out_length == 0) {
    out_v = in_v;
  }
  // Note that in_v and out_v are already normalized.
  const imp::float2 bisector = (Orthogonal(in_v) + Orthogonal(out_v)) * 0.5f;
  // cross_product == |in_v| * |bisector| * cos(theta), where theta is the
  // angle from in_v to bisector counter-clockwise.
  float cross_product = Cross2D(in_v, bisector);
  imp::float2 shift_v = bisector * (distance / cross_product);
  float double_abs_offset = std::abs(distance) * 2;
  // Cap the shift length at 2 * abs(distance) just in case.
  if (length(shift_v) > double_abs_offset) {
    shift_v = SafeNormalized(shift_v) * double_abs_offset;
  }

  return point + shift_v;
}

std::vector<float2> ShiftPointsOrthogonally(const std::vector<float2>& points,
                                            float distance) {
  std::vector<float2> shifted_points;

  int num_points = points.size();
  for (int i = 0; i < num_points; i++) {
    int prev_index = i == 0 ? i : i - 1;
    int next_index = i >= num_points - 1 ? i : i + 1;

    float2 prev = points[prev_index];
    float2 point = points[i];
    float2 next = points[next_index];

    float2 new_pos = ShiftPointOrthogonally(prev, point, next, distance);
    shifted_points.push_back(new_pos);
  }
  return shifted_points;
}

std::vector<float2> GenerateArc(float2 start, float2 end, bool invert) {
  int arc_sector_count = 5;
  std::vector<float2> points;
  const float sector_angle = M_PI / arc_sector_count;
  const float radius = std::sqrt((end.x - start.x) * (end.x - start.x) +
                                 (end.y - start.y) * (end.y - start.y)) /
                       2.0;
  const float2 midpoint((start.x + end.x) / 2.0, (start.y + end.y) / 2.0);
  const float starting_angle = atan2((end.x - start.x), (end.y - start.y));
  for (int i = 0; i < arc_sector_count + 1; ++i) {
    const float angle = starting_angle + sector_angle * i * (invert ? -1 : 1);
    float2 point =
        midpoint + float2{radius * std::sin(angle), radius * std::cos(angle)};
    points.push_back(point);
  }
  return points;
}

std::vector<float2> GenerateOutline2D(const std::vector<float2>& path,
                                      float width) {
  // Obtains the points forming an extruded line by calculating points
  // orthogonally offset from the original path.
  std::vector<float2> shifted_up = ShiftPointsOrthogonally(path, width * 0.5f);
  std::vector<float2> shifted_down =
      ShiftPointsOrthogonally(path, -1.0 * width * 0.5f);
  // Obtains the arcs around the start and end.
  std::vector<float2> start_arc =
      GenerateArc(shifted_up[0], shifted_down[0], /* invert= */ true);
  std::vector<float2> end_arc =
      GenerateArc(shifted_up[shifted_up.size() - 1],
                  shifted_down[shifted_down.size() - 1], /* invert= */ false);

  std::vector<float2> outline;
  outline.insert(outline.end(), shifted_up.begin(), shifted_up.end());
  outline.insert(outline.end(), end_arc.rbegin(), end_arc.rend());
  outline.insert(outline.end(), shifted_down.rbegin(), shifted_down.rend());
  outline.insert(outline.end(), start_arc.begin(), start_arc.end());

  return outline;
}

std::vector<float2> To2DPath(const std::vector<float3>& path_3d) {
  std::vector<float2> path_2d;
  path_2d.reserve(path_3d.size());
  for (int i = 0; i < path_3d.size(); ++i) {
    path_2d.push_back(path_3d.at(i).xy);
  }
  return path_2d;
}

Fixture::Fixture(filament::Engine* engine, filament::Scene* scene,
                 filament::Material* debug_material,
                 filament::Material* debug_screenspace_material) {
  Details** instance = GetDetailsInstanceCreator();
  assert(*instance == nullptr);
  *instance =
      new Details(engine, scene, debug_material, debug_screenspace_material);
}

Fixture::~Fixture() {
  Details** instance = GetDetailsInstanceCreator();
  assert(*instance != nullptr);
  delete *instance;
  *instance = nullptr;
}

void Fixture::Advance() {
  Details* instance = GetDetailsInstancePointer();
  assert(instance != nullptr);
  instance->Advance();
}

DrawSpace::DrawSpace(utils::Entity entity, uint32_t duration_frames,
                     VertexSpace vertex_space)
    : entity_(entity),
      duration_frames_(duration_frames),
      vertex_space_(vertex_space) {}

DrawSpace::~DrawSpace() {
  Details* instance = GetDetailsInstancePointer();
  assert(instance != nullptr);
  instance->Submit(entity_, duration_frames_, geometry_snippets_,
                   vertex_space_);
}

void DrawSpace::UserDefined(Geometry geometry) {
  assert(geometry.positions.size() == geometry.colors.size());
  assert(absl::c_all_of(geometry.indices, [&geometry](const uint16_t index) {
    return index < geometry.positions.size();
  }));

  geometry_snippets_.emplace_back(geometry);
}

void DrawSpace::Point(const float3& position, const Color& color) {
  Geometry geometry;
  geometry.type = filament::backend::PrimitiveType::POINTS;
  geometry.positions.emplace_back(position);
  geometry.colors.emplace_back(color);
  geometry.indices.emplace_back(0u);
  UserDefined(std::move(geometry));
}

void DrawSpace::Line(const float3& start, const float3& end,
                     const Color& color) {
  Geometry geometry;
  geometry.type = filament::backend::PrimitiveType::LINES;
  geometry.positions.emplace_back(start);
  geometry.colors.emplace_back(color);
  geometry.indices.emplace_back(0u);
  geometry.positions.emplace_back(end);
  geometry.colors.emplace_back(color);
  geometry.indices.emplace_back(1u);
  UserDefined(std::move(geometry));
}

void Local::BoxLines(const filament::Box& box, const Color& color) {
  Geometry geometry;
  const auto box_min = box.getMin();
  const auto box_max = box.getMax();
  const auto box_delta = box_max - box_min;
  geometry.type = filament::backend::PrimitiveType::LINES;
  for (auto v : kBoxFractions) {
    geometry.positions.emplace_back(box_min + v * box_delta);
    geometry.colors.emplace_back(color);
  }
  absl::c_copy(kBoxLineIndices, std::back_inserter(geometry.indices));
  UserDefined(std::move(geometry));
}

void Local::SphereLines(const filament::math::float3& center, float radius,
                        const Color& color) {
  Geometry geometry;
  geometry.type = filament::backend::PrimitiveType::LINES;

  const size_t vertex_count = (kSphereStackCount - 1) * kSphereSectorCount + 2;
  geometry.positions.resize(vertex_count);
  geometry.colors.resize(vertex_count);
  geometry.positions[0] = filament::math::float3{0, radius, 0} + center;
  const float stack_angle = M_PI / kSphereStackCount;
  const float sector_angle = 2.0 * M_PI / kSphereSectorCount;
  for (int i = 1; i < kSphereStackCount; ++i) {
    const float y = radius * std::cos(stack_angle * i);
    const float r = radius * std::sin(stack_angle * i);
    for (int j = 0; j < kSphereSectorCount; ++j) {
      geometry.positions[(i - 1) * kSphereSectorCount + j + 1] =
          filament::math::float3{r * std::sin(sector_angle * j), y,
                                 r * std::cos(sector_angle * j)} +
          center;
    }
  }
  const int last_vertex = (kSphereStackCount - 1) * kSphereSectorCount + 1;
  geometry.positions[last_vertex] =
      filament::math::float3{0, -radius, 0} + center;

  for (int i = 0; i < vertex_count; ++i) {
    geometry.colors[i] = color;
  }

  // Draw lines from the top to the first stack/ring
  for (int j = 0; j < kSphereSectorCount; ++j) {
    geometry.indices.emplace_back(0u);
    geometry.indices.emplace_back(j + 1);
  }

  for (int i = 0; i < kSphereStackCount - 2; ++i) {
    int ring_offset = i * kSphereSectorCount;
    for (int j = 1; j < kSphereSectorCount + 1; ++j) {
      // Draw the ring for that stack
      geometry.indices.emplace_back(ring_offset + j);
      geometry.indices.emplace_back(ring_offset + j % kSphereSectorCount + 1);
      // Draw the sector line for that stack
      geometry.indices.emplace_back(ring_offset + j);
      geometry.indices.emplace_back(ring_offset + kSphereSectorCount + j);
    }
  }

  int last_ring_offset = kSphereSectorCount * (kSphereStackCount - 2);
  for (int j = 1; j < kSphereSectorCount + 1; ++j) {
    // Draw the last ring
    geometry.indices.emplace_back(last_ring_offset + j);
    geometry.indices.emplace_back(last_ring_offset + j % kSphereSectorCount +
                                  1);
    // Draw lines from the last ring to the bottom
    geometry.indices.emplace_back(last_ring_offset + j);
    geometry.indices.emplace_back(last_vertex);
  }

  UserDefined(std::move(geometry));
}

void Local::BoxFaces(const filament::Box& box, const Color& color) {
  Geometry geometry;
  const auto box_min = box.getMin();
  const auto box_max = box.getMax();
  const auto box_delta = box_max - box_min;
  geometry.type = filament::backend::PrimitiveType::TRIANGLES;
  for (auto v : kBoxFractions) {
    geometry.positions.emplace_back(box_min + v * box_delta);
    geometry.colors.emplace_back(color);
  }
  absl::c_copy(kBoxFaceIndices, std::back_inserter(geometry.indices));
  UserDefined(std::move(geometry));
}

void Local::CapsuleLines(const filament::math::float3& center, float height,
                         float radius, const Color& color) {
  Geometry geometry;
  geometry.type = filament::backend::PrimitiveType::LINES;

  const size_t vertex_count = kCapsuleStackCount * kSphereSectorCount + 2;
  geometry.positions.resize(vertex_count);
  geometry.colors.resize(vertex_count);
  geometry.positions[0] =
      filament::math::float3{0, radius + height / 2.0f, 0} + center;
  const float stack_angle = M_PI / kCapsuleStackCount;
  const float sector_angle = 2.0 * M_PI / kCapsuleSectorCount;

  for (int i = 1; i <= kCapsuleStackCount / 2; i++) {
    const float y = radius * std::cos(stack_angle * i) + height / 2.0f;
    const float r = radius * std::sin(stack_angle * i);
    for (int j = 0; j < kSphereSectorCount; ++j) {
      geometry.positions[(i - 1) * kSphereSectorCount + j + 1] =
          filament::math::float3{r * std::sin(sector_angle * j), y,
                                 r * std::cos(sector_angle * j)} +
          center;
    }
  }
  for (int i = kCapsuleStackCount / 2; i < kCapsuleStackCount; i++) {
    const float y = -radius * std::cos(stack_angle * (kCapsuleStackCount - i)) -
                    height / 2.0f;
    const float r = radius * std::sin(stack_angle * (kCapsuleStackCount - i));
    for (int j = 0; j < kCapsuleSectorCount; ++j) {
      geometry.positions[i * kCapsuleSectorCount + j + 1] =
          filament::math::float3{r * std::sin(sector_angle * j), y,
                                 r * std::cos(sector_angle * j)} +
          center;
    }
  }
  const int last_vertex = kCapsuleStackCount * kCapsuleSectorCount + 1;
  geometry.positions[last_vertex] =
      filament::math::float3{0, -radius - height / 2.0f, 0} + center;

  for (int i = 0; i < vertex_count; ++i) {
    geometry.colors[i] = color;
  }

  // Draw lines from the top to the first stack/ring
  for (int j = 0; j < kCapsuleSectorCount; ++j) {
    geometry.indices.emplace_back(0u);
    geometry.indices.emplace_back(j + 1);
  }

  for (int i = 0; i < kCapsuleStackCount - 1; ++i) {
    int ring_offset = i * kCapsuleSectorCount;
    for (int j = 1; j < kCapsuleSectorCount + 1; ++j) {
      // Draw the ring for that stack
      geometry.indices.emplace_back(ring_offset + j);
      geometry.indices.emplace_back(ring_offset + j % kCapsuleSectorCount + 1);
      // Draw the sector line for that stack
      geometry.indices.emplace_back(ring_offset + j);
      geometry.indices.emplace_back(ring_offset + kCapsuleSectorCount + j);
    }
  }

  int last_ring_offset = kCapsuleSectorCount * (kCapsuleStackCount - 1);
  for (int j = 1; j < kSphereSectorCount + 1; ++j) {
    // Draw the last ring
    geometry.indices.emplace_back(last_ring_offset + j);
    geometry.indices.emplace_back(last_ring_offset + j % kSphereSectorCount +
                                  1);
    // Draw lines from the last ring to the bottom
    geometry.indices.emplace_back(last_ring_offset + j);
    geometry.indices.emplace_back(last_vertex);
  }

  UserDefined(std::move(geometry));
}

void Local::CylinderLines(const filament::math::float3& base, float radius,
                          float height, const Color& color) {
  Geometry geometry;
  geometry.type = filament::backend::PrimitiveType::LINES;

  // 2 rings
  const size_t vertex_count = 2 * kCylinderSectorCount;
  geometry.positions.resize(vertex_count);
  geometry.colors.resize(vertex_count, color);
  const float sector_angle = 2.0 * M_PI / kCylinderSectorCount;
  const int ring_offset = kCylinderSectorCount;

  for (int j = 0; j < kCylinderSectorCount; ++j) {
    // the 2 rings are height apart
    geometry.positions[j] = geometry.positions[ring_offset + j] =
        filament::math::float3{radius * std::sin(sector_angle * j), 0.0f,
                               radius * std::cos(sector_angle * j)} +
        base;
    geometry.positions[ring_offset + j].y += height;
  }

  for (int j = 0; j < kCylinderSectorCount; ++j) {
    // Draw the first ring
    geometry.indices.emplace_back(j);
    geometry.indices.emplace_back(j + 1);
  }
  geometry.indices.back() = 0;

  for (int j = 0; j < kCylinderSectorCount; ++j) {
    // Draw the second ring
    geometry.indices.emplace_back(ring_offset + j);
    geometry.indices.emplace_back(ring_offset + j + 1);
  }
  geometry.indices.back() = ring_offset;

  for (int j = 0; j < kCylinderSectorCount; ++j) {
    // Draw the lines between the two rings
    geometry.indices.emplace_back(j);
    geometry.indices.emplace_back(ring_offset + j);
  }

  UserDefined(std::move(geometry));
}

void Local::ConeLines(const filament::math::float3& base, float radius,
                      float height, const Color& color) {
  Geometry geometry;
  geometry.type = filament::backend::PrimitiveType::LINES;

  // base ring + tip
  const size_t vertex_count = kConeSectorCount + 1;
  geometry.positions.resize(vertex_count);
  geometry.colors.resize(vertex_count, color);
  const float sector_angle = 2.0 * M_PI / kConeSectorCount;
  const int tip_offset = kConeSectorCount;

  for (int j = 0; j < kConeSectorCount; ++j) {
    geometry.positions[j] =
        filament::math::float3{radius * std::sin(sector_angle * j), 0.0f,
                               radius * std::cos(sector_angle * j)} +
        base;
  }
  geometry.positions[tip_offset] = base + height * kUp;

  for (int j = 0; j < kConeSectorCount; ++j) {
    // Draw the base ring
    geometry.indices.emplace_back(j);
    geometry.indices.emplace_back(j + 1);
  }
  geometry.indices.back() = 0;

  for (int j = 0; j < kConeSectorCount; ++j) {
    // Draw the lines between the tip and the base ring
    geometry.indices.emplace_back(tip_offset);
    geometry.indices.emplace_back(j);
  }

  UserDefined(std::move(geometry));
}

void Local::MeshLines(absl::Span<const MeshVertexAndIndexData> mesh,
                      const Color& color) {
  for (auto primitive : mesh) {
    Geometry geometry;
    geometry.type = filament::backend::PrimitiveType::LINES;

    MeshVertexData* vertices = primitive.vertex_data;
    MeshIndexData* indices = primitive.index_data;

    size_t vertex_count = vertices->GetDescription().vertex_count;
    geometry.positions.resize(vertex_count);
    geometry.colors.resize(vertex_count);

    size_t index_count = indices->GetDescription().index_count;
    geometry.indices.resize(index_count);

    for (int i = 0; i < vertex_count; ++i) {
      geometry.positions[i] = vertices->VertexAttributeAt<float3>(
          i, VertexFormat::VertexAttribute::POSITION);
      geometry.colors[i] = color;
    }

    if (indices->GetDescription().index_type ==
        MeshDescription::IndexType::USHORT) {
      for (int i = 0; i < index_count; ++i) {
        geometry.indices[i] = indices->IndexAt<int16_t>(i);
      }
    } else {
      for (int i = 0; i < index_count; ++i) {
        geometry.indices[i] = indices->IndexAt<int32_t>(i);
      }
    }

    UserDefined(std::move(geometry));
  }
}

void NormalizedScreen::RectLines(const Rect& rect, const Color& color) {
  Geometry geometry;
  geometry.type = filament::backend::PrimitiveType::LINES;
  float2 upper_left = float2(rect.center.x - rect.half_extent.x,
                             rect.center.y + rect.half_extent.y);
  float2 lower_right = float2(rect.center.x + rect.half_extent.x,
                              rect.center.y - rect.half_extent.y);
  geometry.positions.emplace_back(float3(upper_left.x, upper_left.y, 0.0f));
  geometry.colors.emplace_back(color);
  geometry.positions.emplace_back(float3(lower_right.x, upper_left.y, 0.0f));
  geometry.colors.emplace_back(color);
  geometry.positions.emplace_back(float3(lower_right.x, lower_right.y, 0.0f));
  geometry.colors.emplace_back(color);
  geometry.positions.emplace_back(float3(upper_left.x, lower_right.y, 0.0f));
  geometry.colors.emplace_back(color);
  absl::c_copy(kQuadLineIndices, std::back_inserter(geometry.indices));
  UserDefined(std::move(geometry));
}

}  // namespace debug_draw
}  // namespace imp
