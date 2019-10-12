// PropagationPlanner - classes to handled different methods of sound propagation
// Author - Nic Taylor
#include "PropagationPlanner.h"
#include "PropagationPlannerAStar.h"
#include "RoomGeometry.h"
#include "nComplex.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <array>
#include <assert.h>

std::shared_ptr<PropagationPlanner> PropagationPlanner::MakePlanner(const SoundPropagation::MethodType method)
{
    std::shared_ptr<PropagationPlanner> planner = nullptr;
    switch (method)
    {
    case SoundPropagation::Method_DirectLOS:
        planner = std::make_shared<PlannerDirectLOS>();
        break;
    case SoundPropagation::Method_RayCasts:
        planner = std::make_shared<PlannerRayCasts>();
        break;
    case SoundPropagation::Method_Pathfinding:
        planner = std::make_shared<PlannerAStar>();
        break;
    case SoundPropagation::Method_Wave:
        planner = std::make_shared<PlannerWave>();
        break;
    case SoundPropagation::Method_PlaneWave:
        planner = std::make_shared<PlannerWave>(PlannerWave(PlannerWave::Wave_Plane));        
        break;
    case SoundPropagation::Method_LOSAStarFallback:
        planner = std::make_shared<PlannerLOSAStar>();
        break;
    case SoundPropagation::Method_GridEmitter:
        planner = std::make_shared<PlannerGridEmitter>();
        break;
    default:
        break;
    }

    return planner;
}

void PlannerDirectLOS::Preprocess(std::shared_ptr<const RoomGeometry> _room)
{
    room = _room;
}

void PlannerDirectLOS::Plan(const PropagationPlanner::SourceConfig& _config)
{
    source = _config.source;
}

void PlannerDirectLOS::Simulate(PropagationResult& result, const nMath::Vector& _receiver, const float _time_ms) const
{
    (void)_time_ms;

    const nMath::LineSegment los = nMath::LineSegment{ source, _receiver };
    if (result.config == SoundPropagation::PRD_FULL)
    {
        result.intersections.clear();
        result.intersections.push_back(los);
    }

    if (room->Intersects(los))
    {
        result.gain = 0.f;
        return;
    }

    const float distance = nMath::Max(FLT_EPSILON, nMath::Length(source - _receiver));
    const float geometric_attenuation = nMath::Min(1.f, 1.f / distance);
    result.gain = geometric_attenuation;
}

void PlannerRayCasts::Plan(const PropagationPlanner::SourceConfig& _config)
{
    source = _config.source;
}

void PlannerRayCasts::Preprocess(std::shared_ptr<const RoomGeometry> _room)
{
    room = _room;
}

bool PlannerRayCasts::Intersects(PropagationResult& result, const nMath::Vector& start, const nMath::Vector& end) const
{
    const nMath::LineSegment ls{ start, end };
    if (result.config == SoundPropagation::PRD_FULL)
    {
        result.intersections.push_back(ls);
    }

    return room->Intersects(ls);
}

void PlannerRayCasts::Simulate(PropagationResult& result, const nMath::Vector& _receiver, const float _time_ms) const
{
    (void)_time_ms;

    nMath::Vector direction = source - _receiver;
    float distance = nMath::Length(direction);
    if (result.config == SoundPropagation::PRD_FULL)
    {
        result.intersections.clear();
    }
    if (Intersects(result, source, _receiver))
    {
        if (distance > FLT_EPSILON)
        {
            direction = direction / distance;
            nMath::Vector ray_orth = { -direction.y, direction.x, 0.f };
            nMath::Vector ray_orth_inv = -1.f * ray_orth;
            if ((!Intersects(result, ray_orth + _receiver, _receiver) &&
                !Intersects(result, ray_orth + _receiver, source)) ||
                (!Intersects(result, ray_orth_inv + _receiver, _receiver) &&
                    !Intersects(result, ray_orth_inv + _receiver, source)))
            {
                distance += 1.f;
            }
            else if ((!Intersects(result, ray_orth + source, source) &&
                !Intersects(result, ray_orth + source, _receiver)) ||
                (!Intersects(result, ray_orth_inv + source, source) &&
                    !Intersects(result, ray_orth_inv + source, _receiver)))
            {
                distance += 1.f;
            }
            else
            {
                result.gain = 0.f;
                return;
            }
        }
        else
        {
            result.gain = 0.f;
            return;
        }
    }

    const float geometric_attenuation = nMath::Min(1.f, 1.f / nMath::Max(FLT_EPSILON, distance));
    result.gain = geometric_attenuation;
}

void PlannerWave::Plan(const PropagationPlanner::SourceConfig& _config)
{
    source = _config.source;
    frequency = _config.frequency;
    time_factor = 1.f / _config.time_scale;

    first_reflections.clear();
    auto& walls = room->Walls();
    first_reflections.reserve(walls.size());
    for (const nMath::LineSegment& wall : walls)
    {
        nMath::Vector&& reflection = nMath::Project(wall, source);
        first_reflections.emplace_back(reflection);
    }
}

void PlannerWave::Preprocess(std::shared_ptr<const RoomGeometry> _room)
{
    room = _room;
}

void PlannerWave::Simulate(PropagationResult& result, const nMath::Vector& _receiver, const float _time_ms) const
{
    const nMath::LineSegment los = nMath::LineSegment{ source, _receiver };
    if (result.config == SoundPropagation::PRD_FULL)
    {
        result.intersections.clear();
        result.intersections.push_back(los);
    }
    if (room->Intersects(los))
    {
        result.gain = 0.f;
        return;
    }

    const static float inv_speed = 1 / 340.f;

    float value = 0;
    float norm_value = 0.f;
    const float distance = solution_type == Wave_FreeSpace ? nMath::Length(_receiver - source) :
                            (_receiver.x - source.x);
    const float angle = 2.f * (float)M_PI * frequency; // omega
    const float time_scaled = _time_ms * time_factor / 1000.f;

    if (result.config != SoundPropagation::PRD_REFLECTIONS_ONLY)
    {
        const float geometric_attenuation = nMath::Min(1.f, 1.f / distance);
        //const float shift = angle * distance / 340.f; // k * distance
        //float value = ga * cosf(shift - angle * _time_ms)); 
        norm_value = geometric_attenuation;
        const float wave = cosf(angle * (time_scaled - distance * inv_speed));
        value = wave; // TODO: Normalization on gfx side
        if (solution_type == Wave_FreeSpace)
        {
            value *= geometric_attenuation;
        }
        result.absolute = wave;
        result.wave_id = 0;
        //value = geometric_attenuation * cosf(angle * (distance * inv_speed - time_scaled)); // TODO: Normalization on gfx side
    }
    int wave_id = 0;
    for (const nMath::Vector& v : first_reflections)
    {
        ++wave_id;
        const float first_distance = nMath::Length(_receiver - v);
        const float reflect_phase = nMath::Length(source - v);
        //const float first_geometric_attenuation = nMath::Min(1.f, 1.f / (first_distance));
        const float first_geometric_attenuation = nMath::Min(1.f, 1.f / (first_distance + reflect_phase));
        //float first_value = ga_f * cosf(pi/2 + first_shift - angle * _time_ms);
        const float wave = sinf(angle * ((first_distance + reflect_phase) * inv_speed - time_scaled));
        float first_value = first_geometric_attenuation * wave;
        if (wave > result.absolute)
        {
            result.absolute = wave;
            result.wave_id = wave_id;
        }
        //float first_value = first_geometric_attenuation * cosf(angle * (first_distance * inv_speed - time_scaled)); // TODO: Normalization on gfx side
        norm_value += first_geometric_attenuation;
        if (solution_type == Wave_FreeSpace)
        {
            value += first_value;
        }
        else
        {
            value += wave;
        }
    }

    // time independent
    if (result.config == SoundPropagation::PRD_REFLECTIONS_ONLY)
    {
        float real_sum = 0.f;
        float im_sum = 0.f;

        const float theta = angle * inv_speed * distance;
        const float geometric_attenuation = nMath::Min(1.f, 1.f / distance);
        real_sum += cosf(theta) * geometric_attenuation;
        im_sum += sinf(theta) * geometric_attenuation;

        for (const nMath::Vector& v : first_reflections)
        {
            const float reflect_phase = nMath::Length(source - v);
            const float first_distance = nMath::Length(_receiver - v) + reflect_phase;
            const float first_geometric_attenuation = nMath::Min(1.f, 1.f / first_distance);
            const float first_theta = (float)M_PI_2 + angle * inv_speed * first_distance;

            real_sum += cosf(first_theta) * first_geometric_attenuation;
            im_sum += sinf(first_theta) * first_geometric_attenuation;
        }
        
        float amplitude = sqrtf(real_sum * real_sum + im_sum * im_sum);
        float phase = atan2f(im_sum, real_sum);
        float gain2 = cosf(phase - angle * time_scaled) * amplitude;
        //float volume_threshold_96db = 1.58489319e-5f * (1.f + (float)first_reflections.size()) * 2.f;
        //if ((fabsf(gain2) > volume_threshold_96db || fabsf(value) > volume_threshold_96db) &&
        //    fabsf(20.f*log10f(gain2) - 20.f*log10f(value)) > 0.5f)
        {
            result.gain = fabsf(gain2); // 0.9f * 0.5f * (gain2 + norm_value);
            return;
        }
    }

    //const float normalize = 0.5f;
    const float gain = (solution_type == Wave_FreeSpace) ? 0.9f : 
        (0.9f * 1.f/sqrt((float)first_reflections.size() + 1.f));
    result.gain = fabsf(value) * gain;
    result.magnitude = value;
    //result.gain = normalize * gain * (norm_value + value);
}

namespace
{    
    nMath::ComplexExp SteadyState(
        const nMath::Vector& observer,
        const float frequency,
        const std::vector<nMath::Vector> sources)
    {
        const static float inv_speed = 1 / 340.f;
        const float angle = 2.f * (float)M_PI * frequency; // omega
        const float wave_numeber = angle * inv_speed;
        float real_sum = 0.f;
        float im_sum = 0.f;

        for (const nMath::Vector& source : sources)
        {
            const float distance = nMath::Length(observer - source);
            const float phase = distance * wave_numeber;
            const float inv_distance = nMath::Min(1.f, 1.f / distance);

            real_sum += cosf(phase) * inv_distance;
            im_sum += sinf(phase) * inv_distance;
        }

        float amplitude = sqrtf(real_sum * real_sum + im_sum * im_sum);
        float phase = atan2f(im_sum, real_sum);
        return nMath::ComplexExp{ amplitude, phase };
    }
}

template<class PlannerPrimary, class PlannerSecondary, SoundPropagation::MethodType Method>
PlannerTwoStages<PlannerPrimary, PlannerSecondary, Method>::PlannerTwoStages()
{
    planner_primary = std::make_shared<PlannerPrimary>();
    planner_secondary = std::make_shared<PlannerSecondary>();
}

template<class PlannerPrimary, class PlannerSecondary, SoundPropagation::MethodType Method>
void PlannerTwoStages<PlannerPrimary, PlannerSecondary, Method>::Preprocess(std::shared_ptr<const RoomGeometry> _room)
{
    planner_primary->Preprocess(_room);
    planner_secondary->Preprocess(_room);
}

template<class PlannerPrimary, class PlannerSecondary, SoundPropagation::MethodType Method>
void PlannerTwoStages<PlannerPrimary, PlannerSecondary, Method>::Plan(const PropagationPlanner::SourceConfig& _config)
{
    planner_primary->Plan(_config);
    planner_secondary->Plan(_config);
}

template<class PlannerPrimary, class PlannerSecondary, SoundPropagation::MethodType Method>
void PlannerTwoStages<PlannerPrimary, PlannerSecondary, Method>::Simulate(PropagationResult& result,
    const nMath::Vector& _receiver, const float _time_ms) const
{
    planner_primary->Simulate(result, _receiver, _time_ms);
    if (result.gain <= 0.f)
    {
        planner_secondary->Simulate(result, _receiver, _time_ms);
    }
}

// Grid Emmiter
namespace BookChapterCode {

    using nMath::Vector;
    using nMath::Max;
    using nMath::Length;

    struct Sphere {
        Vector center;
        float radius;
    };

    struct AttenuatedPosition {
        bool audible;
        Vector position;
        float spread;
        Vector closest_voxel_center; // For debugging
    };

    // Input is the direction to any voxel center.
    bool PointInsideVoxel(const Vector& direction,
        const float cell_extent) {
        return fabsf(direction.x) < cell_extent &&
            fabsf(direction.y) < cell_extent &&
            fabsf(direction.z) < cell_extent;
    }

    // Use the distance to the axis-aligned voxel to interpolate from 0 to 1.
    float GetNearFieldInterpolation(const Vector& direction,
        const float voxel_extent) {
        const float near_field_range = 1.5f;
        const Vector point_on_voxel = {
            Max(0.f, fabsf(direction.x) - voxel_extent),
            Max(0.f, fabsf(direction.y) - voxel_extent),
            Max(0.f, fabsf(direction.z) - voxel_extent)
        };
        const float dist_to_voxel = Length(point_on_voxel);
        if (dist_to_voxel >= near_field_range) {
            return 0.f;
        }
        return (near_field_range - dist_to_voxel) / near_field_range;
    }

    template<typename VoxelContainer>
    AttenuatedPosition VoxelsToAttenuatedPosition(const VoxelContainer& voxels,
        const Sphere& receiver, const float voxel_extent) {

        const float attenuation_range = receiver.radius;

        float total_weight = 0.f;
        float total_weight_ratio = 0.f;
        Vector total_direction = { 0.0, 0.0, 0.0 };

        float closest_distance = attenuation_range;
        Vector closest_voxel_direction, closest_voxel_center =
        { 0.0, 0.0, 0.0 };

        float z_values[] = { 0.5, 3 };
        int z_value = 0;
        for (const Vector voxel_center : voxels) {
            const Vector direction = voxel_center - receiver.center - Vector{ 0.0f, 0.0f, z_values[z_value++] };
            if (PointInsideVoxel(direction, voxel_extent)) {
                total_direction = { 0.0, 0.0, 0.0 };
                closest_distance = 0.f;
                closest_voxel_center = voxel_center;
                break;
            }
            const float distance = Length(direction);
            if (distance < attenuation_range)
            {
                if (distance < closest_distance) {
                    closest_distance = distance;
                    closest_voxel_direction = direction;
                    closest_voxel_center = voxel_center;
                }
                const float weight = attenuation_range - distance;
                total_weight_ratio += (weight / distance);
                total_direction += (weight / distance) * direction;
                total_weight += weight;
            }
        }

        Vector emitter_position = receiver.center;
        float spread = 0.f;
        const float total_dir_length = Length(total_direction);
        if (total_dir_length <= FLT_EPSILON) {
            spread = 1.f;
            emitter_position += closest_voxel_center;
        }
        else if (total_weight > FLT_EPSILON) {
            spread = 1.f - total_dir_length / total_weight;
            const float near_field_lerp =
                GetNearFieldInterpolation(closest_voxel_direction,
                    voxel_extent);
            if (near_field_lerp > 0.f) {
                spread += (1.f - spread) * near_field_lerp;
            }
            emitter_position +=
                closest_distance * total_direction / total_dir_length;
        }

        AttenuatedPosition result;
        result.audible = closest_distance < attenuation_range;
        result.position = emitter_position;
        result.spread = spread;
        result.closest_voxel_center = closest_voxel_center;
        return result;
    }

    constexpr int kNumSpeakers = 8;
    struct SpeakerData {
        float angle = 0;
        float total_weight;
    };

    struct VirtualSpeakerSet {
        std::array<SpeakerData, kNumSpeakers> speakers;
        Vector closest_voxel_center = { 0.f, 0.f, 0.f };
    };

    template<typename VoxelContainer>
    VirtualSpeakerSet VoxelsToVirtualSpeakers(const VoxelContainer& voxels,
        const Sphere& receiver, const float voxel_extent) {

        VirtualSpeakerSet speaker_set;
        std::array<SpeakerData, kNumSpeakers>& speakers = speaker_set.speakers;

        float angle_dist = 2 * M_PI / kNumSpeakers;
        int speaker_id = 0;
        for (auto& speaker : speakers) {
            speaker.angle = angle_dist * speaker_id++;
            speaker.total_weight = 0.f;
        }

        const float attenuation_range = receiver.radius;

        float closest_distance = attenuation_range;
        Vector& closest_voxel_center = speaker_set.closest_voxel_center;

        for (const Vector voxel_center : voxels) {
            const Vector direction = voxel_center - receiver.center;
            if (PointInsideVoxel(direction, voxel_extent)) {
                closest_distance = 0.f;
                closest_voxel_center = voxel_center;
                for (auto& speaker : speakers) {
                    speaker.total_weight = 1.f / kNumSpeakers;
                }
                break;
            }
            const float distance = Length(direction);
            if (distance < attenuation_range)
            {
                if (distance < closest_distance) {
                    closest_distance = distance;
                    closest_voxel_center = voxel_center;
                }
                const float weight = attenuation_range - distance;
                const float angle = atan2f(direction.y, direction.x);
                auto it_rhs = std::upper_bound(speakers.begin(), speakers.end(), angle,
                    [](const float _angle, const SpeakerData& lhs) {
                    return _angle < lhs.angle;
                });
                auto it_lhs = it_rhs - 1;                
                if (it_rhs == speakers.end()) {
                    it_rhs = speakers.begin();                    
                }
                float rhs_angle = it_rhs->angle;
                if (rhs_angle < it_lhs->angle) {
                    rhs_angle += 2.f * (float)M_PI;
                }
                const float angle_lerp = (angle - it_lhs->angle) / (rhs_angle - it_lhs->angle);
                it_lhs->total_weight += weight * (1.f - angle_lerp);
                it_rhs->total_weight += weight * angle_lerp;
            }
        }
        return speaker_set;
    }

    float GainFromPosition(const VirtualSpeakerSet speaker_set) {
        return Max(0.f, (20.f - Length(speaker_set.closest_voxel_center)) / 20.f);
    }
    
    std::array<float, 5> VirtualSpeakerSetToSpeakerArrangment(const VirtualSpeakerSet speaker_set,
        const std::array<float, 5> speaker_angles) {
        std::array<float, 5> speaker_gains{};        
        float total_weight = 0.f;
        for (const auto& virtual_speaker : speaker_set.speakers) {
            const float weight = virtual_speaker.total_weight;
            if (weight <= FLT_EPSILON) {
                continue;
            }
            float virtual_angle = virtual_speaker.angle;
            auto it_rhs = std::upper_bound(speaker_angles.begin(), speaker_angles.end(),
                virtual_angle);
            auto it_lhs = (it_rhs != speaker_angles.begin() ? it_rhs : speaker_angles.end()) - 1;
            if (it_rhs == speaker_angles.end()) {
                it_rhs = speaker_angles.begin();
            }
            float rhs_angle = *it_rhs;
            if (rhs_angle < *it_lhs) {
                rhs_angle += 2.f * (float)M_PI;
                if (virtual_angle < *it_lhs) {
                    virtual_angle += 2.f * (float)M_PI;
                }
            }
            const float angle_lerp = (virtual_angle - *it_lhs) / (rhs_angle - *it_lhs);
            speaker_gains[it_lhs - speaker_angles.begin()] +=
                weight * (1.f - angle_lerp);
            speaker_gains[it_rhs - speaker_angles.begin()] +=
                weight * angle_lerp;
            total_weight += weight;
        }
        // Normalize
        if (total_weight > FLT_EPSILON) {
            const float rms = GainFromPosition(speaker_set);
            for (float& gain : speaker_gains) {
                gain = rms * sqrtf(gain / total_weight);
            }
        }
        return speaker_gains;
    }
}
constexpr float kVoxelExtent = 0.5f / (float)GridEmitter::GridCellsPerMeter;

struct GridEmitterIterator {
    const GridEmitter::GeometryGrid& grid;
    nMath::Vector current;
    int x, y;
    int begin_x, begin_y;

    GridEmitterIterator(const GridEmitter::GeometryGrid& _grid) : grid(_grid) {
        x = -1; y = 0;
        // Find beginning
        HasNext();
        begin_x = x; begin_y = y;
    }

    const nMath::Vector& Get() const {
        return current;
    }

    GridEmitterIterator end() const {
        GridEmitterIterator end_it(*this);
        end_it.y = GridEmitter::GridResolution;
        return end_it;
    }

    GridEmitterIterator begin() const {
        GridEmitterIterator begin_it(*this);
        // reset
        begin_it.x = begin_x;
        begin_it.y = begin_y;
        return begin_it;
    }

    const nMath::Vector& operator*() const {
        return current;
    }

    bool operator==(const GridEmitterIterator& other) const {
        return (other.x == x || y == GridEmitter::GridResolution) && other.y == y && &other.grid == &grid;
    }

    bool operator!=(const GridEmitterIterator& other) const {
        return !(*this == other);
    }

    GridEmitterIterator operator++() {
        HasNext();
        return *this;
    }

    bool Increment() {
        ++x;
        if (x == GridEmitter::GridResolution) {
            x = 0;
            ++y;
        }
        return y < GridEmitter::GridResolution;
    }

    bool HasNext() {
        while (Increment()) {
            // Talk about this logic and multiple grids/chunks
            if (grid[y][x]) {
                break;
            }
        }
        if (y >= GridEmitter::GridResolution) {
            return false;
        }
        // Update
        //const float z = 1.f;
        const float z = -0.5f;
        const float half_grid_cell_size = 0.5f / (float)GridEmitter::GridCellsPerMeter;
        current = { half_grid_cell_size + x / (float)GridEmitter::GridCellsPerMeter,
            half_grid_cell_size + y / (float)GridEmitter::GridCellsPerMeter,
            half_grid_cell_size + z / (float)GridEmitter::GridCellsPerMeter };
        current.x -= GridEmitter::GridDistance / 2.f;
        current.y -= GridEmitter::GridDistance / 2.f;
        return true;
    }
};

void PlannerGridEmitter::Preprocess(std::shared_ptr<const RoomGeometry> _room) {
    _room;
}

void PlannerGridEmitter::Plan(const PropagationPlanner::SourceConfig& _config) {
    grid_emitter = _config.grid_emitter;
    near_field_mode = _config.near_field_mode;
    weight_function = _config.grid_emitter_weight_function;
}

void PlannerGridEmitter::Simulate(PropagationResult& result, const nMath::Vector& _receiver, const float _time_ms) const {
    _time_ms;
    const float attenuation_range = 20.f;
    const float near_field = 1.5f;
    assert(grid_emitter != nullptr);
    if (grid_emitter == nullptr) {        
        return;
    }
    const GridEmitter::GeometryGrid& grid = grid_emitter->Grid();

    const float half_grid_cell_size = 0.5f / (float)GridEmitter::GridCellsPerMeter;    

    float total_weight = 0.f;
    float spread = 0.f;
    float closest_distance = FLT_MAX;
    nMath::Vector closest_grid_dir = { 0.0, 0.0, 0.0 };
    nMath::Vector closest_grid_pos = { 0.0, 0.0, 0.0 };
    nMath::Vector total_dir = { 0.0, 0.0, 0.0 };
    nMath::Vector emitter_direction = { 0.0, 0.0, 0.0 };

    for (int x = 0; x < GridEmitter::GridResolution; ++x) {
        for (int y = 0; y < GridEmitter::GridResolution; ++y) {
            if (grid[y][x]) {
                nMath::Vector grid_cell_center = { half_grid_cell_size + x / (float)GridEmitter::GridCellsPerMeter,
                    half_grid_cell_size + y / (float)GridEmitter::GridCellsPerMeter, 0.f };
                grid_cell_center.x -= GridEmitter::GridDistance / 2.f;
                grid_cell_center.y -= GridEmitter::GridDistance / 2.f;
                const nMath::Vector direction = grid_cell_center - _receiver;
                if (weight_function != SoundPropagation::GEWF_DistantOnly &&
                    fabs(direction.x) < half_grid_cell_size &&
                    fabs(direction.y) < half_grid_cell_size &&
                    fabs(direction.z) < half_grid_cell_size)
                {
                    spread = 1.0;
                    emitter_direction = { 0.001f, 0.0, 0.0 };
                    closest_distance = 0.f;
                    closest_grid_pos = grid_cell_center;
                    break;
                }
                float distance = nMath::Length(direction);
                if (weight_function == SoundPropagation::GEWF_DistantOnly) {
                    static const float kInnerAttenuation = 5.f;
                    if (distance < kInnerAttenuation) {
                        distance = attenuation_range * (1 - distance / kInnerAttenuation);
                    }
                }
                if (distance < attenuation_range)
                {
                    if (distance < closest_distance) {
                        closest_distance = distance;
                        closest_grid_dir = direction;
                        closest_grid_pos = grid_cell_center;
                    }
                    float weight = attenuation_range - distance;
                    switch (weight_function) {
                    case SoundPropagation::GEWF_Squared:
                        weight *= weight;
                        break;
                    case SoundPropagation::GEWF_DistantOnly:
                        break;
                    }
                    total_dir += (weight / distance) * direction;
                    total_weight += weight;
                }
            }
        }
    }

    if (total_weight > 0.f && emitter_direction == nMath::Vector{ 0.f,0.f,0.f }) {
        const float total_dir_length = nMath::Length(total_dir);
        if (total_dir_length <= FLT_EPSILON) {
            spread = 1.f;
            emitter_direction = nMath::Vector{ closest_distance, 0.f, 0.f };
        }
        else {
            spread = 1.f - total_dir_length / total_weight;
            switch (near_field_mode) {
            case SoundPropagation::NFM_L_Infinite:
            {
                const float near_field_range = nMath::Max(fabsf(closest_grid_dir.x), fabsf(closest_grid_dir.y));
                if (near_field_range < near_field + half_grid_cell_size) {
                    const float near_field_lerp = nMath::Max(0.f, near_field - (near_field_range - half_grid_cell_size)) / near_field;
                    spread += nMath::Min(1.f, (1.f - spread) * near_field_lerp);
                }
            }
            break;
            case SoundPropagation::NFM_L2:
                {
                    const nMath::Vector closest_to_grid = {nMath::Max(0.f, fabsf(closest_grid_dir.x) - half_grid_cell_size),
                        nMath::Max(0.f, fabsf(closest_grid_dir.y) - half_grid_cell_size), 0.f};
                    const float dist_to_cell = nMath::Length(closest_to_grid);
                    if (dist_to_cell < near_field) {
                        const float near_field_lerp = (near_field - dist_to_cell)/ near_field;
                        spread += (1.f - spread) * near_field_lerp;
                    }
                }
                break;
            }
            emitter_direction = closest_distance * total_dir / total_dir_length;
        }
    }
    emitter_direction += _receiver;
    result.emitter_direction = emitter_direction;
    result.spread = spread;
    result.closest_point = closest_grid_pos;    

    const auto receiver = BookChapterCode::Sphere{ _receiver, attenuation_range };
    {
        GridEmitterIterator iterator{ grid };
        const auto& bc = BookChapterCode::VoxelsToAttenuatedPosition<GridEmitterIterator>(iterator,
            receiver, kVoxelExtent);
        if (bc.position != result.emitter_direction || bc.spread != result.spread) {
            if (bc.position.x != FLT_MAX) {
                result.emitter_direction = bc.position;
            }
            result.spread = bc.spread;
        }
        result.gain = nMath::Max(0.f, (attenuation_range - closest_distance) / attenuation_range);
    }

    {
        GridEmitterIterator iterator{ grid };
        const auto speaker_set = BookChapterCode::VoxelsToVirtualSpeakers(iterator, receiver, kVoxelExtent);
        const auto speaker_arrangement = BookChapterCode::VirtualSpeakerSetToSpeakerArrangment(speaker_set,
        std::array<float, 5>{ M_PI / 3, M_PI_2, 2.f*M_PI / 3.f, 7.f*M_PI / 6.f, 11.f*M_PI / 6 });
    }    
}
