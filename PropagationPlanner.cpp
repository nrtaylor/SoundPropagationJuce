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
        planner = std::make_shared<PlannerTwoStages<PlannerDirectLOS, PlannerAStar> >();
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

template<class PlannerPrimary, class PlannerSecondary>
PlannerTwoStages<PlannerPrimary, PlannerSecondary>::PlannerTwoStages()
{
    planner_primary = std::make_shared<PlannerPrimary>();
    planner_secondary = std::make_shared<PlannerSecondary>();
}

template<class PlannerPrimary, class PlannerSecondary>
void PlannerTwoStages<PlannerPrimary, PlannerSecondary>::Preprocess(std::shared_ptr<const RoomGeometry> _room)
{
    planner_primary->Preprocess(_room);
    planner_secondary->Preprocess(_room);
}

template<class PlannerPrimary, class PlannerSecondary>
void PlannerTwoStages<PlannerPrimary, PlannerSecondary>::Plan(const PropagationPlanner::SourceConfig& _config)
{
    planner_primary->Plan(_config);
    planner_secondary->Plan(_config);
}

template<class PlannerPrimary, class PlannerSecondary>
void PlannerTwoStages<PlannerPrimary, PlannerSecondary>::Simulate(PropagationResult& result, const nMath::Vector& _receiver, const float _time_ms) const
{
    planner_primary->Simulate(result, _receiver, _time_ms);
    if (result.gain <= 0.f)
    {
        planner_secondary->Simulate(result, _receiver, _time_ms);
    }
}

// Grid Emmiter
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

    float total_weight = 0.0;
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
                if (fabs(direction.x) < half_grid_cell_size &&
                    fabs(direction.y) < half_grid_cell_size &&
                    fabs(direction.z) < half_grid_cell_size)
                {
                    spread = 1.0;
                    emitter_direction = { 0.001f, 0.0, 0.0 };
                    closest_distance = 0.f;
                    closest_grid_pos = grid_cell_center;
                    break;
                }
                const float distance = nMath::Length(direction);
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
                        // TODO: distant cells only
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
            emitter_direction = closest_distance * total_dir / nMath::Length(total_dir);
        }
    }
    emitter_direction += _receiver;
    result.emitter_direction = emitter_direction;
    result.spread = spread;
    result.closest_point = closest_grid_pos;
    result.gain = nMath::Max(0.f, (attenuation_range - closest_distance) / attenuation_range);
}