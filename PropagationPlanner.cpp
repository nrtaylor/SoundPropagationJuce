// PropagationPlanner - classes to handled different methods of sound propagation
// Author - Nic Taylor
#include "PropagationPlanner.h"
#include "PropagationPlannerAStar.h"
#include "RoomGeometry.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <array>

std::shared_ptr<PropagationPlanner> PropagationPlanner::MakePlanner(const SoundPropagation::MethodType method)
{
    std::shared_ptr<PropagationPlanner> planner = nullptr;
    switch (method)
    {
    case SoundPropagation::Method_SpecularLOS:
        planner = std::make_shared<PlannerSpecularLOS>();
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
    case SoundPropagation::Method_LOSAStarFallback:
        planner = std::make_shared<PlannerTwoStages<PlannerSpecularLOS, PlannerAStar> >();
        break;
    default:
        break;
    }

    return planner;
}

void PlannerSpecularLOS::Preprocess(std::shared_ptr<const RoomGeometry> _room)
{
    room = _room;
}

void PlannerSpecularLOS::Plan(const PropagationPlanner::SourceConfig& _config)
{
    source = _config.source;
}

void PlannerSpecularLOS::Simulate(PropagationResult& result, const nMath::Vector& _receiver, const float _time_ms) const
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
    const float distance = nMath::Length(_receiver - source);
    const float angle = 2.f * (float)M_PI * frequency; // omega
    const float time_scaled = _time_ms * time_factor / 1000.f;

    if (result.config != SoundPropagation::PRD_REFLECTIONS_ONLY)
    {
        const float geometric_attenuation = nMath::Min(1.f, 1.f / distance);
        //const float shift = angle * distance / 340.f; // k * distance
        //float value = ga * cosf(shift - angle * _time_ms)); 
        value = geometric_attenuation + geometric_attenuation * cosf(angle * (distance * inv_speed - time_scaled)); // TODO: Normalization on gfx side
        //value = geometric_attenuation * cosf(angle * (distance * inv_speed - time_scaled)); // TODO: Normalization on gfx side
    }
    for (const nMath::Vector& v : first_reflections)
    {
        const float first_distance = nMath::Length(_receiver - v);
        const float reflect_phase = nMath::Length(source - v);
        //const float first_geometric_attenuation = nMath::Min(1.f, 1.f / (first_distance));
        const float first_geometric_attenuation = nMath::Min(1.f, 1.f / (first_distance + reflect_phase));
        //float first_value = ga_f * cosf(pi/2 + first_shift - angle * _time_ms);
        float first_value = first_geometric_attenuation + first_geometric_attenuation * sinf(angle * ((first_distance + reflect_phase) * inv_speed - time_scaled)); // TODO: Normalization on gfx side
        //float first_value = first_geometric_attenuation * cosf(angle * (first_distance * inv_speed - time_scaled)); // TODO: Normalization on gfx side
        value += first_value;
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
            const float first_distance = nMath::Length(_receiver - v);
            const float first_geometric_attenuation = nMath::Min(1.f, 1.f / (first_distance + distance));
            const float first_theta = angle * inv_speed * first_distance;

            real_sum += cosf(first_theta) * first_geometric_attenuation;
            im_sum += sinf(first_theta) * first_geometric_attenuation;
        }
        
        float amplitude = sqrtf(real_sum * real_sum + im_sum * im_sum);
        float phase = atan2f(im_sum, real_sum);
        float gain2 = cosf(phase - angle * time_scaled) * amplitude + amplitude;
        float volume_threshold_96db = 1.58489319e-5f * (1.f + (float)first_reflections.size()) * 2.f;
        if ((fabsf(gain2) > volume_threshold_96db || fabsf(value) > volume_threshold_96db) &&
            fabsf(20.f*log10f(gain2) - 20.f*log10f(value)) > 0.5f)
        {
            result.gain = 0.9f * 0.5f * gain2;
            return;
        }
    }

    const float normalize = 0.5f;
    const float gain = 0.9f;
    result.gain = normalize * gain * value;
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