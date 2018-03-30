// Room Geometry - class representing set of "Walls" to simulate sound propagation
// Author - Nic Taylor
#include "RoomGeometry.h"
#define _USE_MATH_DEFINES
#include <math.h>

namespace nMath
{
    inline float Min(float lhs, float rhs)
    {
        return (lhs < rhs) ? lhs : rhs;
    }

    inline float Max(float lhs, float rhs)
    {
        return (lhs > rhs) ? lhs : rhs;
    }
}

typedef signed int int32; // TODO: PCH

nMath::Vector MovingEmitter::Update(const signed int _elapsedMs)
{
    angle += 2 * (float)M_PI * (_elapsedMs * frequency) / 1000.f;
    nMath::Vector new_position{ cosf(angle), sinf(angle), 0.f };
    pan_amount = new_position.x;

    const float emitter_radius = radius.load();
    emitter_pos = { new_position.x*emitter_radius, new_position.y*emitter_radius, 0.f };

    return emitter_pos;
}

// gain left/right should be pulled out as a process
void MovingEmitter::ComputeGain(const float new_gain)
{
    // Factor such that panned hard left/right will have the same rms as pan center.
    const float normed_loudness = new_gain * (float)M_SQRT1_2;
    gain_left.store(normed_loudness * sqrtf(1.f - pan_amount));
    gain_right.store(normed_loudness * sqrtf(1.f + pan_amount));
}

float MovingEmitter::Gain(const signed int channel) const
{
    switch (channel)
    {
    case 0:
        return global_gain.load() * gain_left.load();
    case 1:
        return global_gain.load() * gain_right.load();
    default:
        return 0.f;
    }
}

void RayCastCollector::Reset()
{    
    ray_casts.clear();
}

void RayCastCollector::Add(const nMath::LineSegment& ray_cast)
{
    ray_casts.emplace_back(ray_cast);
}

auto RayCastCollector::RayCasts() -> RayCastCollector::ConstRefLineSegments const
{
    return ray_casts;
}

RoomGeometry::RoomGeometry() :
        bounding_box{ {},{} }
{
    walls.reserve(4);
}

void RoomGeometry::AddWall(const nMath::Vector start, const nMath::Vector end)
{
    walls.emplace_back(nMath::LineSegment{ start, end });
    if (walls.size() == 1)
    {
        bounding_box = nMath::LineSegment{
            { nMath::Min(start.x, end.x), nMath::Min(start.y,end.y), 0.f },
            { nMath::Max(start.x, end.x), nMath::Max(start.y,end.y), 0.f } };
    }
    else
    {
        if (nMath::Min(start.x, end.x) < bounding_box.start.x)
        {
            bounding_box.start.x = nMath::Min(start.x, end.x);
        }
        if (nMath::Max(start.x, end.x) > bounding_box.end.x)
        {
            bounding_box.end.x = nMath::Max(start.x, end.x);
        }
        if (nMath::Min(start.y, end.y) < bounding_box.start.y)
        {
            bounding_box.start.y = nMath::Min(start.y, end.y);
        }
        if (nMath::Max(start.y, end.y) > bounding_box.end.y)
        {
            bounding_box.end.y = nMath::Max(start.y, end.y);
        }
    }
}

template<bool capture_debug>
float RoomGeometry::Simulate(const nMath::Vector& source, const nMath::Vector& receiver, const SoundPropagation::MethodType method) const
{
    switch (method)
    {
    case SoundPropagation::Method_SpecularLOS:
        return SimulateSpecularLOS<capture_debug>(source, receiver);
    case SoundPropagation::Method_RayCasts:
        return SimulateRayCasts<capture_debug>(source, receiver);
    default:
        return 0.f;
    }
}

void RoomGeometry::SwapCollector(std::unique_ptr<RayCastCollector>& collector)
{
    std::swap(ray_cast_collector, collector);
}

template<>
void RoomGeometry::CaptureDebug<false>(const nMath::LineSegment& _line) const
{
    (void)_line;
}

template<>
void RoomGeometry::CaptureDebug<true>(const nMath::LineSegment& _line) const
{
    if (ray_cast_collector != nullptr)
    {
        ray_cast_collector->Add(_line);
    }
}

template<bool capture_debug>
bool RoomGeometry::Intersects(const nMath::LineSegment& _line) const
{
    CaptureDebug<capture_debug>(_line);

    if (nMath::Min(_line.start.x, _line.end.x) > bounding_box.end.x ||
        nMath::Max(_line.start.x, _line.end.x) < bounding_box.start.x ||
        nMath::Min(_line.start.y, _line.end.y) > bounding_box.end.y ||
        nMath::Max(_line.start.y, _line.end.y) < bounding_box.start.y)
    {
        return false;
    }

    const nMath::Vector line2 = _line.end - _line.start;
    for (const nMath::LineSegment& wall : walls)
    {
        nMath::Vector wall2 = wall.end - wall.start;
        const float numerator = wall2.y * (_line.start.x - wall.start.x) - wall2.x * (_line.start.y - wall.start.y);
        const float denominator = line2.y * wall2.x - line2.x * wall2.y;

        if (denominator != 0.f)
        {
            float r = numerator / denominator;
            if (r >= 0.f && r <= 1.f)
            {
                const float numerator2 = line2.y * (wall.start.x - _line.start.x) - line2.x * (wall.start.y - _line.start.y);
                const float denominator2 = wall2.y * line2.x - wall2.x * line2.y;
                r = numerator2 / denominator2;
                if (r >= 0.f && r <= 1.f)
                {
                    return true;
                }
            }
        }
    }

    return false;
}

template<bool capture_debug>
float RoomGeometry::SimulateSpecularLOS(const nMath::Vector& source, const nMath::Vector& receiver) const
{
    if (Intersects<capture_debug>(nMath::LineSegment{ source, receiver }))
    {
        return 0.f;
    }

    static const float near_field_distance = 0.75f; // around 440 hz

    const float distance = nMath::Length(source - receiver);
    if (distance < near_field_distance)
    {
        return 1.f;
    }
    const float geometric_attenuation = nMath::Min(1.f, 1.f / distance);
    return geometric_attenuation;
}

template<bool capture_debug>
float RoomGeometry::SimulateRayCasts(const nMath::Vector& source, const nMath::Vector& receiver) const
{
    nMath::Vector direction = source - receiver;
    float distance = nMath::Length(direction);
    if (Intersects<capture_debug>(nMath::LineSegment{ source, receiver }))
    {
        if (distance > FLT_EPSILON)
        {
            direction = direction / distance;
            nMath::Vector ray_orth = { -direction.y, direction.x, 0.f };
            nMath::Vector ray_orth_inv = -1.f * ray_orth;
            if ((!Intersects<capture_debug>(nMath::LineSegment{ ray_orth + receiver, receiver }) &&
                !Intersects<capture_debug>(nMath::LineSegment{ ray_orth + receiver, source })) ||
                (!Intersects<capture_debug>(nMath::LineSegment{ ray_orth_inv + receiver, receiver }) &&
                    !Intersects<capture_debug>(nMath::LineSegment{ ray_orth_inv + receiver, source })))
            {
                distance += 1.f;
            }
            else if ((!Intersects<capture_debug>(nMath::LineSegment{ ray_orth + source, source }) &&
                !Intersects<capture_debug>(nMath::LineSegment{ ray_orth + source, receiver })) ||
                (!Intersects<capture_debug>(nMath::LineSegment{ ray_orth_inv + source, source }) &&
                    !Intersects<capture_debug>(nMath::LineSegment{ ray_orth_inv + source, receiver })))
            {
                distance += 1.f;
            }
            else
            {
                return 0.f;
            }
        }
        else
        {
            return 0.f;
        }
    }

    static const float near_field_distance = 0.75f; // around 440 hz
    if (distance < near_field_distance)
    {
        return 1.f;
    }
    const float geometric_attenuation = nMath::Min(1.f, 1.f / distance);
    return geometric_attenuation;
}
