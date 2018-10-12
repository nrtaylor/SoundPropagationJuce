// Room Geometry - class representing set of "Walls" to simulate sound propagation
// Author - Nic Taylor
#include "RoomGeometry.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <array>

typedef signed int int32; // TODO: PCH

nMath::Vector MovingEmitter::Update(const signed int _elapsedMs)
{
    angle += 2 * (float)M_PI * (_elapsedMs * frequency) / 1000.f;
    while (angle > 2.f * (float)M_PI) {
        angle -= 2.f * (float)M_PI;
    }
    nMath::Vector new_position{ cosf(angle), sinf(angle), 0.f };
    pan_amount = new_position.x;

    const float emitter_radius = radius.load();
    emitter_pos = { new_position.x*emitter_radius, new_position.y*emitter_radius, 0.f };

    return emitter_pos;
}

enum PanningLaw : int {
    PAN_LAW_TRIG_3,
    PAN_LAW_RATIO_3,
    PAN_LAW_LINEAR_6
};

// gain left/right should be pulled out as a process
void MovingEmitter::ComputeGain(const float new_gain)
{
    static const PanningLaw panning_law = PAN_LAW_LINEAR_6;
    switch (panning_law) {
    case PAN_LAW_TRIG_3:
        // Factor such that panned hard left/right will have the same rms as pan center.
    {
        const float normed_loudness = new_gain * (float)M_SQRT1_2;
        gain_left.store(normed_loudness * sqrtf(1.f - pan_amount));
        gain_right.store(normed_loudness * sqrtf(1.f + pan_amount));
    }
    break;
    case PAN_LAW_RATIO_3:
    {
        float angle_mod = angle;
        if (angle > (float)M_PI) {
            angle_mod = (float)M_PI - (angle_mod - floorf(angle_mod / (float)M_PI) * (float)M_PI);
        }
        const float theta = angle_mod / (FLT_EPSILON + (float)M_PI - angle_mod);
        gain_left.store(new_gain * sqrtf(1 / (1 + theta*theta))*theta);
        gain_right.store(new_gain * sqrtf(1 / (1 + theta*theta)));
    }
    break;
    case PAN_LAW_LINEAR_6:
    {
        const float lerp_value = (1 - pan_amount) / 2.f;
        gain_left.store(new_gain * lerp_value);
        gain_right.store(new_gain * (1.f - lerp_value));
    }
    break;
    }
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

bool RoomGeometry::Intersects(const nMath::LineSegment& _line) const
{
    if (nMath::Min(_line.start.x, _line.end.x) > bounding_box.end.x ||
        nMath::Max(_line.start.x, _line.end.x) < bounding_box.start.x ||
        nMath::Min(_line.start.y, _line.end.y) > bounding_box.end.y ||
        nMath::Max(_line.start.y, _line.end.y) < bounding_box.start.y)
    {
        return false;
    }

    for (const nMath::LineSegment& wall : walls)
    {
        if (nMath::Intersect2D(wall, _line))
        {
            return true;
        }
    }

    return false;
}
