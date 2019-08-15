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

    const float emitter_radius = radius.load();
    emitter_pos = { new_position.x*emitter_radius, new_position.y*emitter_radius, 0.f };

    return emitter_pos;
}

// gain left/right should be pulled out as a process
void MovingEmitter::ComputeGain(const nMath::Vector& receiver_pos, const float new_gain)
{
    const nMath::Vector direction = emitter_pos - receiver_pos;
    float pan_amount = direction.x / nMath::Length(direction);
    float spread_pan_amount = pan_amount;
    const float _spread = spread.load();
    if (_spread > 0.f) {
        spread_pan_amount *= 1.f - _spread / 100.f;
    }
    switch (pan_law.load()) {
    case PAN_LAW_TRIG_3:
        // Factor such that panned hard left/right will have the same rms as pan center.
    {
        const float normed_loudness = new_gain * (float)M_SQRT1_2;
        gain_left.store(normed_loudness * sqrtf(1.f - spread_pan_amount));
        gain_right.store(normed_loudness * sqrtf(1.f + spread_pan_amount));
    }
    break;
    case PAN_LAW_RATIO_3: // For testing pan laws only. Not correct otherwise.
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
        const float lerp_value = (1 - spread_pan_amount) / 2.f;
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

void GridEmitter::Update(const nMath::Vector& receiver_pos) {
    const float half_grid_cell_size = 0.5f / (float)GridCellsPerMeter;

    const float attenuation_range = 45.f;

    float total_weight = 0.0;
    float spread = 0.f;
    float closest_distance = FLT_MAX;
    nMath::Vector total_dir = { 0.0, 0.0, 0.0 };
    nMath::Vector emitter_direction = { 0.0, 0.0, 0.0 };

    // TODO: Track closest point.
    for (int x = 0; x < GridResolution; ++x) {
        for (int y = 0; y < GridResolution; ++y) {
            if (grid[y][x]) {
                nMath::Vector grid_cell_center = {x / (float)GridCellsPerMeter,
                    y / (float)GridCellsPerMeter, 0.f};
                grid_cell_center.x -= GridDistance / 2.f;
                grid_cell_center.y -= GridDistance / 2.f;
                const nMath::Vector direction = grid_cell_center - receiver_pos;
                if (fabs(direction.x) < half_grid_cell_size &&
                    fabs(direction.y) < half_grid_cell_size &&
                    fabs(direction.z) < half_grid_cell_size)
                {
                    spread = 1.0;
                    emitter_direction = { 1.0, 0.0, 0.0 };
                    break;
                }            
                const float distance = nMath::Length(direction);                
                if (distance < attenuation_range)
                {
                    if (distance < closest_distance) {
                        closest_distance = distance;
                    }
                    const float weight = attenuation_range - distance;
                    total_dir += (weight / distance) * direction;
                    total_weight += weight;
                }
            }
        }
    }

    if (total_weight > 0.f && emitter_direction == nMath::Vector{ 0.f,0.f,0.f }) {
        spread = 1 - nMath::Length(total_dir) / total_weight;
        emitter_direction = closest_distance * total_dir / nMath::Length(total_dir);
        emitter_direction = emitter_direction + receiver_pos;
    }    
    point.SetPosition(emitter_direction);
    point.SetSpread(spread);
}

void GridEmitter::GridOn(const nMath::Vector& position) {
    const int grid_half = (int)GridResolution / 2;
    const nMath::Vector grid_position = position * (float)GridCellsPerMeter +
        nMath::Vector{ (float)grid_half, (float)grid_half };
    const int grid_x = (int)grid_position.x;
    const int grid_y = (int)grid_position.y;
    if (grid_x >= 0 && grid_x < GridResolution && grid_y >= 0 && grid_y < GridResolution) {
        grid[grid_y][grid_x] = true;
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
