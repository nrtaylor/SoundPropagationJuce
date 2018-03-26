// Room Geometry - class representing set of "Walls" to simulate sound propagation
// Author - Nic Taylor
#include "RoomGeometry.h"
#define _USE_MATH_DEFINES
#include <math.h>

void MovingEmitter::Update(int32 _elapsedMs)
{
    angle += 2 * (float)M_PI * (_elapsedMs * frequency) / 1000.f;
    Vector3D<float> new_position(cosf(angle), sinf(angle), 0.f);
    pan_amount = new_position.x;

    const float emitter_radius = radius.load();
    emitter.SetPosition(Vector3D<float>(new_position.x*emitter_radius, new_position.y*emitter_radius, 0.f));
}

// gain left/right should be pulled out as a process
void MovingEmitter::ComputeGain(const float new_gain)
{
    // Factor such that panned hard left/right will have the same rms as pan center.
    const float normed_loudness = new_gain * (float)M_SQRT1_2;
    gain_left.store(normed_loudness * sqrtf(1.f - pan_amount));
    gain_right.store(normed_loudness * sqrtf(1.f + pan_amount));
}

float MovingEmitter::Gain(const int32 channel) const
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

void MovingEmitter::Paint(Graphics& _g, const Rectangle<int> _bounds, const float _zoom_factor) const
{
    const float min_extent = (float)jmin(_bounds.getWidth(), _bounds.getHeight());
    const Vector3D<float> center(min_extent / 2.f, min_extent / 2.f, 0.f);

    _g.setColour(Colour::fromRGB(0xFF, 0xFF, 0xFF));
    _g.fillEllipse(center.x - 1.f, center.y - 1.f, 2, 2);

    const Vector3D<float>& emitter_pos = emitter.GetPosition();
    Vector3D<float> emitter_draw_pos(emitter_pos.x * _zoom_factor + center.x, -emitter_pos.y * _zoom_factor + center.y, 0.f);
    _g.fillEllipse(emitter_draw_pos.x - 1.f, emitter_draw_pos.y - 1.f, 2.5, 2.5);
}

void RayCastCollector::Start()
{
    collector_lock.lock();
    ray_casts.clear();
}

void RayCastCollector::Add(const LineSegment& ray_cast)
{
    ray_casts.emplace_back(ray_cast);
}

void RayCastCollector::Finished()
{
    collector_lock.unlock();
}

void RayCastCollector::Paint(Graphics& _g, const Rectangle<int> _bounds, const float _zoom_factor)
{
    std::lock_guard<std::mutex> guard(collector_lock);
    if (ray_casts.size() > 0)
    {
        const float min_extent = (float)jmin(_bounds.getWidth(), _bounds.getHeight());
        const Vector3D<float> center(min_extent / 2.f, min_extent / 2.f, 0.f);
        _g.setColour(Colour::fromRGB(0x0, 0xAA, 0xAA));
        for (const LineSegment line : ray_casts)
        {
            _g.drawLine((line.start.x * _zoom_factor) + center.x,
                -(line.start.y * _zoom_factor) + center.y,
                (line.end.x * _zoom_factor) + center.x,
                -(line.end.y * _zoom_factor) + center.y);
        }
    }
}

RoomGeometry::RoomGeometry() :
        bounding_box{ {},{} }
{
    walls.reserve(4);
}

void RoomGeometry::AddWall(const Vector3D<float> start, const Vector3D<float> end)
{
    walls.emplace_back(LineSegment{ start, end });
    if (walls.size() == 1)
    {
        bounding_box = LineSegment{
            { jmin(start.x, end.x), jmin(start.y,end.y), 0.f },
            { jmax(start.x, end.x), jmax(start.y,end.y), 0.f } };
    }
    else
    {
        if (jmin(start.x, end.x) < bounding_box.start.x)
        {
            bounding_box.start.x = jmin(start.x, end.x);
        }
        if (jmax(start.x, end.x) > bounding_box.end.x)
        {
            bounding_box.end.x = jmax(start.x, end.x);
        }
        if (jmin(start.y, end.y) < bounding_box.start.y)
        {
            bounding_box.start.y = jmin(start.y, end.y);
        }
        if (jmax(start.y, end.y) > bounding_box.end.y)
        {
            bounding_box.end.y = jmax(start.y, end.y);
        }
    }
}

template<bool capture_debug>
float RoomGeometry::Simulate(const Vector3D<float>& source, const Vector3D<float>& receiver, const SoundPropagation::MethodType method) const
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

void RoomGeometry::AssignCollector(std::unique_ptr<RayCastCollector>& collector)
{
    if (collector != nullptr)
    {
        ray_cast_collector = std::move(collector);
    }
    else if (ray_cast_collector != nullptr)
    {
        collector = std::move(ray_cast_collector);
    }
}

template<bool capture_debug>
bool RoomGeometry::Intersects(const LineSegment& _line) const
{
    if (capture_debug &&
        ray_cast_collector != nullptr)
    {
        ray_cast_collector->Add(_line);
    }

    if (jmin(_line.start.x, _line.end.x) > bounding_box.end.x ||
        jmax(_line.start.x, _line.end.x) < bounding_box.start.x ||
        jmin(_line.start.y, _line.end.y) > bounding_box.end.y ||
        jmax(_line.start.y, _line.end.y) < bounding_box.start.y)
    {
        return false;
    }

    const Vector3D<float> line2 = _line.end - _line.start;
    for (const LineSegment& wall : walls)
    {
        Vector3D<float> wall2 = wall.end - wall.start;
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

void RoomGeometry::Paint(Graphics& _g, const Rectangle<int> _bounds, const float _zoom_factor) const
{
    const float min_extent = (float)jmin(_bounds.getWidth(), _bounds.getHeight());
    const Vector3D<float> center(min_extent / 2.f, min_extent / 2.f, 0.f);

    _g.setColour(Colour::fromRGB(0x77, 0x1c, 0x47));
    Path room;
    for (const LineSegment& wall : walls)
    {
        const Line<float> drawLine(
            wall.start.x * _zoom_factor + center.x,
            -wall.start.y * _zoom_factor + center.y,
            wall.end.x * _zoom_factor + center.x,
            -wall.end.y * _zoom_factor + center.y);
        room.addLineSegment(drawLine, 2.f);
    }
    _g.fillPath(room);
}

template<bool capture_debug>
float RoomGeometry::SimulateSpecularLOS(const Vector3D<float>& source, const Vector3D<float>& receiver) const
{
    if (Intersects<capture_debug>(LineSegment{ source, receiver }))
    {
        return 0.f;
    }

    static const float near_field_distance = 0.75f; // around 440 hz

    const float distance = (source - receiver).length();
    if (distance < near_field_distance)
    {
        return 1.f;
    }
    const float geometric_attenuation = jmin(1.f, 1.f / distance);
    return geometric_attenuation;
}

template<bool capture_debug>
float RoomGeometry::SimulateRayCasts(const Vector3D<float>& source, const Vector3D<float>& receiver) const
{
    Vector3D<float> direction = source - receiver;
    float distance = direction.length();
    if (Intersects<capture_debug>(LineSegment{ source, receiver }))
    {
        if (distance > FLT_EPSILON)
        {
            direction = direction / distance;
            Vector3D<float> ray_orth = { -direction.y, direction.x, 0.f };
            Vector3D<float> ray_orth_inv = ray_orth * -1.f;
            if ((!Intersects<capture_debug>(LineSegment{ ray_orth + receiver, receiver }) &&
                !Intersects<capture_debug>(LineSegment{ ray_orth + receiver, source })) ||
                (!Intersects<capture_debug>(LineSegment{ ray_orth_inv + receiver, receiver }) &&
                    !Intersects<capture_debug>(LineSegment{ ray_orth_inv + receiver, source })))
            {
                distance += 1.f;
            }
            else if ((!Intersects<capture_debug>(LineSegment{ ray_orth + source, source }) &&
                !Intersects<capture_debug>(LineSegment{ ray_orth + source, receiver })) ||
                (!Intersects<capture_debug>(LineSegment{ ray_orth_inv + source, source }) &&
                    !Intersects<capture_debug>(LineSegment{ ray_orth_inv + source, receiver })))
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
    const float geometric_attenuation = jmin(1.f, 1.f / distance);
    return geometric_attenuation;
}
