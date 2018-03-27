// Room Geometry - class representing set of "Walls" to simulate sound propagation
// Author - Nic Taylor
#pragma once

#include "../JuceLibraryCode/JuceHeader.h"
#include <vector>
#include <mutex>

namespace SoundPropagation
{
    enum MethodType : int32
    {
        Method_SpecularLOS = 1,
        Method_RayCasts,
        Method_Pathfinding,

        Method_Off
    };
}

struct LineSegment
{
    Vector3D<float> start;
    Vector3D<float> end;
};

class SoundEmitter
{
public:
    SoundEmitter() {}

    void SetPosition(const Vector3D<float>& _position)
    {
        position = _position;
    }

    const Vector3D<float>& GetPosition() const
    {
        return position;
    }

private:
    Vector3D<float> position;
};

class MovingEmitter
{
public:
    MovingEmitter() :
        frequency(0.2f),
        radius(10.f),
        angle(0.f)
    {
        gain_left = 0.f;
        gain_right = 0.f;
        pan_amount = 0.f;
        global_gain = 0.8f;
    }

    const Vector3D<float> GetPosition() const
    {
        return emitter.GetPosition();
    }

    void SetFrequency(const float& _frequency)
    {
        frequency.store(_frequency);
    }

    void SetGlobalGain(const float& _gain)
    {
        global_gain.store(_gain);
    }

    void SetRadius(const float& _radius)
    {
        radius.store(_radius);
    }

    void Update(int32 _elapsedMs);

    void ComputeGain(const float new_gain);

    float Gain(const int32 channel) const;

    void Paint(Graphics& _g, const Rectangle<int> _bounds, const float _zoom_factor) const;

private:
    std::atomic<float> frequency;
    std::atomic<float> global_gain;
    std::atomic<float> radius;
    float angle;
    std::atomic<float> gain_left;
    std::atomic<float> gain_right;
    SoundEmitter emitter;
    float pan_amount;
};

class RayCastCollector
{
public:
    RayCastCollector() {}

    void Start();

    void Add(const LineSegment& ray_cast);

    void Finished();

    void Paint(Graphics& _g, const Rectangle<int> _bounds, const float _zoom_factor);
private:
    std::vector<LineSegment> ray_casts;
    std::mutex collector_lock;
};

class RoomGeometry
{
public:
    RoomGeometry();

    void AddWall(const Vector3D<float> start, const Vector3D<float> end);

    template<bool capture_debug = false>
    float Simulate(const Vector3D<float>& source, const Vector3D<float>& receiver, const SoundPropagation::MethodType method) const;

    void AssignCollector(std::unique_ptr<RayCastCollector>& collector);

    template<bool capture_debug = false>
    bool Intersects(const LineSegment& _line) const;

    void Paint(Graphics& _g, const Rectangle<int> _bounds, const float _zoom_factor) const;
private:
    std::vector<LineSegment> walls;
    LineSegment bounding_box;
    std::unique_ptr<RayCastCollector> ray_cast_collector;

    template<bool capture_debug = false>
    float SimulateSpecularLOS(const Vector3D<float>& source, const Vector3D<float>& receiver) const;

    template<bool capture_debug = false>
    float SimulateRayCasts(const Vector3D<float>& source, const Vector3D<float>& receiver) const;

    template<bool capture_debug>
    void CaptureDebug(const LineSegment& _line) const;
};

template float RoomGeometry::Simulate<false>(const Vector3D<float>& source, const Vector3D<float>& receiver, const SoundPropagation::MethodType method) const;
template float RoomGeometry::Simulate<true>(const Vector3D<float>& source, const Vector3D<float>& receiver, const SoundPropagation::MethodType method) const;