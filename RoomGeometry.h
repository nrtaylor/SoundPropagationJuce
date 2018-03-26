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
};

template float RoomGeometry::Simulate<0>(const Vector3D<float>& source, const Vector3D<float>& receiver, const SoundPropagation::MethodType method) const;
template float RoomGeometry::Simulate<1>(const Vector3D<float>& source, const Vector3D<float>& receiver, const SoundPropagation::MethodType method) const;