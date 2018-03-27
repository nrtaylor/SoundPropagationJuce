// Room Geometry - class representing set of "Walls" to simulate sound propagation
// Author - Nic Taylor
#pragma once

#include "nVector.h"
#include <atomic>
#include <vector>
#include <mutex>

//typedef signed int int32; // TODO: Conflicts with Juce

namespace SoundPropagation
{
    enum MethodType : signed int
    {
        Method_SpecularLOS = 1,
        Method_RayCasts,
        Method_Pathfinding,

        Method_Off
    };
}

class SoundEmitter
{
public:
    SoundEmitter() 
    {
        position = { 0.f, 0.f, 0.f };
    }

    void SetPosition(const nMath::Vector& _position)
    {
        position = _position;
    }

    const nMath::Vector& GetPosition() const
    {
        return position;
    }

private:
    nMath::Vector position;
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

    const nMath::Vector GetPosition() const
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

    void Update(signed int _elapsedMs);

    void ComputeGain(const float new_gain);

    float Gain(const signed int channel) const;

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

    void Add(const nMath::LineSegment& ray_cast);

    void Finished();

    const std::vector<nMath::LineSegment>& RayCasts(); // TODO: Clean-up locks
private:
    std::vector<nMath::LineSegment> ray_casts;
    std::mutex collector_lock;
};

class RoomGeometry
{
public:
    RoomGeometry();

    void AddWall(const nMath::Vector start, const nMath::Vector end);

    template<bool capture_debug = false>
    float Simulate(const nMath::Vector& source, const nMath::Vector& receiver, const SoundPropagation::MethodType method) const;

    void AssignCollector(std::unique_ptr<RayCastCollector>& collector);

    template<bool capture_debug = false>
    bool Intersects(const nMath::LineSegment& _line) const;

    const std::vector<nMath::LineSegment>& Walls() const // TODO: clean-up
    { 
        return walls; 
    }

private:
    std::vector<nMath::LineSegment> walls;
    nMath::LineSegment bounding_box;
    std::unique_ptr<RayCastCollector> ray_cast_collector;

    template<bool capture_debug = false>
    float SimulateSpecularLOS(const nMath::Vector& source, const nMath::Vector& receiver) const;

    template<bool capture_debug = false>
    float SimulateRayCasts(const nMath::Vector& source, const nMath::Vector& receiver) const;

    template<bool capture_debug>
    void CaptureDebug(const nMath::LineSegment& _line) const;
};

template float RoomGeometry::Simulate<false>(const nMath::Vector& source, const nMath::Vector& receiver, const SoundPropagation::MethodType method) const;
template float RoomGeometry::Simulate<true>(const nMath::Vector& source, const nMath::Vector& receiver, const SoundPropagation::MethodType method) const;