// Room Geometry - class representing set of "Walls" to simulate sound propagation
// Author - Nic Taylor
#pragma once

#include "nVector.h"
#include <atomic>
#include <memory>
#include <vector>

//typedef signed int int32; // TODO: Conflicts with Juce

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
        emitter_pos = { 0.f, 0.f, 0.f };
    }

    nMath::Vector Update(const signed int _elapsedMs);

    nMath::Vector GetPosition() const
    {
        return emitter_pos;
    }

    // The following should be thread safe.
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

    void ComputeGain(const float new_gain);

    float Gain(const signed int channel) const;

private:
    std::atomic<float> frequency;
    std::atomic<float> global_gain;
    std::atomic<float> radius;    
    std::atomic<float> gain_left;
    std::atomic<float> gain_right;

    nMath::Vector emitter_pos;
    float angle;
    float pan_amount;
};

class RayCastCollector
{
private:
    std::vector<nMath::LineSegment> ray_casts;

public:
    typedef std::add_const<std::add_lvalue_reference<decltype(ray_casts)>::type>::type ConstRefLineSegments;
    RayCastCollector() {}

    void Reset();

    void Add(const nMath::LineSegment& ray_cast);

    auto RayCasts() -> ConstRefLineSegments const;
};

class RoomGeometry
{
private:
    std::vector<nMath::LineSegment> walls;
    nMath::LineSegment bounding_box;
    std::unique_ptr<RayCastCollector> ray_cast_collector;
public:
    typedef std::add_const<std::add_lvalue_reference<decltype(walls)>::type>::type ConstRefLineSegments;
    RoomGeometry();

    void AddWall(const nMath::Vector start, const nMath::Vector end);
    
    bool Intersects(const nMath::LineSegment& _line) const;

    const std::vector<nMath::LineSegment>& Walls() const
    { 
        return walls; 
    }

    void SwapCollector(std::unique_ptr<RayCastCollector>& collector);

private:
    template<bool capture_debug>
    void CaptureDebug(const nMath::LineSegment& _line) const;
};
