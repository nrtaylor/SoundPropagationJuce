// Room Geometry - class representing set of "Walls" to simulate sound propagation
// Author - Nic Taylor
#pragma once

#include "nVector.h"
#include <atomic>
#include <memory>
#include <vector>
#include <array>

//typedef signed int int32; // TODO: Conflicts with Juce

enum PanningLaw : int {
    PAN_LAW_TRIG_3 = 1,
    PAN_LAW_RATIO_3,
    PAN_LAW_LINEAR_6
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
        global_gain = 0.8f;
        pan_law = PAN_LAW_TRIG_3;
        emitter_pos = { 0.f, 0.f, 0.f };        
    }

    // Not thread safe. Lock emitter first.
    nMath::Vector Update(const signed int _elapsedMs);

    nMath::Vector GetPosition() const
    {
        return emitter_pos;
    }

    void SetPosition(const nMath::Vector& position)
    {
        emitter_pos = position;
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

    float GetFrequency() const
    {
        return frequency.load();
    }

    float GetGlobalGain() const
    {
        return global_gain.load();
    }

    float GetRadius() const
    {
        return radius.load();
    }

    void SetPanLaw(const PanningLaw _pan_law)
    {
        pan_law.store(_pan_law);
    }

    void SetSpread(const float _spread) {
        spread.store(_spread);
    }

    float GetSpread() const
    {
        return spread.load();
    }

    PanningLaw GetPanLaw() const
    {
        return pan_law.load();
    }

    void ComputeGain(const nMath::Vector& receiver_pos, const float new_gain);

    float Gain(const signed int channel) const;

private:
    std::atomic<float> frequency;
    std::atomic<float> global_gain;
    std::atomic<float> radius;    
    std::atomic<float> gain_left;
    std::atomic<float> gain_right;
    std::atomic<float> spread;
    std::atomic<PanningLaw> pan_law;

    nMath::Vector emitter_pos;
    float angle;
};

class GridEmitter {
public:
    const static uint32_t GridDistance = 60; // meters    
    const static uint32_t GridCellsPerMeter = 1;
    const static uint32_t GridResolution = GridCellsPerMeter * GridDistance;
    using GeometryGrid = std::array<std::array<bool, GridResolution>, GridResolution>;

    GridEmitter() {
        for (auto& row : grid) {
            row.fill(false);
        }
        //grid[15][16] = true;
        //grid[15][16] = true;
        //grid[15][15] = true;
        //grid[41][20] = true;
        //grid[42][20] = true;
        //grid[44][20] = true;
    }

    void GridOn(const nMath::Vector& position);
    const GeometryGrid& Grid() const { return grid; }
       
private:
    GeometryGrid grid;
};

class RoomGeometry
{
private:
    std::vector<nMath::LineSegment> walls;
    nMath::LineSegment bounding_box;
public:
    typedef std::add_const<std::add_lvalue_reference<decltype(walls)>::type>::type ConstRefLineSegments;
    RoomGeometry();

    void AddWall(const nMath::Vector start, const nMath::Vector end);
    
    bool Intersects(const nMath::LineSegment& _line) const;

    const std::vector<nMath::LineSegment>& Walls() const
    { 
        return walls; 
    }

private:
};
