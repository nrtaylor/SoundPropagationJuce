// Room Geometry - class representing set of "Walls" to simulate sound propagation
// Author - Nic Taylor
#pragma once

#include "nVector.h"
#include <atomic>
#include <memory>
#include <vector>

//typedef signed int int32; // TODO: Conflicts with Juce

namespace SoundPropagation
{
    enum MethodType : signed int
    {
        Method_SpecularLOS = 1,
        Method_RayCasts,
        Method_Pathfinding,
        Method_Wave,

        Method_Off
    };
}

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

    template<bool capture_debug = false>
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

class PropagationPlanner
{
public:
    virtual void Preprocess(std::shared_ptr<const RoomGeometry> _room) = 0;

    struct SourceConfig
    {
        const nMath::Vector source;
        const float frequency;
        const float time_scale;
    };    
    virtual void Plan(const SourceConfig& _config) = 0;
    virtual float Simulate(const nMath::Vector& _receiver, const float _time_ms) const = 0;
};

class PlannerSpecularLOS : public PropagationPlanner
{
    void Preprocess(std::shared_ptr<const RoomGeometry> _room) override;
    void Plan(const PropagationPlanner::SourceConfig& _config) override;
    float Simulate(const nMath::Vector& _receiver, const float _time_ms) const override;
private:
    nMath::Vector source;
    std::shared_ptr<const RoomGeometry> room;
};

class PlannerRayCasts : public PropagationPlanner
{
    void Preprocess(std::shared_ptr<const RoomGeometry> _room) override;
    void Plan(const PropagationPlanner::SourceConfig& _config) override;
    float Simulate(const nMath::Vector& _receiver, const float _time_ms) const override;
private:
    nMath::Vector source;
    std::shared_ptr<const RoomGeometry> room;
};

class PlannerWave : public PropagationPlanner
{
public:
    void Preprocess(std::shared_ptr<const RoomGeometry> _room) override;
    void Plan(const PropagationPlanner::SourceConfig& _config) override;
    float Simulate(const nMath::Vector& _receiver, const float _time_ms) const override;
private:
    std::vector<nMath::Vector> first_reflections;
    nMath::Vector source;
    std::shared_ptr<const RoomGeometry> room;
    float frequency;
    float time_factor;
};

class PlannerAStar : public PropagationPlanner
{
private:
    const static uint32_t GridDistance = 60; // meters    
    const static uint32_t GridCellsPerMeter = 2;
    const static uint32_t GridResolution = GridCellsPerMeter * GridDistance;
    typedef std::array<std::array<bool, GridResolution>, GridResolution> GeometryGrid;
    typedef std::array<std::array<float, GridResolution>, GridResolution> GeometryGridCache;
    enum GridNodeState : int8_t
    {
        GNS_NOT_FOUND = 0,
        GNS_FOUND,
        GNS_CHECKED
    };
    struct GridNode
    {
        uint32_t score;
        int8_t link_index;
        GridNodeState state;
    };
    typedef std::array<std::array<GridNode, GridResolution>, GridResolution> GeometryGridScore;

    struct Coord
    {
        int row;
        int col;
    };

public:
    PlannerAStar();
    void Preprocess(std::shared_ptr<const RoomGeometry> _room) override;
    void Plan(const PropagationPlanner::SourceConfig& _config) override;
    float Simulate(const nMath::Vector& _receiver, const float _time_ms) const override;

    const std::unique_ptr<GeometryGrid>& Grid() const
    {
        return grid;
    }
private:
    float FindAStarDiscrete(const Coord& receiver_coord);

    std::shared_ptr<const RoomGeometry> room;
    std::unique_ptr<GeometryGrid> grid;
    std::unique_ptr<GeometryGridCache> grid_cache;
    Coord source_coord;
};
