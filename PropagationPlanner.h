// PropagationPlanner - classes to handled different methods of sound propagation from
// geometric approximations to frequency dependent methods based on the wave equation.
// Author - Nic Taylor
#pragma once

#include "nVector.h"
#include <atomic>
#include <memory>
#include <vector>

namespace SoundPropagation
{
    enum MethodType : signed int
    {
        Method_SpecularLOS = 1,
        Method_RayCasts,
        Method_Pathfinding,
        Method_Wave,
        Method_LOSAStarFallback,

        Method_Off
    };

    enum ResultConfig
    {
        PRD_GAIN,
        PRD_REFLECTIONS_ONLY,
        PRD_FULL,
    };
}

class RoomGeometry;

struct PropagationSimulationCache
{
    virtual ~PropagationSimulationCache() = default;
};

struct PropagationResult
{
    const SoundPropagation::ResultConfig config;
    float gain; // TODO: find better term. Perhaps dampening?

    std::vector<nMath::LineSegment> intersections;
    std::shared_ptr<PropagationSimulationCache> cache;
};

class PropagationPlanner
{
public:
    static std::shared_ptr<PropagationPlanner> MakePlanner(const SoundPropagation::MethodType method);
    virtual void Preprocess(std::shared_ptr<const RoomGeometry> _room) = 0;

    struct SourceConfig
    {
        const nMath::Vector source;
        const float frequency;
        const float time_scale;
    };
    virtual void Plan(const SourceConfig& _config) = 0;
    virtual void Simulate(PropagationResult& result, const nMath::Vector& _receiver, const float _time_ms) const = 0;
    virtual ~PropagationPlanner() = default;
};

class PlannerSpecularLOS : public PropagationPlanner
{
public:
    void Preprocess(std::shared_ptr<const RoomGeometry> _room) override;
    void Plan(const PropagationPlanner::SourceConfig& _config) override;
    void Simulate(PropagationResult& result, const nMath::Vector& _receiver, const float _time_ms) const override;
private:
    nMath::Vector source;
    std::shared_ptr<const RoomGeometry> room;
};

class PlannerRayCasts : public PropagationPlanner
{
public:
    void Preprocess(std::shared_ptr<const RoomGeometry> _room) override;
    void Plan(const PropagationPlanner::SourceConfig& _config) override;
    void Simulate(PropagationResult& result, const nMath::Vector& _receiver, const float _time_ms) const override;
private:
    nMath::Vector source;
    std::shared_ptr<const RoomGeometry> room;

    bool Intersects(PropagationResult& result, const nMath::Vector& start, const nMath::Vector& end) const;
};

template<class PlannerPrimary, class PlannerSecondary>
class PlannerTwoStages : public PropagationPlanner
{
public:
    PlannerTwoStages();

    void Preprocess(std::shared_ptr<const RoomGeometry> _room) override;
    void Plan(const PropagationPlanner::SourceConfig& _config) override;
    void Simulate(PropagationResult& result, const nMath::Vector& _receiver, const float _time_ms) const override;

    std::shared_ptr<const PlannerSecondary> Secondary() const
    {
        return planner_secondary;
    }

private:
    std::shared_ptr<PlannerPrimary> planner_primary;
    std::shared_ptr<PlannerSecondary> planner_secondary;
};

class PlannerWave : public PropagationPlanner
{
public:
    void Preprocess(std::shared_ptr<const RoomGeometry> _room) override;
    void Plan(const PropagationPlanner::SourceConfig& _config) override;
    void Simulate(PropagationResult& result, const nMath::Vector& _receiver, const float _time_ms) const override;
private:
    std::vector<nMath::Vector> first_reflections;
    nMath::Vector source;
    std::shared_ptr<const RoomGeometry> room;
    float frequency;
    float time_factor;
};