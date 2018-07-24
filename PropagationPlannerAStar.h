// PropagationPlanner - classes to handled different methods of sound propagation from
// geometric approximations to frequency dependent methods based on the wave equation.
// Author - Nic Taylor
#pragma once

#include "PropagationPlanner.h"
#include <array>

class PlannerAStar : public PropagationPlanner
{
public:
    const static uint32_t GridDistance = 60; // meters    
    const static uint32_t GridCellsPerMeter = 2;
    const static uint32_t GridResolution = GridCellsPerMeter * GridDistance;
    using GeometryGrid = std::array<std::array<bool, GridResolution>, GridResolution>;
private:
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
    using GeometryGridScore = std::array<std::array<GridNode, GridResolution>, GridResolution>;

    struct Coord
    {
        int row;
        int col;
    };

    using GeometryGridCache = std::array<std::array<float, GridResolution>, GridResolution>;
    struct AStarSimulateCache : public PropagationSimulationCache
    {
        GeometryGridCache grid_result;
        std::shared_ptr<GeometryGridScore> grid_score;
    };

public:
    PlannerAStar() {}
    void Preprocess(std::shared_ptr<const RoomGeometry> _room) override;
    void Plan(const PropagationPlanner::SourceConfig& _config) override;
    void Simulate(PropagationResult& result, const nMath::Vector& _receiver, const float _time_ms) const override;

    static bool GridNodeSearched(std::shared_ptr<const PropagationSimulationCache> cache, int row, int col);

    const GeometryGrid& Grid() const
    {
        return grid;
    }
private:
    float FindAStarDiscrete(PropagationResult& result, const Coord& receiver_coord, std::shared_ptr<AStarSimulateCache> grid_cache) const;
    
    GeometryGrid grid;
    std::shared_ptr<const RoomGeometry> room;
    Coord source_coord;
};
