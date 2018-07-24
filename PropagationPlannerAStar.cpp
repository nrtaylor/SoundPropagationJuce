// PropagationPlanner - classes to handled different methods of sound propagation
// Author - Nic Taylor
#include "PropagationPlannerAStar.h"
#include "RoomGeometry.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <array>

template<class T, class Less>
class PriorityQueue
{
public:
    PriorityQueue(size_t storage_size, const Less& _comp)
        : comp(_comp)
    {
        container.reserve(storage_size);
    }

    bool Empty() const
    {
        return container.empty();
    }

    void Push(T&& _val);
    void Pop();
    const T& Top() const
    {
        return container[0];
    }
private:
    std::vector<T> container;
    Less comp;
};

template<class T, class Less>
void PriorityQueue<T, Less>::Push(T&& _val)
{
    size_t index = container.size();
    container.emplace_back(_val);

    while (index != 0)
    {
        T& child = container[index];
        index >>= 1;
        T& parent = container[index];
        if (comp(_val, parent))
        {
            std::swap(child, parent);
        }
        else
        {
            break;
        }
    }
}

template<class T, class Less>
void PriorityQueue<T, Less>::Pop()
{
    container[0] = container.back();
    container.pop_back();

    const uint32_t heap = (uint32_t)container.size();
    uint32_t index = 0;
    uint32_t next = 0;
    while (index < heap)
    {
        const uint32_t lhs = (index << 1) + 1;
        if (lhs < heap)
        {
            if (comp(container[lhs], container[next]))
            {
                next = lhs;
            }
            const uint32_t rhs = lhs + 1;
            if (rhs < heap && comp(container[rhs], container[next]))
            {
                next = rhs;
            }
        }
        if (next != index)
        {
            std::swap(container[index], container[next]);
            index = next;
        }
        else
        {
            break;
        }
    }
}

void PlannerAStar::Preprocess(std::shared_ptr<const RoomGeometry> _room)
{
    room = _room;
    std::fill_n(&grid[0][0], GridResolution * GridResolution, false);

    auto& walls = room->Walls();
    for (const nMath::LineSegment& ls : walls)
    {
        const nMath::Vector start = ls.start;
        const nMath::Vector end = ls.end;
        // Update Grid
        const int grid_half = (int)GridResolution / 2;
        nMath::LineSegment grid_line{
            start * (float)GridCellsPerMeter + nMath::Vector{ (float)grid_half, (float)grid_half },
            end * (float)GridCellsPerMeter + nMath::Vector{ (float)grid_half, (float)grid_half }
        };

        if (fabsf(end.x - start.x) < fabsf(end.y - start.y))
        {
            if (end.x < start.x)
            {
                std::swap(grid_line.start, grid_line.end);
            }

            const int grid_start = nMath::Max(0, (int)grid_line.start.x);
            const int grid_end = nMath::Min((int)GridResolution - 1, (int)ceilf(grid_line.end.x));

            const float inv_delta = 1.f / (grid_line.end.x - grid_line.start.x);
            const float delta_y = grid_line.end.y - grid_line.start.y;
            int prev_grid_y = (int)grid_line.start.y;
            for (int i = grid_start; i <= grid_end; ++i)
            {
                float t = ((float)i - grid_line.start.x) * inv_delta; // can be nan
                t = nMath::Max(0.f, nMath::Min(1.f, t)); // end points
                if (grid_start == grid_end)
                {
                    t = 1.f;
                }
                const float y = grid_line.start.y + t * delta_y; // prevent rounding error when start.y == end.y
                const int grid_y = nMath::Min<int>(nMath::Max(0, (int)y), (int)GridResolution - 1);
                for (int j = nMath::MinClamped(prev_grid_y, grid_y, 0); j <= nMath::MaxClamped(prev_grid_y, grid_y, (int)GridResolution - 1); ++j)
                {
                    grid[j][i] = true;
                }
                prev_grid_y = grid_y;
            }
        }
        else
        {
            if (end.y < start.y)
            {
                std::swap(grid_line.start, grid_line.end);
            }

            const int grid_start = nMath::Max(0, (int)grid_line.start.y);
            const int grid_end = nMath::Min((int)GridResolution - 1, (int)ceilf(grid_line.end.y));

            const float inv_delta = 1.f / (grid_line.end.y - grid_line.start.y);
            const float delta_x = grid_line.end.x - grid_line.start.x;
            int prev_grid_x = (int)grid_line.start.x;
            for (int i = grid_start; i <= grid_end; ++i)
            {
                float t = ((float)i - grid_line.start.y) * inv_delta; // can be nan
                t = nMath::Max(0.f, nMath::Min(1.f, t)); // end points
                if (grid_start == grid_end)
                {
                    t = 1.f;
                }
                const float x = grid_line.start.x + t * delta_x; // prevent rounding error when start.x == end.x
                const int grid_x = nMath::Min<int>(nMath::Max(0, (int)x), (int)GridResolution - 1);
                for (int j = nMath::MinClamped(prev_grid_x, grid_x, 0); j <= nMath::MaxClamped(prev_grid_x, grid_x, (int)GridResolution - 1); ++j)
                {
                    grid[i][j] = true;
                }
                prev_grid_x = grid_x;
            }
        }
    }
}

void PlannerAStar::Plan(const SourceConfig& _config)
{
    const int grid_half = (int)GridResolution / 2;
    const nMath::Vector grid_source = _config.source * (float)GridCellsPerMeter + nMath::Vector{ (float)grid_half, (float)grid_half };
    source_coord = Coord{ (int)grid_source.y, (int)grid_source.x };
}

float PlannerAStar::FindAStarDiscrete(PropagationResult& result, const Coord& receiver_coord, std::shared_ptr<AStarSimulateCache> cache) const
{
    if (grid[receiver_coord.row][receiver_coord.col]) // wall
    {
        if (result.config == SoundPropagation::PRD_FULL)
        {
            result.intersections.clear();
            cache->grid_score = nullptr;
        }
        cache->grid_result[receiver_coord.row][receiver_coord.col] = 0.f;
        return 0.f;
    }

    // Verion 1: distance based
    //auto heuristic = [&receiver_coord](const Coord& c)
    //{
    //    const float distance = sqrtf((float)((c.col - receiver_coord.col)*(c.col - receiver_coord.col) +
    //        (c.row - receiver_coord.row)*(c.row - receiver_coord.row)));
    //    return (uint32_t)(distance * 1000.f);
    //};

    // Version 2: moving "behind" the source is penalized more. Penalization is somewhat arbitrary.
    const Coord source_dir = { receiver_coord.row - source_coord.row, receiver_coord.col - source_coord.col };
    auto heuristic = [&receiver_coord, &source_dir, this](const Coord& c)
    {
        const Coord direction = { c.row - receiver_coord.row, c.col - receiver_coord.col };
        const float distance = sqrtf((float)direction.col * direction.col + direction.row * direction.row);

        const Coord from_source = { c.row - source_coord.row, c.col - source_coord.col };
        const float dot = (float)(from_source.row * source_dir.row + from_source.col * source_dir.col);
        if (dot >= 0.f)
        {
            return (uint32_t)(distance * 1000.f);
        }
        else
        {
            return (uint32_t)((distance + sqrt(-dot)) * 1000.f);
        }
    };

    GeometryGridScore grid_nodes;
    const GridNode empty_node{ INT_MAX, -1, GNS_NOT_FOUND };
    std::fill_n(&grid_nodes[0][0], GridResolution * GridResolution, empty_node);
    typedef std::pair<uint32_t, Coord> ScoredCoord;
    auto compareScore = [](const ScoredCoord& lhs, const ScoredCoord& rhs)
    {
        return lhs.first < rhs.first;
    };

    PriorityQueue<ScoredCoord, decltype(compareScore)> prediction((size_t)(8 * M_SQRT2 * GridResolution), compareScore);

    const uint32_t ideal_distance = heuristic(source_coord);
    prediction.Push(ScoredCoord(ideal_distance, source_coord));

    // Start
    grid_nodes[source_coord.row][source_coord.col] = GridNode{ 0, -1, GNS_FOUND };

    static Coord neighbors[] = {
        { -1, -1 },
        { -1,  0 },
        { -1,  1 },
        { 0, -1 },
        { 0,  1 },
        { 1, -1 },
        { 1,  0 },
        { 1,  1 },
    };

    const int max_test = (int)(GridResolution * GridResolution / 2);
    uint32_t num_checked = 0;
    uint32_t num_discovered = 1;
    static const uint32_t score_diagonal = (uint32_t)(M_SQRT2*1000.f);
    while (num_discovered)
    {
        Coord next_coord = prediction.Top().second;
        GridNode& node = grid_nodes[next_coord.row][next_coord.col];
        if (node.state == GNS_CHECKED)
        {
            prediction.Pop();
            continue;
        }
        node.state = GNS_CHECKED;
        --num_discovered;

        if (next_coord.col == receiver_coord.col &&
            next_coord.row == receiver_coord.row)
        {
            break;
        }
        if (++num_checked > max_test)
        {
            if (result.config == SoundPropagation::PRD_FULL)
            {
                result.intersections.clear();
                cache->grid_score = std::make_shared<GeometryGridScore>(grid_nodes);
            }
            return 0.f;
        }

        prediction.Pop();

        const uint32_t score = node.score;
        for (int i = 0; i < 8; ++i)
        {
            Coord neighbor = neighbors[i];
            const uint32_t neighbor_dist = (neighbor.row == 0 || neighbor.col == 0) ? 1000 : score_diagonal;
            neighbor.row += next_coord.row;
            neighbor.col += next_coord.col;
            if (neighbor.row >= 0 && neighbor.row < GridResolution &&
                neighbor.col >= 0 && neighbor.col < GridResolution)
            {
                GridNode& neighbor_info = grid_nodes[neighbor.row][neighbor.col];
                if (grid[neighbor.row][neighbor.col])
                {
                    continue; // not a neighbor; wall
                }
                if (neighbor_info.state == GNS_CHECKED)
                {
                    continue;
                }

                const uint32_t neighbor_score = score + neighbor_dist;
                if (neighbor_info.score > neighbor_score)
                {
                    if (neighbor_info.state == GNS_NOT_FOUND)
                    {
                        ++num_discovered;
                    }

                    grid_nodes[neighbor.row][neighbor.col] = GridNode{ neighbor_score, (int8_t)i, GNS_FOUND };
                    prediction.Push(ScoredCoord{ neighbor_score + heuristic(neighbor), neighbor });
                }
            }
        }
    }

    if (result.config == SoundPropagation::PRD_FULL)
    {
        cache->grid_score = std::make_shared<GeometryGridScore>(grid_nodes);
    }

    if (prediction.Empty())
    {
        if (result.config == SoundPropagation::PRD_FULL)
        {
            result.intersections.clear();
        }
        cache->grid_result[receiver_coord.row][receiver_coord.col] = 0.f;
        return 0.f;
    }

    Coord next_coord = prediction.Top().second;
    const float distance = nMath::Max(FLT_EPSILON, grid_nodes[next_coord.row][next_coord.col].score / (1000.f * GridCellsPerMeter));
    const float geometric_attenuation = nMath::Min(1.f, 1.f / distance);

    cache->grid_result[receiver_coord.row][receiver_coord.col] = geometric_attenuation;

    if (result.config == SoundPropagation::PRD_FULL)
    {
        result.intersections.clear();

        const int grid_half = (int)GridResolution / 2;
        Coord coord = next_coord;
        while (grid_nodes[coord.row][coord.col].link_index >= 0)
        {
            GridNode& coord_info = grid_nodes[coord.row][coord.col];
            Coord incoming_coord = coord;
            incoming_coord.row -= neighbors[coord_info.link_index].row;
            incoming_coord.col -= neighbors[coord_info.link_index].col;
            const nMath::LineSegment&& segment{
                { ((float)coord.col - grid_half) / GridCellsPerMeter, ((float)coord.row - grid_half) / GridCellsPerMeter, 0.f },
                { ((float)incoming_coord.col - grid_half) / GridCellsPerMeter, ((float)incoming_coord.row - grid_half) / GridCellsPerMeter, 0.f }
            };
            result.intersections.emplace_back(segment);
            coord = incoming_coord;
        }
    }
    return geometric_attenuation;
}

void PlannerAStar::Simulate(PropagationResult& result, const nMath::Vector& _receiver, const float _time_ms) const
{
    (void)_time_ms;
    std::shared_ptr<AStarSimulateCache> cache = nullptr;
    if (result.cache == nullptr)
    {
        cache = std::make_shared<AStarSimulateCache>();
        std::fill_n(&cache->grid_result[0][0], GridResolution * GridResolution, -1.f);
        result.cache = std::static_pointer_cast<PropagationSimulationCache>(cache);
    }
    else
    {
        cache = std::dynamic_pointer_cast<AStarSimulateCache>(result.cache);
    }

    const int grid_half = (int)GridResolution / 2;
    nMath::Vector grid_receiver = _receiver * (float)GridCellsPerMeter + nMath::Vector{ (float)grid_half, (float)grid_half };

    if (grid_receiver.x < 0 || grid_receiver.y < 0 ||
        grid_receiver.x >= GridResolution || grid_receiver.y >= GridResolution)
    {
        result.gain = 0.f;
        return;
    };

    const Coord receiver_coord = Coord{ (int)grid_receiver.y, (int)grid_receiver.x };

    const nMath::Vector cell_receiver{ grid_receiver.x - (float)receiver_coord.col, grid_receiver.y - (float)receiver_coord.row, 0.f };

    float bary[3];
    Coord coord_tri[3];
    // Find the three cell coordinates closest to the receiver.
    if (cell_receiver.x < 0.5f)
    {
        coord_tri[0] = { 0, 0 };
        coord_tri[1] = { 1, 0 };
        if (cell_receiver.y < 0.5f)
        {
            bary[1] = cell_receiver.y;
            bary[2] = cell_receiver.x;
            coord_tri[2] = { 0, 1 };
        }
        else
        {
            bary[2] = cell_receiver.x;
            bary[1] = cell_receiver.y - bary[2];
            coord_tri[2] = { 1, 1 };
        }
    }
    else
    {
        // To treat coord_tri[0] as 0, imagine the triangle being shifted left by 1 col.
        // This means the x/col coord is in the negative direction, but becuase we are 
        // guarunteed to be on or in the triangle, that component of the triangle needs a
        // weight of 1.f - x.
        coord_tri[0] = { 0, 1 }; // coord_tri[0] = { 0, 1 - 1 }; replace with to solve bary by hand.
        coord_tri[1] = { 1, 1 };
        if (cell_receiver.y < 0.5f)
        {
            bary[1] = cell_receiver.y;
            bary[2] = 1.f - cell_receiver.x;
            coord_tri[2] = { 0, 0 };
        }
        else
        {
            bary[2] = 1.f - cell_receiver.x;
            bary[1] = cell_receiver.y - bary[2];
            coord_tri[2] = { 1, 0 };
        }
    }

    bary[0] = 1.f - bary[1] - bary[2];

    float sum = 0.f;
    for (int i = 0; i < 3; ++i)
    {
        Coord test_coord = receiver_coord;
        test_coord.row += coord_tri[i].row;
        test_coord.col += coord_tri[i].col;
        if (test_coord.col < GridResolution &&
            test_coord.row < GridResolution)
        {
            float gain = cache->grid_result[test_coord.row][test_coord.col];
            if (gain < 0)
            {
                gain = FindAStarDiscrete(result, receiver_coord, cache);
            }
            sum += bary[i] * gain;
        }
    }

    result.gain = sum;
}

bool PlannerAStar::GridNodeSearched(std::shared_ptr<const PropagationSimulationCache> cache, int row, int col)
{
    if (std::shared_ptr<const AStarSimulateCache> astar_cache = std::dynamic_pointer_cast<const AStarSimulateCache>(cache))
    {
        return astar_cache->grid_score != nullptr &&
               (*astar_cache->grid_score)[row][col].state == GNS_CHECKED;
    }
    return false;
}