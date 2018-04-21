// Room Geometry - class representing set of "Walls" to simulate sound propagation
// Author - Nic Taylor
#include "RoomGeometry.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <array>

typedef signed int int32; // TODO: PCH

nMath::Vector MovingEmitter::Update(const signed int _elapsedMs)
{
    angle += 2 * (float)M_PI * (_elapsedMs * frequency) / 1000.f;
    nMath::Vector new_position{ cosf(angle), sinf(angle), 0.f };
    pan_amount = new_position.x;

    const float emitter_radius = radius.load();
    emitter_pos = { new_position.x*emitter_radius, new_position.y*emitter_radius, 0.f };

    return emitter_pos;
}

// gain left/right should be pulled out as a process
void MovingEmitter::ComputeGain(const float new_gain)
{
    // Factor such that panned hard left/right will have the same rms as pan center.
    const float normed_loudness = new_gain * (float)M_SQRT1_2;
    gain_left.store(normed_loudness * sqrtf(1.f - pan_amount));
    gain_right.store(normed_loudness * sqrtf(1.f + pan_amount));
}

float MovingEmitter::Gain(const signed int channel) const
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

void RayCastCollector::Reset()
{    
    ray_casts.clear();
}

void RayCastCollector::Add(const nMath::LineSegment& ray_cast)
{
    ray_casts.emplace_back(ray_cast);
}

auto RayCastCollector::RayCasts() -> RayCastCollector::ConstRefLineSegments const
{
    return ray_casts;
}

RoomGeometry::RoomGeometry() :
        bounding_box{ {},{} }
{
    walls.reserve(4);

    grid = std::make_unique<GeometryGrid>();
    std::fill_n(&(*grid)[0][0], GridResolution * GridResolution, false);
}

void RoomGeometry::AddWall(const nMath::Vector start, const nMath::Vector end)
{
    walls.emplace_back(nMath::LineSegment{ start, end });
    if (walls.size() == 1)
    {
        bounding_box = nMath::LineSegment{
            { nMath::Min(start.x, end.x), nMath::Min(start.y,end.y), 0.f },
            { nMath::Max(start.x, end.x), nMath::Max(start.y,end.y), 0.f } };
    }
    else
    {
        if (nMath::Min(start.x, end.x) < bounding_box.start.x)
        {
            bounding_box.start.x = nMath::Min(start.x, end.x);
        }
        if (nMath::Max(start.x, end.x) > bounding_box.end.x)
        {
            bounding_box.end.x = nMath::Max(start.x, end.x);
        }
        if (nMath::Min(start.y, end.y) < bounding_box.start.y)
        {
            bounding_box.start.y = nMath::Min(start.y, end.y);
        }
        if (nMath::Max(start.y, end.y) > bounding_box.end.y)
        {
            bounding_box.end.y = nMath::Max(start.y, end.y);
        }
    }

    // Update Grid
    const int grid_half = (int)GridResolution / 2;
    nMath::LineSegment grid_line{
        start * (float)GridCellsPerMeter + nMath::Vector{ (float)grid_half, (float)grid_half },
        end * (float)GridCellsPerMeter + nMath::Vector{ (float)grid_half, (float)grid_half }
    };

    if (fabsf(end.x - start.x) > fabsf(end.y - start.y))
    {
        if (end.x < start.x)
        {
            std::swap(grid_line.start, grid_line.end);
        }

        const int grid_start = nMath::Max(0, (int)grid_line.start.x);
        const int grid_end = nMath::Min((int)GridResolution, (int)ceilf(grid_line.end.x));

        const float inv_delta = 1.f / (grid_line.start.x - grid_line.end.x);
        const float delta_y = grid_line.start.y - grid_line.end.y;
        int prev_grid_y = -1;
        for (int i = grid_start; i <= grid_end; ++i)
        {
            float t = ((float)i - grid_line.end.x) * inv_delta;
            t = nMath::Max(0.f, nMath::Min(1.f, t)); // end points
            const float y = grid_line.end.y + t * delta_y; // prevent rounding error when start.y == end.y
            const int grid_y = nMath::Min<int>(nMath::Max(0,(int)y), (int)GridResolution - 1);
            if (prev_grid_y >= 0)
            {
                for (int j = nMath::Min(prev_grid_y, grid_y); j <= nMath::Max(prev_grid_y, grid_y); ++j)
                {
                    (*grid)[j][i - 1] = true;
                }
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
        const int grid_end = nMath::Min((int)GridResolution, (int)ceilf(grid_line.end.y));

        const float inv_delta = 1.f / (grid_line.start.y - grid_line.end.y);
        const float delta_x = grid_line.start.x - grid_line.end.x;
        int prev_grid_x = -1;
        for (int i = grid_start; i <= grid_end; ++i)
        {
            float t = ((float)i - grid_line.end.y) * inv_delta;
            t = nMath::Max(0.f, nMath::Min(1.f, t)); // end points
            const float x = grid_line.end.x + t * delta_x; // prevent rounding error when start.y == end.y
            const int grid_x = nMath::Min<int>(nMath::Max(0, (int)x), (int)GridResolution - 1);
            if (prev_grid_x >= 0)
            {
                for (int j = nMath::Min(prev_grid_x, grid_x); j <= nMath::Max(prev_grid_x, grid_x); ++j)
                {
                    (*grid)[i - 1][j] = true;
                }
            }
            prev_grid_x = grid_x;
        }
    }
}

template<bool capture_debug>
float RoomGeometry::Simulate(const nMath::Vector& source, const nMath::Vector& receiver, const SoundPropagation::MethodType method) const
{
    switch (method)
    {
    case SoundPropagation::Method_SpecularLOS:
        return SimulateSpecularLOS<capture_debug>(source, receiver);
    case SoundPropagation::Method_RayCasts:
        return SimulateRayCasts<capture_debug>(source, receiver);
    case SoundPropagation::Method_Pathfinding:
        return SimulateAStar<capture_debug>(source, receiver);
    default:
        return 0.f;
    }
}

void RoomGeometry::SwapCollector(std::unique_ptr<RayCastCollector>& collector)
{
    std::swap(ray_cast_collector, collector);
}

template<>
void RoomGeometry::CaptureDebug<false>(const nMath::LineSegment& _line) const
{
    (void)_line;
}

template<>
void RoomGeometry::CaptureDebug<true>(const nMath::LineSegment& _line) const
{
    if (ray_cast_collector != nullptr)
    {
        ray_cast_collector->Add(_line);
    }
}

template<bool capture_debug>
bool RoomGeometry::Intersects(const nMath::LineSegment& _line) const
{
    CaptureDebug<capture_debug>(_line);

    if (nMath::Min(_line.start.x, _line.end.x) > bounding_box.end.x ||
        nMath::Max(_line.start.x, _line.end.x) < bounding_box.start.x ||
        nMath::Min(_line.start.y, _line.end.y) > bounding_box.end.y ||
        nMath::Max(_line.start.y, _line.end.y) < bounding_box.start.y)
    {
        return false;
    }

    for (const nMath::LineSegment& wall : walls)
    {
        if (nMath::Intersect2D(wall, _line))
        {
            return true;
        }
    }

    return false;
}

template<bool capture_debug>
float RoomGeometry::SimulateSpecularLOS(const nMath::Vector& source, const nMath::Vector& receiver) const
{
    if (Intersects<capture_debug>(nMath::LineSegment{ source, receiver }))
    {
        return 0.f;
    }

    static const float near_field_distance = 0.75f; // around 440 hz

    const float distance = nMath::Length(source - receiver);
    if (distance < near_field_distance)
    {
        return 1.f;
    }
    const float geometric_attenuation = nMath::Min(1.f, 1.f / distance);
    return geometric_attenuation;
}

template<bool capture_debug>
float RoomGeometry::SimulateRayCasts(const nMath::Vector& source, const nMath::Vector& receiver) const
{
    nMath::Vector direction = source - receiver;
    float distance = nMath::Length(direction);
    if (Intersects<capture_debug>(nMath::LineSegment{ source, receiver }))
    {
        if (distance > FLT_EPSILON)
        {
            direction = direction / distance;
            nMath::Vector ray_orth = { -direction.y, direction.x, 0.f };
            nMath::Vector ray_orth_inv = -1.f * ray_orth;
            if ((!Intersects<capture_debug>(nMath::LineSegment{ ray_orth + receiver, receiver }) &&
                !Intersects<capture_debug>(nMath::LineSegment{ ray_orth + receiver, source })) ||
                (!Intersects<capture_debug>(nMath::LineSegment{ ray_orth_inv + receiver, receiver }) &&
                    !Intersects<capture_debug>(nMath::LineSegment{ ray_orth_inv + receiver, source })))
            {
                distance += 1.f;
            }
            else if ((!Intersects<capture_debug>(nMath::LineSegment{ ray_orth + source, source }) &&
                !Intersects<capture_debug>(nMath::LineSegment{ ray_orth + source, receiver })) ||
                (!Intersects<capture_debug>(nMath::LineSegment{ ray_orth_inv + source, source }) &&
                    !Intersects<capture_debug>(nMath::LineSegment{ ray_orth_inv + source, receiver })))
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
    const float geometric_attenuation = nMath::Min(1.f, 1.f / distance);
    return geometric_attenuation;
}

template<bool capture_debug>
float RoomGeometry::SimulateAStar(const nMath::Vector& source, const nMath::Vector& receiver) const
{
    struct Coord
    {
        int row;
        int col;
    };
    //auto coord_hash = [](const Coord& c)-> std::size_t
    //{
    //    return std::hash<uint64_t>()(((uint64_t)c.row << (uint64_t)32) | c.col);
    //};

    struct coord_hash
    {
        std::size_t operator()(const Coord& c) const
        {
            return std::hash<uint64_t>()(((uint64_t)c.row << (uint64_t)32) | c.col);
        }
    };

    struct coord_eq
    {
        bool operator()(const Coord& lhs, const Coord& rhs) const
        {
            return lhs.col == rhs.col && lhs.row == rhs.row;
        }
    };

    const int grid_half = (int)GridResolution / 2;
    nMath::Vector grid_source = source * (float)GridCellsPerMeter + nMath::Vector{ (float)grid_half, (float)grid_half };
    nMath::Vector grid_receiver = receiver * (float)GridCellsPerMeter + nMath::Vector{ (float)grid_half, (float)grid_half };

    if (grid_source.x < 0 || grid_source.y < 0 ||
        grid_receiver.x < 0 || grid_receiver.y < 0 ||
        grid_source.x >= GridResolution || grid_source.y >= GridResolution ||
        grid_receiver.x >= GridResolution || grid_receiver.y >= GridResolution)
    {
        return 0.f;
    };

    const Coord source_coord = Coord{(int)grid_source.x, (int)grid_source.y};
    const Coord receiver_coord = Coord{ (int)grid_receiver.x, (int)grid_receiver.y };

    auto heuristic = [&receiver_coord](const Coord& c)
    {
        return sqrtf((float)((c.col - receiver_coord.col)*(c.col - receiver_coord.col) +
            (c.row - receiver_coord.row)*(c.row - receiver_coord.row)));
    };
    
    std::unordered_set<Coord, coord_hash, coord_eq> discovered; discovered.reserve(2 * GridResolution);
    std::unordered_set<Coord, coord_hash, coord_eq> checked; checked.reserve(2 * GridResolution);
    std::unordered_map<Coord, Coord, coord_hash, coord_eq> incoming; incoming.reserve(2 * GridResolution);
    std::unordered_map<Coord, float, coord_hash, coord_eq> scores; scores.reserve(2 * GridResolution);
    typedef std::pair<float, Coord> ScoredCoord;
    auto compareScore = [](const ScoredCoord& lhs, const ScoredCoord& rhs)
    {
        return lhs.first > rhs.first;
    };
    std::priority_queue<ScoredCoord, std::vector<ScoredCoord>, decltype(compareScore)> prediction(compareScore);
    prediction.push(ScoredCoord(heuristic(source_coord), source_coord));

    discovered.insert(source_coord);
    scores[source_coord] = 0;
    
    static Coord neighbors[] = {
        {-1, -1},
        {-1,  0},
        {-1,  1},
        { 0, -1},
        { 0,  1},
        { 1, -1 },
        { 1,  0 },
        { 1,  1 },
    };

    while (!discovered.empty())
    {
        Coord next_coord = prediction.top().second;
        if (checked.find(next_coord) != checked.end())
        {
            prediction.pop();
            continue;
        }

        if (next_coord.col == receiver_coord.col &&
            next_coord.row == receiver_coord.row)
        {
            break;
        }
        prediction.pop();

        discovered.erase(next_coord);
        checked.insert(next_coord);

        float score = scores[next_coord];
        for (int i = 0; i < 8; ++i)
        {
            Coord neighbor = neighbors[i];
            const float neighbor_dist = (neighbor.row == 0 || neighbor.col == 0) ? 1.f : (float)M_SQRT2;
            neighbor.row += next_coord.row;
            neighbor.col += next_coord.col;
            if (neighbor.row >= 0 && neighbor.row < GridResolution &&
                neighbor.col >= 0 && neighbor.col < GridResolution)
            {
                if (checked.find(neighbor) != checked.end())
                {
                    continue;
                }

                discovered.insert(neighbor);
                float neighbor_score = score + neighbor_dist;
                if (scores.find(neighbor) == scores.end() ||
                    scores[neighbor] > neighbor_score)
                {
                    incoming[neighbor] = next_coord;
                    scores[neighbor] = neighbor_score;
                    prediction.emplace(neighbor_score + heuristic(neighbor), neighbor);
                }
            }
        }
    }

    Coord next_coord = prediction.top().second;
    return heuristic(source_coord) / scores[next_coord];
}
