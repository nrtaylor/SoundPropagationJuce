// Vector
// Author - Nic Taylor
#pragma once
#include <math.h>

namespace nMath
{
    template <typename T>
    inline T Min(T lhs, T rhs)
    {
        return (lhs < rhs) ? lhs : rhs;
    }

    template <typename T>
    inline T Max(T lhs, T rhs)
    {
        return (lhs > rhs) ? lhs : rhs;
    }

    template <typename T>
    inline T MinClamped(T lhs, T rhs, T min_value)
    {
        return Max(min_value, Min(lhs, rhs));
    }

    template <typename T>
    inline T MaxClamped(T lhs, T rhs, T max_value)
    {
        return Min(max_value, Max(lhs, rhs));
    }

    struct Vector
    {
        float x, y, z;
    };

    struct LineSegment
    {
        Vector start, end;
    };

    // Helper functions
    inline bool operator== (const Vector& lhs, const Vector& rhs)
    {
        return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
    }

    inline float Dot(const Vector& lhs, const Vector& rhs)
    {
        return lhs.x*rhs.x + lhs.y*rhs.y + lhs.z*rhs.z;
    }

    inline Vector operator+ (const Vector& lhs, const Vector& rhs)
    {
        Vector v = { lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z };
        return v;
    }

    inline Vector operator+= (Vector& lhs, const Vector& rhs)
    {
        lhs = { lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z };
        return lhs;
    }

    inline Vector operator- (const Vector& lhs, const Vector& rhs)
    {
        Vector v = { lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z };
        return v;
    }

    inline Vector operator* (float c, const Vector& v)
    {
        Vector cv = { c*v.x, c*v.y, c*v.z };
        return cv;
    }

    inline Vector operator* (const Vector& v, float c)
    {
        Vector cv = { c*v.x, c*v.y, c*v.z };
        return cv;
    }

    inline Vector operator/ (const Vector& v, float c)
    {
        Vector cv = { v.x / c, v.y / c, v.z / c };
        return cv;
    }

    inline float LengthSquared(const Vector& v)
    {
        return Dot(v, v);
    }

    inline float Length(const Vector& v)
    {
        return sqrtf(Dot(v, v));
    }

    inline Vector Project(const LineSegment& segment, const Vector& v)
    {
        Vector projected = v - segment.start;
        Vector s_dir = segment.end - segment.start;
        const float proportion = Dot(projected, s_dir);

        if (proportion <= 0.f)
        {
            return segment.start;
        }
        const float length_sq = LengthSquared(s_dir);
        if (proportion >= length_sq)
        {
            return segment.end;
        }
        projected = (s_dir * (proportion / length_sq)) + segment.start;
        return projected;
    }

    inline Vector Rotate2D(const Vector& v, const float angle_rads)
    {
        return Vector{
            cosf(angle_rads) * v.x - sinf(angle_rads) * v.y,
            sinf(angle_rads) * v.x + cosf(angle_rads) * v.y,
            v.z
        };
    }

    inline bool Intersect2D(const LineSegment& v, const LineSegment& test)
    {
        if (nMath::Min(v.start.x, v.end.x) > nMath::Max(test.start.x, test.end.x) ||
            nMath::Max(v.start.x, v.end.x) < nMath::Min(test.start.x, test.end.x) ||
            nMath::Min(v.start.y, v.end.y) > nMath::Max(test.start.y, test.end.y) ||
            nMath::Max(v.start.y, v.end.y) < nMath::Min(test.start.y, test.end.y))
        {
            return false;
        }

        const nMath::Vector test_dir = test.end - test.start;
        const nMath::Vector v_dir = v.end - v.start;

        const float numerator = v_dir.y * (test.start.x - v.start.x) - v_dir.x * (test.start.y - v.start.y);
        const float denominator = test_dir.y * v_dir.x - test_dir.x * v_dir.y;

        if (denominator == 0.f)
        {
            return true; // colinear and overlapping
        }

        float r = numerator / denominator;
        if (r >= 0.f && r <= 1.f)
        {
            const float numerator2 = test_dir.x * (v.start.y - test.start.y) - test_dir.y * (v.start.x - test.start.x);                
            r = numerator2 / denominator;

            if (r >= 0.f && r <= 1.f)
            {
                return true;
            }
        }

        return false;
    }
}
