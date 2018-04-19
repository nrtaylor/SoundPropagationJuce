// Vector
// Author - Nic Taylor
#pragma once
#include <math.h>

namespace nMath
{
    struct Vector
    {
        float x, y, z;
    };

    struct LineSegment
    {
        Vector start, end;
    };

    // Helper functions
    inline float Dot(const Vector& lhs, const Vector& rhs)
    {
        return lhs.x*rhs.x + lhs.y*rhs.y + lhs.z*rhs.z;
    }

    inline Vector operator+ (const Vector& lhs, const Vector& rhs)
    {
        Vector v = { lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z };
        return v;
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
        const nMath::Vector test_dir = test.end - test.start;
        const nMath::Vector v_dir = v.end - v.start;

        const float numerator = v_dir.y * (test.start.x - v.start.x) - v_dir.x * (test.start.y - v.start.y);
        const float denominator = test_dir.y * v_dir.x - test_dir.x * v_dir.y;

        if (denominator != 0.f)
        {
            float r = numerator / denominator;
            if (r >= 0.f && r <= 1.f)
            {
                //const float numerator2 = test_dir.y * (v.start.x - test.start.x) - test_dir.x * (v.start.y - test.start.y);
                //const float denominator2 = v_dir.y * test_dir.x - v_dir.x * test_dir.y;
                //const float r2 = (test.start.x - v.start.x + r * test_dir.x) / v_dir.x;
                const float numerator2 = test_dir.x * (v.start.y - test.start.y) - test_dir.y * (v.start.x - test.start.x);
                const float denominator2 = denominator;
                r = numerator2 / denominator2;

                if (r >= 0.f && r <= 1.f)
                {
                    return true;
                }
            }
        }
        return false;
    }
}
