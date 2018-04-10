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
}
