// RandomTests
// Author - Nic Taylor
#pragma once
#include <math.h>

namespace RandomTest
{
    void RotateRect(nMath::Vector _offset, float width, float height, float _angle_degree)
    {
        const float angle = _angle_degree * (float)M_PI / 180.f;

        nMath::Vector ext1 = {
            cosf(angle) * width,
            sinf(angle) * width,
        };

        nMath::Vector ext2 = {
            -sinf(angle) * height,
            cosf(angle) * height,
        };

        nMath::Vector start = _offset - ext1 - ext2;
        nMath::Vector end = _offset - ext1 + ext2;

        const float theta = atan2f(end.x - start.x, end.y - start.y);
        const float theta2 = atan2f(ext2.x, ext2.y);

        jassert(fabsf(theta - theta2) < 10.f * FLT_EPSILON);

        nMath::Vector start_prime = nMath::Rotate2D(start, theta);
        nMath::Vector end_prime = nMath::Rotate2D(end, theta);

        nMath::Vector rect_center = nMath::Rotate2D(_offset, theta);
        nMath::Vector rect_ext1 = nMath::Rotate2D(_offset + ext1, theta) - rect_center;
        nMath::Vector rect_ext2 = nMath::Rotate2D(_offset + ext2, theta) - rect_center;

        jassert(fabsf(start_prime.x - end_prime.x) < 10.f * FLT_EPSILON);
    }

    void Test()
    {
        RandomTest::RotateRect(nMath::Vector{ 1.f, 1.5f, 0.f }, 2.f, 0.75f, 33.f);
        RandomTest::RotateRect(nMath::Vector{ 15.f, 1.5f, 0.f }, 2.f, 0.75f, 33.f);
        RandomTest::RotateRect(nMath::Vector{ 1.f, 1.5f, 0.f }, 2.f, 0.75f, 0.f);
    }
}