// Vector
// Author - Nic Taylor
#pragma once
#include <math.h>

namespace nMath
{
    struct Complex
    {
        float real;
        float imaginary;
    };

    struct ComplexExp
    {
        float amplitude;
        float phase;
    };

    Complex ToComplex(const ComplexExp& exp_)
    {
        return Complex{ exp_.amplitude  * cosf(exp_.phase),
                exp_.amplitude  * sinf(exp_.phase) };
    }

    Complex operator+ (const Complex& lhs, const Complex& rhs)
    {
        const Complex c = { lhs.real + rhs.real, lhs.imaginary + rhs.imaginary };
        return c;
    }

    float Magnitude(const Complex& c_)
    {
        return sqrtf(c_.real * c_.real + c_.imaginary * c_.imaginary);
    }

    float Arg(const Complex& c_)
    {
        return atan2f(c_.imaginary, c_.real);
    }

    ComplexExp ToComplexExp(const Complex& c_)
    {
        return ComplexExp{
            Magnitude(c_),
            Arg(c_)
        };
    }
}
