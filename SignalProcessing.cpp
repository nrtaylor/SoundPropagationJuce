// Signal Processing - simple DSP related structs and functions
// Author - Nic Taylor

#include "SignalProcessing.h"
#define _USE_MATH_DEFINES
#include <math.h>

using namespace NicDSP;

void Butterworth1Pole::Initialize(float cutoff_frequency, float sample_rate)
{
    const float blt_freq_warping = 1 / tanf((float)M_PI * cutoff_frequency / sample_rate);

    const float a0 = 1 + blt_freq_warping;
    a1 = (1 - blt_freq_warping) / a0;
    b0 = 1 / a0;
    b1 = b0;
}
