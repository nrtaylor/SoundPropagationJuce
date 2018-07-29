// NRT: JUCE Wrapper for AtmosphericAbsorption parameters.
#pragma once

#include "../JuceLibraryCode/JuceHeader.h"

class AtmosphericAbsorptionComponent : public Component
{
public:
    AtmosphericAbsorptionComponent();

    void SetDistance(const float _distance, const NotificationType notification);

    std::function<void(const float cuttoff_frequency)> on_coefficient_changed;
private:
    void HandleValueChanged(const NotificationType notification);

    Slider slider_temperature;
    Slider slider_humidity;
    Slider slider_pressure;
    Label label_cutoff;

    float distance;
};