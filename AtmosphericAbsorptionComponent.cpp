#include "AtmosphericAbsorptionComponent.h"
#include "AtmosphericAbsorption.h"

AtmosphericAbsorptionComponent::AtmosphericAbsorptionComponent()
    : distance(1.f)
{
    auto value_changed_func = [this] { HandleValueChanged(sendNotification); };

    const int slider_width = 212;    

    slider_temperature.setBounds(4, 0, slider_width, 22);
    slider_temperature.setRange(-20.0, 120.0, 1.0);
    slider_temperature.setTextValueSuffix(" F");
    slider_temperature.setValue(60.0);    
    slider_temperature.onValueChange = value_changed_func;
    addAndMakeVisible(&slider_temperature);

    slider_humidity.setBounds(4, 22, slider_width, 22);
    slider_humidity.setRange(1, 99.9, 0.50);
    slider_humidity.setTextValueSuffix(" %H");
    slider_humidity.setValue(60.0);
    addAndMakeVisible(&slider_humidity);
    slider_humidity.onValueChange = value_changed_func;

    slider_pressure.setBounds(4, 44, slider_width, 22);
    slider_pressure.setRange(
        AtmosphericAbsorption::kPressureEverestLevelPascals,
        AtmosphericAbsorption::kPressureDeadSeaRecordLevelPascals,
        25.0);
    slider_pressure.setTextValueSuffix(" kPa");
    slider_pressure.setValue(AtmosphericAbsorption::kPressureSeaLevelPascals);
    slider_pressure.setSkewFactorFromMidPoint(AtmosphericAbsorption::kPressureSeaLevelPascals);    
    addAndMakeVisible(&slider_pressure);
    slider_pressure.onValueChange = value_changed_func;

    label_cutoff.setText("Cuttoff Freq", dontSendNotification);
    label_cutoff.setBounds(0, 66, slider_width, 22);
    addAndMakeVisible(&label_cutoff);

    setSize(slider_width + 4, 88);

    HandleValueChanged(dontSendNotification);
}

void AtmosphericAbsorptionComponent::HandleValueChanged(const NotificationType notification)
{
    const double next_temperature = slider_temperature.getValue();
    const double next_humidity = slider_humidity.getValue();
    const double next_pressure = slider_pressure.getValue();
    const double filter_gain_at_cutoff_db = -3.;
    const double target_atmospheric_coefficient = -filter_gain_at_cutoff_db / (double)distance;
    const float cuttoff_frequency = (float)AtmosphericAbsorption::Frequency(target_atmospheric_coefficient, next_humidity, next_temperature, next_pressure);

    juce::String cutoff_text;
    cutoff_text << "Cutoff Freq:" <<  juce::String(cuttoff_frequency, 1) << " Hz";
    label_cutoff.setText(cutoff_text, dontSendNotification);

    if (notification != dontSendNotification)
    {
        if (on_coefficient_changed != nullptr)
        {
            on_coefficient_changed(cuttoff_frequency);
        }
    }
}

void AtmosphericAbsorptionComponent::SetDistance(const float _distance, const NotificationType notification)
{
    if (_distance != distance)
    {
        distance = _distance;
        HandleValueChanged(notification);
    }
}
