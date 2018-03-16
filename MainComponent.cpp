/*
  ==============================================================================
  SoundPropagation - main Juce class
  ==============================================================================
*/

#include "MainComponent.h"
#include "AtmosphericAbsorption.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <atomic>

struct Butterworth1Pole
{
    float a1;
    float b0;
    float b1;

    float x1;
    float y1;

    bool bypass;

    Butterworth1Pole() :
        x1(0.0),
        y1(0.0),
        bypass(false) {}

    void Initialize(double cutoff_frequency, double sample_rate)
    {
        const float blt_freq_warping = 1 / tan(M_PI * cutoff_frequency / sample_rate);

        const float a0 = 1 + blt_freq_warping;
        a1 = (1 - blt_freq_warping) / a0;
        b0 = 1 / a0;
        b1 = b0;
    }

    float process(float x)
    {
        if (!bypass)
        {
            y1 = b0*x + b1*x1 - a1*y1;
            x1 = x;

            return y1;
        }
        return x;
    }
};

class SoundEmitter
{
public:
    SoundEmitter() {}

    void SetPosition(const Vector3D<float>& _position)
    {
        position = _position;
    }

    const Vector3D<float>& GetPosition() const
    {
        return position;
    }

private:
    Vector3D<float> position;
};

struct LineSegment
{
    Vector3D<float> start;
    Vector3D<float> end;
};

class RoomGeometry
{
public:
    RoomGeometry()
    {
        walls.reserve(4);
        walls.emplace_back(LineSegment{ { -12.f,  8.f, 0.f },{ 12.f,  8.f, 0.f } });
        walls.emplace_back(LineSegment{ { 12.f,  8.f, 0.f },{ 12.f, -8.f, 0.f } });
        walls.emplace_back(LineSegment{ { 12.f, -8.f, 0.f },{ -12.f, -8.f, 0.f } });
        walls.emplace_back(LineSegment{ { -12.f, -8.f, 0.f },{ -12.f,  8.f, 0.f } });
    }

    bool Intesects(const LineSegment& _line) const
    {
        const Vector3D<float> line2 = _line.end - _line.start;
        for (const LineSegment& wall : walls)
        {
            Vector3D<float> wall2 = wall.end - wall.start;
            const float numerator = wall2.y * (_line.start.x - wall.start.x) - wall2.x * (_line.start.y - wall.start.y);
            const float denominator = line2.y * wall2.x - line2.x * wall2.y;
            if (denominator != 0.f)
            {
                float r = numerator / denominator;
                if (r >= 0.f && r <= 1.f)
                {
                    return true;
                }
            }
        }

        return false;
    }

    void Paint(Graphics& _g, const Rectangle<int> _bounds, const float _zoom_factor) const
    {
        const float min_extent = (float)jmin(_bounds.getWidth(), _bounds.getHeight());
        const Vector3D<float> center(min_extent / 2.f, min_extent / 2.f, 0.f);

        _g.setColour(Colour::fromRGB(0x77, 0x1c, 0x47));
        Path room;
        for (const LineSegment& wall : walls)
        {
            const Line<float> drawLine(
                wall.start.x * _zoom_factor + center.x,
                -wall.start.y * _zoom_factor + center.y,
                wall.end.x * _zoom_factor + center.x,
                -wall.end.y * _zoom_factor + center.y);
            room.addLineSegment(drawLine, 2.f);
        }
        _g.fillPath(room);
    }
private:
    std::vector<LineSegment> walls;
};

class MovingEmitter
{
public:
    MovingEmitter() :
        frequency(0.25f),
        radius(10.f),
        angle(0.f)
    {
        gain_left = 0.f;
        gain_right = 0.f;
        global_gain = 0.8f;
    }

    void Update(int32 _elapsedMs)
    {
        angle += 2 * (float)M_PI * (_elapsedMs * frequency) / 1000.f;
        Vector3D<float> new_position(cosf(angle), sinf(angle), 0.f);
        emitter.SetPosition(new_position);
        const float geometric_attenuation = jmin(1.f, 1.f / radius);
        const float normed_loudness = (float)M_SQRT1_2;
        gain_left.store(normed_loudness * geometric_attenuation * sqrtf(1.f - new_position.x));
        gain_right.store(normed_loudness * geometric_attenuation * sqrtf(1.f + new_position.x));
    }

    void Clip(const RoomGeometry& room)
    {
        const LineSegment to_listener = { { emitter.GetPosition().x * radius.load(), emitter.GetPosition().y * radius.load(), 0.f },{ 0.f, 0.f, 0.f } };
        if (room.Intesects(to_listener))
        {
            gain_left.store(0.f);
            gain_right.store(0.f);
        }
    }

    float Gain(const int32 channel) const
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

    void SetFrequency(const float& _frequency)
    {
        frequency.store(_frequency);
    }

    void SetGlobalGain(const float& _gain)
    {
        global_gain.store(_gain);
    }
    
    void SetRadius(const float& _radius)
    {
        radius.store(_radius);
    }
    
    void Paint(Graphics& _g, const Rectangle<int> _bounds, const float _zoom_factor)
    {        
        const float min_extent = (float)jmin(_bounds.getWidth(), _bounds.getHeight());
        const Vector3D<float> center(min_extent / 2.f, min_extent / 2.f, 0.f);

        _g.setColour(Colour::fromRGB(0xFF, 0xFF, 0xFF));
        _g.fillEllipse(center.x - 1.f, center.y - 1.f, 2, 2);

        const float scale = _zoom_factor * radius.load();
        const Vector3D<float>& emitter_pos = emitter.GetPosition();
        Vector3D<float> emitter_draw_pos(emitter_pos.x * scale + center.x, -emitter_pos.y * scale + center.y, 0.f);
        _g.fillEllipse(emitter_draw_pos.x - 1.f, emitter_draw_pos.y - 1.f, 2.5, 2.5);
    }
private:
    std::atomic<float> frequency;
    std::atomic<float> global_gain;
    std::atomic<float> radius;
    float angle;    
    std::atomic<float> gain_left;
    std::atomic<float> gain_right;
    SoundEmitter emitter;
};

//==============================================================================
MainComponent::MainComponent() :
    initialized(false),
    selected_test_buffer(0),
    sample_rate(0.f)
{
    start_time = Time::getMillisecondCounter();

    setAudioChannels(0, 2);
    setWantsKeyboardFocus(true);

    addAndMakeVisible(&slider_gain);
    slider_gain.setRange(0.0, 2.0, 0.05);
    slider_gain.setTextValueSuffix(" %");
    slider_gain.setValue(0.8);
    slider_gain.addListener(this);

    addAndMakeVisible(&label_gain);
    label_gain.setText("Gain", dontSendNotification);
    label_gain.attachToComponent(&slider_gain, true);

    addAndMakeVisible(&slider_freq);
    slider_freq.setRange(0.001, 10.0, 0.001);
    slider_freq.setTextValueSuffix(" Hz");
    slider_freq.setValue(0.25);
    slider_freq.setSkewFactorFromMidPoint(1.0);
    slider_freq.addListener(this);

    addAndMakeVisible(&label_freq);
    label_freq.setText("Freq", dontSendNotification);
    label_freq.attachToComponent(&slider_freq, true);

    addAndMakeVisible(&slider_radius);
    slider_radius.setRange(0.1, 25.0, 0.1);
    slider_radius.setTextValueSuffix(" m");
    slider_radius.setValue(10.0);
    slider_radius.addListener(this);

    addAndMakeVisible(&label_radius);
    label_radius.setText("Radius", dontSendNotification);
    label_radius.attachToComponent(&slider_radius, true);

    addAndMakeVisible(&group_atmosphere);
    group_atmosphere.setText("Atmosphere");
    
    slider_temperature.setBounds(20, 22, 198, 22);
    slider_temperature.setRange(-20.0, 120.0, 1.0);
    slider_temperature.setTextValueSuffix(" F");
    slider_temperature.setValue(60.0);
    slider_temperature.addListener(this);
    group_atmosphere.addAndMakeVisible(&slider_temperature);

    slider_humidity.setBounds(20, 44, 198, 22);
    slider_humidity.setRange(1, 99.9, 0.50);
    slider_humidity.setTextValueSuffix(" %H");
    slider_humidity.setValue(60.0);
    slider_humidity.addListener(this);
    group_atmosphere.addAndMakeVisible(&slider_humidity);

    slider_pressure.setBounds(20, 66, 198, 22);
    slider_pressure.setRange(
        AtmosphericAbsorption::kPressureEverestLevelPascals,
        AtmosphericAbsorption::kPressureDeadSeaRecordLevelPascals,
        25.0);
    slider_pressure.setTextValueSuffix(" kPa");
    slider_pressure.setValue(AtmosphericAbsorption::kPressureSeaLevelPascals);
    slider_pressure.setSkewFactorFromMidPoint(AtmosphericAbsorption::kPressureSeaLevelPascals);
    slider_pressure.addListener(this);
    group_atmosphere.addAndMakeVisible(&slider_pressure);

    label_cutoff.setText("Cuttoff Freq", dontSendNotification);
    label_cutoff.setBounds(12, 88, 198, 22);
    group_atmosphere.addAndMakeVisible(&label_cutoff);

    addAndMakeVisible(&combo_selected_sound);

    // Make sure you set the size of the component after
    // you add any child components.
    setSize (800, 600);

    atmospheric_filters[0] = std::make_unique<Butterworth1Pole>();
    atmospheric_filters[0]->bypass = true;
    atmospheric_filters[1] = std::make_unique<Butterworth1Pole>();
    atmospheric_filters[1]->bypass = true;
    moving_emitter = std::make_unique<MovingEmitter>();
    room = std::make_unique<RoomGeometry>();    
}

MainComponent::~MainComponent()
{
    // This shuts down the GL system and stops the rendering calls.
    shutdownOpenGL();
    shutdownAudio();
}

namespace SoundBufferHelper
{
    void LoadFromFile(SoundBuffer& buffer, AudioFormatManager& format_manager, const char* file_name)
    {
        File file = File::getCurrentWorkingDirectory().getChildFile(file_name);
        std::unique_ptr<AudioFormatReader> reader(format_manager.createReaderFor(file));

        if (reader != nullptr)
        {
            buffer.buffer = std::make_unique<AudioSampleBuffer>();
            buffer.buffer->setSize(reader->numChannels, reader->lengthInSamples);
            reader->read(buffer.buffer.get(),
                0,
                reader->lengthInSamples,
                0,
                true,
                true);
        }
    }
}

//==============================================================================
void MainComponent::initialise()
{
    {
        AudioFormatManager format_manager; format_manager.registerBasicFormats();
               
        SoundBufferHelper::LoadFromFile(test_buffers[0], format_manager, "..\\..\\..\\lake_mono_2chnl.wav");
        strcpy(test_buffers[0].name, "Lake");
        combo_selected_sound.addItem(test_buffers[0].name, 1);
        SoundBufferHelper::LoadFromFile(test_buffers[1], format_manager, "..\\..\\..\\test_tones.wav");
        strcpy(test_buffers[1].name, "Two Tones");
        combo_selected_sound.addItem(test_buffers[1].name, 2);
    }

    {
        MessageManagerLock lock;
        combo_selected_sound.setSelectedId(1);
        combo_selected_sound.addListener(this);

        addAndMakeVisible(&label_selected_sound);
        label_selected_sound.setText("Source", dontSendNotification);
        label_selected_sound.attachToComponent(&combo_selected_sound, true);
    }    

    startTimerHz(60);
    initialized = true;
}

void MainComponent::shutdown()
{
    // Free any GL objects created for rendering here.
}

void MainComponent::render()
{
    // This clears the context with a black background.
    OpenGLHelpers::clear (Colours::black);

    // Add your rendering code here...
}

// AudioAppComponent
void MainComponent::setAudioChannels(int numInputChannels, int numOutputChannels, const XmlElement* const xml)
{
    String audioError = deviceManager.initialise(numInputChannels, numOutputChannels, xml, true);
    jassert(audioError.isEmpty());

    deviceManager.addAudioCallback(&audioSourcePlayer);
    audioSourcePlayer.setSource(this);
}

void MainComponent::shutdownAudio()
{
    audioSourcePlayer.setSource(nullptr);
    deviceManager.removeAudioCallback(&audioSourcePlayer);
    deviceManager.closeAudioDevice();
}

//==============================================================================
void MainComponent::paint (Graphics& _g)
{
    // You can add your component specific drawing code here!
    // This will draw over the top of the openGL background.
    const Rectangle<int> bounds = _g.getClipBounds();
    const float zoom_factor = 10.f;
    room->Paint(_g, bounds, zoom_factor);
    moving_emitter->Paint(_g, bounds, zoom_factor);
}

void MainComponent::resized()
{
    // This is called when the MainContentComponent is resized.
    // If you add any child components, this is where you should
    // update their positions.

    int32 new_width = getWidth();

    combo_selected_sound.setBounds(new_width - 202, 200 - 24, 200, 20);

    slider_gain.setBounds(new_width - 202, 200, 200, 20);
    slider_freq.setBounds(new_width - 202, 200 + 22, 200, 20);
    slider_radius.setBounds(new_width - 202, 200 + 46, 200, 20);
    group_atmosphere.setBounds(new_width - 244, 200 + 88, 238, 160);
}

// Audio Component
void MainComponent::prepareToPlay(int samplesPerBlockExpected,
    double sampleRate)
{
    sample_rate = (float)sampleRate;
}

void MainComponent::releaseResources()
{
}

void MainComponent::getNextAudioBlock(const AudioSourceChannelInfo& bufferToFill)
{
    bufferToFill.clearActiveBufferRegion();
    if (!initialized)
    {
        return;
    }
    const int channels = bufferToFill.buffer->getNumChannels();

    int samples_remaining = bufferToFill.numSamples;
    int sample_offset = bufferToFill.startSample;

    SoundBuffer& buffer_info = test_buffers[selected_test_buffer];
    AudioSampleBuffer* buffer = buffer_info.buffer.get();
    if (buffer == nullptr ||
        buffer->getNumChannels() <= 0)
    {
        return;
    }
    int& buffer_index = buffer_info.index;

    const float gain_left = moving_emitter->Gain(0);
    const float gain_right = moving_emitter->Gain(1);    

    while (samples_remaining > 0)
    {
        int buffer_samples_remaining = buffer->getNumSamples() - buffer_index;
        int samples_to_write = jmin(samples_remaining, buffer_samples_remaining);

        for (int channel = 0; channel < channels; ++channel)
        {
            Butterworth1Pole& lpf = *atmospheric_filters[channel].get();
            const float *read_pos = buffer->getReadPointer(channel, buffer_index);
            float *write_pos = buffer->getWritePointer(channel, buffer_index);

            for (int s = 0; s < samples_to_write; ++s, ++read_pos, ++write_pos)
            {
                *write_pos = lpf.process(*read_pos);
            }

            bufferToFill.buffer->addFrom(channel,
                sample_offset,
                *buffer,
                channel,
                buffer_index,
                samples_to_write,
                (channel == 0) ? gain_left : gain_right);
        }

        samples_remaining -= samples_to_write;
        sample_offset += samples_to_write;
        buffer_index += samples_to_write;

        if (buffer_index >= buffer->getNumSamples())
        {
            buffer_index = 0;
        }
    }

}

// Animated Component
void MainComponent::update()
{
    int32 frame_time = Time::getMillisecondCounter();
    moving_emitter->Update(frame_time - start_time);
    moving_emitter->Clip(*room.get());
    start_time = frame_time;
}

void MainComponent::timerCallback()
{
    update();
    repaint();
}

// UI

void MainComponent::sliderValueChanged(Slider* slider)
{
    if (slider == &slider_gain)
    {
        moving_emitter->SetGlobalGain((float)slider_gain.getValue());
    }
    else if (slider == &slider_freq)
    {
        moving_emitter->SetFrequency((float)slider_freq.getValue());
    }
    else if (slider == &slider_radius ||
             slider == &slider_temperature ||
             slider == &slider_humidity ||
             slider == &slider_pressure)
    {
        const float next_radius = (float)slider_radius.getValue();
        const double next_temperature = slider_temperature.getValue();
        const double next_humidity = slider_humidity.getValue();
        const double next_pressure = slider_pressure.getValue();
        moving_emitter->SetRadius(next_radius);
        const double filter_gain_at_cutoff_db = -3.;
        const double target_atmospheric_coefficient = -filter_gain_at_cutoff_db / (double)next_radius;
        const float cuttoff_frequency = (float)AtmosphericAbsorption::Frequency(target_atmospheric_coefficient, next_humidity, next_temperature, next_pressure);
        atmospheric_filters[0]->Initialize(cuttoff_frequency, sample_rate);
        atmospheric_filters[0]->bypass = cuttoff_frequency > sample_rate / 2.f;
        atmospheric_filters[1]->Initialize(cuttoff_frequency, sample_rate);
        atmospheric_filters[1]->bypass = cuttoff_frequency > sample_rate / 2.f;
        
        char b[256];
        sprintf_s(b, "Cutoff %.1f Hz", cuttoff_frequency);
        label_cutoff.setText(b, dontSendNotification);
    }
}

void MainComponent::comboBoxChanged(ComboBox* comboBoxThatHasChanged)
{
    if (comboBoxThatHasChanged == &combo_selected_sound)
    {
        const uint32 next_id = combo_selected_sound.getSelectedId();
        if (next_id > 0)
        {
            selected_test_buffer = next_id - 1;
        }
    }
}