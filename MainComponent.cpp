/*
  ==============================================================================
  SoundPropagation - main Juce class
  ==============================================================================
*/

#include "MainComponent.h"
#include <math.h>
#include <atomic>

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
        angle += 2 * MathConstants<float>::pi * (_elapsedMs * frequency) / 1000.f;
        Vector3D<float> new_position(cosf(angle), sinf(angle), 0.f);
        emitter.SetPosition(new_position);
        gain_left.store(0.5 * sqrtf(1.f - new_position.x));
        gain_right.store(0.5 * sqrtf(1.f + new_position.x));
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
    
    void Paint(Graphics& g)
    {
        const Rectangle<int> rect = g.getClipBounds();
        float min_extent = (float)jmin(rect.getWidth(), rect.getHeight());
        
        Vector3D<float> center(min_extent / 2.f, min_extent / 2.f, 0.f);

        g.setColour(Colour::fromRGB(0xFF, 0xFF, 0xFF));
        g.fillEllipse(center.x - 1.f, center.y - 1.f, 2, 2);

        const float scale = 10.f * radius.load();
        const Vector3D<float>& emitter_pos = emitter.GetPosition();
        Vector3D<float> emitter_draw_pos(emitter_pos.x * scale + center.x, -emitter_pos.y * scale + center.y, 0.f);
        g.fillEllipse(emitter_draw_pos.x - 1.f, emitter_draw_pos.y - 1.f, 2.5, 2.5);
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
    test_sound_buffer_index(0)
{
    start_time = Time::getMillisecondCounter();

    setAudioChannels(0, 2);
    setWantsKeyboardFocus(true);

    addAndMakeVisible(&slider_gain);
    slider_gain.setRange(0.0, 1.0, 0.05);
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

    // Make sure you set the size of the component after
    // you add any child components.
    setSize (800, 600);

    moving_emitter = std::make_unique<MovingEmitter>();
    test_sound_buffer = std::make_unique<AudioSampleBuffer>();
}

MainComponent::~MainComponent()
{
    // This shuts down the GL system and stops the rendering calls.
    shutdownOpenGL();
    shutdownAudio();
}

//==============================================================================
void MainComponent::initialise()
{
    {
        AudioFormatManager format_manager; format_manager.registerBasicFormats();
        File file = File::getCurrentWorkingDirectory().getChildFile("..\\..\\..\\test_tones.wav");
        std::unique_ptr<AudioFormatReader> reader(format_manager.createReaderFor(file));

        if (reader != nullptr)
        {            
            test_sound_buffer->setSize(reader->numChannels, reader->lengthInSamples);
            reader->read(test_sound_buffer.get(),
                0,
                reader->lengthInSamples,
                0,
                true,
                true);
        }
    }

    startTimerHz(60);
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
void MainComponent::paint (Graphics& g)
{
    // You can add your component specific drawing code here!
    // This will draw over the top of the openGL background.
    
    moving_emitter->Paint(g);
}

void MainComponent::resized()
{
    // This is called when the MainContentComponent is resized.
    // If you add any child components, this is where you should
    // update their positions.

    int32 new_width = getWidth();

    slider_gain.setBounds(new_width - 202, 200, 200, 20);
    slider_freq.setBounds(new_width - 202, 200 + 22, 200, 20);
    slider_radius.setBounds(new_width - 202, 200 + 44, 200, 20);
}

// Audio Component
void MainComponent::prepareToPlay(int samplesPerBlockExpected,
    double sampleRate)
{
    return;
}

void MainComponent::releaseResources()
{
}

void MainComponent::getNextAudioBlock(const AudioSourceChannelInfo& bufferToFill)
{
    bufferToFill.clearActiveBufferRegion();
    const int channels = bufferToFill.buffer->getNumChannels();

    int samples_remaining = bufferToFill.numSamples;
    int sample_offset = bufferToFill.startSample;

    AudioSampleBuffer* buffer = test_sound_buffer.get();
    if (buffer == nullptr ||
        buffer->getNumChannels() <= 0)
    {
        return;
    }
    int& buffer_index = test_sound_buffer_index;

    const float gain_left = moving_emitter->Gain(0);
    const float gain_right = moving_emitter->Gain(1);

    while (samples_remaining > 0)
    {
        int buffer_samples_remaining = buffer->getNumSamples() - buffer_index;
        int samples_to_write = jmin(samples_remaining, buffer_samples_remaining);

        for (int channel = 0; channel < channels; ++channel)
        {
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
    else if (slider == &slider_radius)
    {
        moving_emitter->SetRadius((float)slider_radius.getValue());
    }
}