/*
  ==============================================================================
  SoundPropagation - main Juce class
  ==============================================================================
*/

#pragma once

#include "../JuceLibraryCode/JuceHeader.h"

class MovingEmitter;
class RoomGeometry;
struct Butterworth1Pole;
//==============================================================================
/*
    This component lives inside our window, and this is where you should put all
    your controls and content.
*/
class MainComponent   : public OpenGLAppComponent,
                        public AudioSource,
                        public Slider::Listener,
                        private Timer
{
public:
    //==============================================================================
    MainComponent();
    ~MainComponent();

    //==============================================================================
    void initialise() override;
    void shutdown() override;
    void render() override;

    //==============================================================================
    void paint (Graphics& g) override;
    void resized() override;

    //==============================================================================
    void prepareToPlay(int samplesPerBlockExpected,
        double sampleRate) override;
    void releaseResources() override;
    void getNextAudioBlock(const AudioSourceChannelInfo& bufferToFill) override;

    // AudioAppComponent
    AudioDeviceManager deviceManager;
    void setAudioChannels(int numInputChannels, int numOutputChannels, const XmlElement* const xml = nullptr);
    void shutdownAudio();

    // AnimatedComponent
    void update();

    // GUI
    void sliderValueChanged(Slider* slider) override;

private:
    //==============================================================================
    AudioSourcePlayer audioSourcePlayer;
    std::unique_ptr<MovingEmitter> moving_emitter;
    std::unique_ptr<RoomGeometry> room;
    std::unique_ptr<AudioSampleBuffer> test_sound_buffer;
    std::unique_ptr<Butterworth1Pole> atmospheric_filter;
    
    int32 test_sound_buffer_index;
    uint32 start_time;

    float sample_rate;

    uint32 initialized;

    // Gui

    Slider slider_gain;
    Label label_gain;
    Slider slider_freq;
    Label label_freq;
    Slider slider_radius;
    Label label_radius;

    GroupComponent group_atmosphere;    
    Slider slider_temperature;
    Slider slider_humidity;
    //Label label_temperature;
    Label label_cutoff;

    void timerCallback() override;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (MainComponent)
};
