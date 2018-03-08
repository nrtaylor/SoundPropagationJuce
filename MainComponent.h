/*
  ==============================================================================
  SoundPropagation - main Juce class
  ==============================================================================
*/

#pragma once

#include "../JuceLibraryCode/JuceHeader.h"

class MovingEmitter;
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
    std::unique_ptr<AudioSampleBuffer> test_sound_buffer;
    
    int32 test_sound_buffer_index;
    uint32 start_time;

    // Gui

    Slider slider_gain;
    Label label_gain;
    Slider slider_freq;
    Label label_freq;
    Slider slider_radius;
    Label label_radius;

    void timerCallback() override;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (MainComponent)
};
