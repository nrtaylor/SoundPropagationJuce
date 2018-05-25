/*
  ==============================================================================
  SoundPropagation - main Juce class
  ==============================================================================
*/

#pragma once

#include "../JuceLibraryCode/JuceHeader.h"
#include <mutex>
#include <array>

class MovingEmitter;
class RoomGeometry;
class RayCastCollector;
class PlannerAStar;
class PlannerWave;
namespace nDSP
{
    struct Butterworth1Pole;
}

namespace SoundPropagation
{
    enum MethodType : int32;
}

struct SoundBuffer
{
    SoundBuffer() :
        index(0),
        buffer(nullptr)
    {
        name[0] = (char)0;
    }

    char name[256];
    std::unique_ptr<AudioSampleBuffer> buffer;
    int32 index;
};

//==============================================================================
/*
    This component lives inside our window, and this is where you should put all
    your controls and content.
*/
class MainComponent   : public OpenGLAppComponent,
                        public AudioSource,
                        public Slider::Listener,
                        public ComboBox::Listener,
                        public ToggleButton::Listener,
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
    void comboBoxChanged(ComboBox* comboBoxThatHasChanged) override;
    void buttonClicked(Button* buttonClicked) override;

private:
    //==============================================================================
    AudioSourcePlayer audioSourcePlayer;
    std::unique_ptr<MovingEmitter> moving_emitter;
    std::unique_ptr<std::mutex> mutex_emitter_update;
    std::array<std::unique_ptr<nDSP::Butterworth1Pole>, 2> atmospheric_filters;
    std::vector<std::shared_ptr<RoomGeometry>> rooms;
    std::shared_ptr<RoomGeometry> current_room;
    std::unique_ptr<RayCastCollector> ray_cast_collector;
    std::unique_ptr<std::mutex> mutex_ray_cast_collector;

    std::unique_ptr<PlannerAStar> planner_astar;
    std::atomic<bool> planners_refresh;

    std::unique_ptr<PlannerWave> planner_wave;
    
    std::array<SoundBuffer, 2> test_buffers;
    std::atomic_uint32_t selected_test_buffer;
    uint32 start_time;

    float sample_rate;

    uint32 initialized;

    // Gui
    ComboBox combo_room;
    Label label_selected_room;

    ComboBox combo_method;
    Label label_method;

    ComboBox combo_compare_to_method;
    Label label_compare_to_method;

    std::atomic<SoundPropagation::MethodType> current_method;
    std::atomic<SoundPropagation::MethodType> current_compare_to_method;

    Image image_spl;
    Image image_next;
    std::atomic_bool show_spl;
    std::atomic_bool show_ray_casts;
    std::atomic_bool show_grid;
    std::atomic_bool show_contours;
    std::mutex mutex_image;
    std::atomic_bool flag_refresh_image;
    std::atomic_bool flag_update_working;
    std::atomic_bool flag_gamma_correct;

    Slider slider_gain;
    Label label_gain;
    Slider slider_freq;
    Label label_freq;
    Slider slider_radius;
    Label label_radius;
    ToggleButton button_show_spl;
    ToggleButton button_show_ray_casts;
    ToggleButton button_show_grid;
    ToggleButton button_show_contours;
    ToggleButton button_gamma_correct;

    Slider slider_spl_freq;
    Label label_spl_freq;
    std::atomic<float> test_frequency;

    Slider slider_time_scale;
    Label label_time_scale;
    std::atomic<float> time_scale;

    GroupComponent group_atmosphere;    
    Slider slider_temperature;
    Slider slider_humidity;
    Slider slider_pressure;
    Label label_cutoff;

    ComboBox combo_selected_sound;
    Label label_selected_sound;

    void timerCallback() override;

    void PaintEmitter(Graphics& _g, const Rectangle<int> _bounds, const float _zoom_factor) const;
    void PaintRoom(Graphics& _g, const Rectangle<int> _bounds, const float _zoom_factor) const;
    void PaintRayCasts(Graphics& _g, const Rectangle<int> _bounds, const float _zoom_factor) const;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (MainComponent)
};
