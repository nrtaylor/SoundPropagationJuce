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
struct PropagationResult;
class PropagationPlanner;
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
        buffer(nullptr),
        name("")
    {}

    String name;
    std::shared_ptr<AudioSampleBuffer> buffer;
    int32 index;
};

struct PropagationSource
{
    enum SourceType : int32
    {
        SOURCE_OFF = 1,
        SOURCE_FILE,
        SOURCE_FREQUENCY
    };
    SoundBuffer test_buffer;
    std::shared_ptr<PropagationPlanner> planner;
    std::atomic<SourceType> source_type;
};

enum ReadWriteControl : uint32
{
    RW_NONE = 0x0,
    RW_WRITING = 0x01,
    RW_READING = 0x02,
};

template<class T>
struct ReadWriteObject
{
    std::atomic<ReadWriteControl> lock;
    T object;
};

struct PlannerToResult
{
    std::shared_ptr<const PropagationPlanner> planner;
    std::unique_ptr<PropagationResult> result;
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

    //==============================================================================
    void mouseDown(const MouseEvent& event) override;

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
    
    using ReadWriteResult = ReadWriteObject<PlannerToResult>;
    std::array<ReadWriteResult, 3> simulation_results;
    uint32 write_index;
    std::atomic_uint32_t read_index;
    std::atomic<bool> planners_refresh;
    
    uint32 start_time;

    std::array<PropagationSource, 3> sources;
    std::atomic_int32_t selected_source;

    float sample_rate;

    uint32 initialized;

    std::atomic_int32_t receiver_x;
    std::atomic_int32_t receiver_y;

    // Gui
    ComboBox combo_room;
    Label label_selected_room;

    ComboBox combo_method;
    Label label_method;

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

    TextButton button_source[3];
    TextButton button_loadfile;
    Label label_loadfile;

    TextButton button_save_image;

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

    void GenerateSPLImage(Image& _image,
        std::shared_ptr<PropagationPlanner> planner, 
        std::shared_ptr<RoomGeometry> room,
        const float _time, const float _zoom_factor, const bool _allow_timeout = false);
    void ExportAsImage(const File& file, const int width, const int height, const float _zoom_factor);

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (MainComponent)
};
