/*
  ==============================================================================
  SoundPropagation - main Juce class
  ==============================================================================
*/

#pragma once

#include "../JuceLibraryCode/JuceHeader.h"
#include "AtmosphericAbsorptionComponent.h"
#include "nReadOrWriteObject.h"
#include <mutex>
#include <array>

class MovingEmitter;
class RoomGeometry;
class PropagationPlanner;
struct PropagationResult;
namespace nDSP { struct Butterworth1Pole; }
namespace SoundPropagation { enum MethodType : int32; }

struct SoundBuffer
{
    SoundBuffer() :        
        name(""),
        buffer(nullptr),        
        index(0)
    {}

    String name;
    std::shared_ptr<AudioSampleBuffer> buffer;
    int32 index;
};

struct SoundPropagationSource
{
    enum SourceType : int32
    {
        SOURCE_OFF = 1,
        SOURCE_FILE,
        SOURCE_FREQUENCY
    };
    SoundBuffer test_buffer;
    std::shared_ptr<PropagationPlanner> planner;
    std::shared_ptr<MovingEmitter> moving_emitter;
    std::atomic<SourceType> source_type;
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

private:
    //==============================================================================
    AudioSourcePlayer audioSourcePlayer;
    
    std::unique_ptr<std::mutex> mutex_emitter_update;
    std::array<std::unique_ptr<nDSP::Butterworth1Pole>, 2> atmospheric_filters;
    std::vector<std::shared_ptr<RoomGeometry>> rooms;
    std::shared_ptr<RoomGeometry> current_room;    

    struct PlannerToResult
    {
        std::shared_ptr<const PropagationPlanner> planner;
        std::unique_ptr<PropagationResult> result;
    };
    using ReadWriteResult = ReadOrWriteObject<PlannerToResult>;
    std::array<ReadWriteResult, 3> simulation_results;
    uint32 write_index;
    std::atomic_uint32_t read_index;
    
    uint32 start_time;

    std::array<SoundPropagationSource, 3> sources;
    std::atomic_int32_t selected_source;

    float sample_rate;

    uint32 initialized;

    std::atomic_int32_t receiver_x;
    std::atomic_int32_t receiver_y;

    std::atomic<SoundPropagation::MethodType> current_method;

    Image image_spl;
    Image image_next;
    std::mutex mutex_image;
    std::atomic_bool flag_refresh_image;
    std::atomic_bool flag_update_working;

    // Image options
    std::atomic_bool show_pressure;
    std::atomic_bool show_ray_casts;
    std::atomic_bool show_grid;
    std::atomic_bool show_contours;
    std::atomic_bool show_crests_only;
    std::atomic_bool flag_gamma_correct;

    // Gui
    ComboBox combo_room;
    Label label_selected_room;
    ComboBox combo_method;
    Label label_method;

    Slider slider_gain;
    Label label_gain;
    Slider slider_freq;
    Label label_freq;
    Slider slider_radius;
    Label label_radius;
    ToggleButton button_show_pressure;
    ToggleButton button_show_ray_casts;
    ToggleButton button_show_grid;
    ToggleButton button_show_contours;
    ToggleButton button_show_crests_only;
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
    AtmosphericAbsorptionComponent atmospheric_component;

    ComboBox combo_selected_sound;
    Label label_selected_sound;

    void timerCallback() override;

    void PaintEmitter(Graphics& _g, const Rectangle<int> _bounds, const float _zoom_factor) const;
    void PaintRoom(Graphics& _g, const Rectangle<int> _bounds, const float _zoom_factor) const;
    void PaintSimulation(Graphics& _g, const Rectangle<int> _bounds, const float _zoom_factor);

    void GenerateSPLImage(Image& _image,
        std::shared_ptr<PropagationPlanner> planner, 
        std::shared_ptr<RoomGeometry> room,
        const float _time, const float _zoom_factor, const bool _allow_timeout = false);
    void ExportAsImage(const File& file, const int width, const int height, const float _zoom_factor);

    void SetAtmosphericFilterCuttoff(const float cuttoff_frequency);

    // UI Helpers
    void RefreshSourceParams();

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (MainComponent)
};
