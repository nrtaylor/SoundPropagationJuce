/*
  ==============================================================================
  SoundPropagation - main Juce class
  ==============================================================================
*/

#include "MainComponent.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include "nSignalProcessing.h"
#include "RoomGeometry.h"
#include "PropagationPlanner.h"
#include "PropagationPlannerAStar.h"

//#define PROFILE_SIMULATION

namespace ImageHelper
{
    Image SquareImage(const Rectangle<int>& bounds)
    {
        const int extent = nMath::Min(bounds.getWidth(), bounds.getHeight());
        return Image(Image::ARGB, extent, extent, true);
    }

    nMath::Vector Center(const Rectangle<int>& bounds)
    {
        const float min_extent = (float)nMath::Min(bounds.getWidth(), bounds.getHeight());
        return nMath::Vector { min_extent / 2.f, min_extent / 2.f, 0.f };
    }
}

namespace SoundBufferHelper
{
    void LoadFromFile(SoundBuffer& buffer, AudioFormatManager& format_manager, const File& file)
    {        
        std::unique_ptr<AudioFormatReader> reader(format_manager.createReaderFor(file));

        buffer.index = -1;
        if (reader != nullptr)
        {
            std::shared_ptr<AudioSampleBuffer> samples = std::make_shared<AudioSampleBuffer>();
            samples->setSize(reader->numChannels, static_cast<int>(reader->lengthInSamples));
            reader->read(samples.get(),
                0,
                static_cast<int>(reader->lengthInSamples),
                0,
                true,
                true);
            buffer.buffer = samples;
            buffer.index = 0;
            buffer.name = file.getFileName();
        }
    }

    void LoadFromFile(SoundBuffer& buffer, AudioFormatManager& format_manager, const char* file_name)
    {
        const File file = File::getCurrentWorkingDirectory().getChildFile(file_name);
        LoadFromFile(buffer, format_manager, file);
    }
}

//==============================================================================
MainComponent::MainComponent() :
    initialized(false),
    sample_rate(0.f),
    image_spl()
{
    start_time = Time::getMillisecondCounter();

    show_pressure = false;
    show_ray_casts = false;
    show_grid = false;
    show_contours = false;
    show_crests_only = false;
    flag_refresh_image = false;
    flag_update_working = false;
    flag_gamma_correct = false;

    mutex_emitter_update = std::make_unique<std::mutex>();

    setAudioChannels(0, 2);
    setWantsKeyboardFocus(true);

    selected_source = 0;

    button_source[0].setButtonText("Source 1");
    button_source[0].setToggleState(true, dontSendNotification);
    button_source[0].onClick = [this]() 
    { 
        selected_source = 0;
        label_selected_sound.setText("Source 1", dontSendNotification);
        RefreshSourceParams();
    };
    button_source[1].setButtonText("Source 2");
    button_source[1].onClick = [this]() 
    { 
        selected_source = 1;
        label_selected_sound.setText("Source 2", dontSendNotification); 
        RefreshSourceParams();
    };
    button_source[2].setButtonText("Source 3");
    button_source[2].onClick = [this]() 
    { 
        selected_source = 2;
        label_selected_sound.setText("Source 3", dontSendNotification);
        RefreshSourceParams();
    };
    const double default_emitter_gain = 0.8;
    const double default_emitter_freq = 0.0;
    const double default_emitter_radius = 10.0;
    for (int i = 0; i < 3; ++i)
    {
        button_source[i].setRadioGroupId(1001);
        button_source[i].setClickingTogglesState(true);
        addAndMakeVisible(button_source[i]);

        sources[i].planner.reset(new PlannerDirectLOS());        
        sources[i].moving_emitter = std::make_shared<MovingEmitter>();
        sources[i].moving_emitter->SetGlobalGain((float)default_emitter_gain);
        sources[i].moving_emitter->SetFrequency((float)default_emitter_freq);
        sources[i].moving_emitter->SetRadius((float)default_emitter_radius);
        sources[i].source_type = SoundPropagationSource::SOURCE_OFF;
    }

    button_loadfile.setButtonText("...");
    button_loadfile.onClick = [this]() {
        FileChooser chooser("Select Sound File", File::getCurrentWorkingDirectory(), "*.wav;*.aiff");
        if (chooser.browseForFileToOpen())
        {
            AudioFormatManager format_manager; format_manager.registerBasicFormats();
            const File& file = chooser.getResult();
            SoundBuffer& buffer_state = sources[selected_source].test_buffer;
            SoundBufferHelper::LoadFromFile(buffer_state, format_manager, file);
            label_loadfile.setText(buffer_state.name, dontSendNotification);
        }
    };
    addChildComponent(button_loadfile);
    label_loadfile.setText("[None]", dontSendNotification);
    addChildComponent(label_loadfile);
    
    button_save_image.setButtonText("Save Image");
    button_save_image.onClick = [this]() {
        FileChooser chooser("Select Image File", File::getCurrentWorkingDirectory(), "*.png");
        if (chooser.browseForFileToOpen())
        {
            ExportAsImage(chooser.getResult(), 1024, 1024, 2.f);
        }
    };
    addAndMakeVisible(&button_save_image);

    addAndMakeVisible(&slider_gain);
    slider_gain.setRange(0.0, 2.0, 0.05);
    slider_gain.setTextValueSuffix(" %");
    slider_gain.setValue(default_emitter_gain);
    slider_gain.addListener(this);

    addAndMakeVisible(&label_gain);
    label_gain.setText("Gain", dontSendNotification);
    label_gain.attachToComponent(&slider_gain, true);

    addAndMakeVisible(&slider_freq);
    slider_freq.setRange(0, 8.0, 0.001);
    slider_freq.setTextValueSuffix(" Hz");
    slider_freq.setValue(default_emitter_freq);
    slider_freq.setSkewFactorFromMidPoint(1.0);
    slider_freq.addListener(this);

    addAndMakeVisible(&label_freq);
    label_freq.setText("Freq", dontSendNotification);
    label_freq.attachToComponent(&slider_freq, true);

    addAndMakeVisible(&slider_radius);
    slider_radius.setRange(0.1, 25.0, 0.1);
    slider_radius.setTextValueSuffix(" m");
    slider_radius.setValue(default_emitter_radius);
    slider_radius.addListener(this);

    addAndMakeVisible(&label_radius);
    label_radius.setText("Radius", dontSendNotification);
    label_radius.attachToComponent(&slider_radius, true);

    addAndMakeVisible(&button_show_pressure);
    button_show_pressure.setButtonText("Pressure");
    button_show_pressure.setTooltip("Plot sound pressure as percent gain applied to the signal.");
    button_show_pressure.onClick = [this]() 
    { 
        const bool next_show_pressure = button_show_pressure.getToggleState();
        button_show_contours.setEnabled(next_show_pressure);
        button_gamma_correct.setEnabled(next_show_pressure);
        button_show_crests_only.setEnabled(next_show_pressure && 
            (current_method == SoundPropagation::Method_Wave ||
             current_method == SoundPropagation::Method_PlaneWave));
        show_pressure = next_show_pressure;
    };

    addAndMakeVisible(&button_show_ray_casts);
    button_show_ray_casts.setButtonText("Ray Casts");
    button_show_ray_casts.setTooltip("Draw ray casts used to compute sound pressure.");
    button_show_ray_casts.onClick = [this]() { show_ray_casts = button_show_ray_casts.getToggleState(); };

    addAndMakeVisible(&button_show_grid);
    button_show_grid.setButtonText("Draw Grid");
    button_show_grid.onClick = [this]() { show_grid = button_show_grid.getToggleState(); };

    addAndMakeVisible(&button_show_contours);
    button_show_contours.setButtonText("Contours");    
    button_show_contours.setEnabled(false);
    button_show_contours.onClick = [this]() { show_contours = button_show_contours.getToggleState(); };

    addAndMakeVisible(&button_show_crests_only);
    button_show_crests_only.setButtonText("Wave Fronts");
    button_show_crests_only.setEnabled(false);
    button_show_crests_only.setTooltip("Plot only sound pressure gain corresponding to the wave(s) period,\n or phase % 2pi = 0.");
    button_show_crests_only.onClick = [this]() { show_crests_only = button_show_crests_only.getToggleState(); };

    addAndMakeVisible(&button_gamma_correct);
    button_gamma_correct.setButtonText("Gamma");
    button_gamma_correct.setEnabled(false);
    button_gamma_correct.setTooltip("Apply gamma correct to sound pressure plot.");
    button_gamma_correct.onClick = [this]() { flag_gamma_correct = button_gamma_correct.getToggleState(); };
    
    addAndMakeVisible(&slider_spl_freq);
    slider_spl_freq.setRange(20.0, 20000.0, 0.5);
    slider_spl_freq.setTextValueSuffix(" Hz");
    slider_spl_freq.setValue(440.0);
    slider_spl_freq.setSkewFactorFromMidPoint(440.0);
    slider_spl_freq.addListener(this);
    test_frequency = 440.f;

    addAndMakeVisible(&label_spl_freq);
    label_spl_freq.setText("Test Freq", dontSendNotification);
    label_spl_freq.attachToComponent(&slider_spl_freq, true);

    addAndMakeVisible(&slider_time_scale);
    slider_time_scale.setRange(1.0, 2000.0, 1.0);
    slider_time_scale.setTextValueSuffix(" x");
    slider_time_scale.setValue(340.0);    
    slider_time_scale.addListener(this);
    time_scale = 340.f;

    addAndMakeVisible(&label_time_scale);
    label_time_scale.setText("Time Stretch", dontSendNotification);
    label_time_scale.attachToComponent(&slider_time_scale, true);

    addAndMakeVisible(&group_atmosphere);
    group_atmosphere.setText("Atmosphere");
    
    atmospheric_component.setBounds(4, 14, atmospheric_component.getWidth(), atmospheric_component.getHeight());
    group_atmosphere.addAndMakeVisible(&atmospheric_component);
    atmospheric_component.on_coefficient_changed = [this](const float cuttoff_frequency) 
    {
        SetAtmosphericFilterCuttoff(cuttoff_frequency);
    };

    addAndMakeVisible(&combo_selected_sound);

    // Make sure you set the size of the component after
    // you add any child components.
    setSize (1020, 600);

    receiver_x = 300;
    receiver_y = 300;

    atmospheric_filters[0] = std::make_unique<nDSP::Butterworth1Pole>();
    atmospheric_filters[0]->bypass = true;
    atmospheric_filters[1] = std::make_unique<nDSP::Butterworth1Pole>();
    atmospheric_filters[1]->bypass = true;
    
    for (int i = 0; i < (int)simulation_results.size(); ++i)
    {
        simulation_results[i].lock = RW_NONE;
        simulation_results[i].object = {
            nullptr,
            std::make_unique<PropagationResult>(PropagationResult{ SoundPropagation::PRD_FULL })
        };
    }
    write_index = 0;
    read_index = 1;

    current_room = nullptr;
    addAndMakeVisible(&combo_room);    
    combo_room.addListener(this);

    addAndMakeVisible(&label_selected_room);
    label_selected_room.setText("Room", dontSendNotification);
    label_selected_room.attachToComponent(&combo_room, true);
    // Empty
    {
        std::shared_ptr<RoomGeometry> room = std::make_shared<RoomGeometry>(RoomGeometry());        
        rooms.emplace_back(room);
        current_room = rooms.back();
        combo_room.addItem("Empty", static_cast<int>(rooms.size()));
        combo_room.setSelectedId(1);
    }
    // Rect
    {
        std::shared_ptr<RoomGeometry> room = std::make_shared<RoomGeometry>(RoomGeometry());
        room->AddWall({ -12.f,  8.f, 0.f }, { 12.f,  8.f, 0.f });
        room->AddWall({ 12.f,  8.f, 0.f }, { 12.f, -8.f, 0.f });
        room->AddWall({ 12.f, -8.f, 0.f }, { -12.f, -8.f, 0.f });
        room->AddWall({ -12.f, -8.f, 0.f }, { -12.f,  8.f, 0.f });
        rooms.emplace_back(room);
        combo_room.addItem("Rectangular", static_cast<int>(rooms.size()));
    }
    // Rect Bigger
    {
        std::shared_ptr<RoomGeometry> room = std::make_shared<RoomGeometry>(RoomGeometry());
        room->AddWall({ -20.f,  16.f, 0.f }, { 20.f,  16.f, 0.f });
        room->AddWall({ 20.f,  16.f, 0.f }, { 20.f, -16.f, 0.f });
        room->AddWall({ 20.f, -16.f, 0.f }, { -20.f, -16.f, 0.f });
        room->AddWall({ -20.f, -16.f, 0.f }, { -20.f,  16.f, 0.f });
        rooms.emplace_back(room);
        combo_room.addItem("Rect Bigger", static_cast<int>(rooms.size()));
    }
    // Rect Long
    {
        std::shared_ptr<RoomGeometry> room = std::make_shared<RoomGeometry>(RoomGeometry());
        room->AddWall({ -24.f,  11.25f, 0.f }, { 24.f,  11.25f, 0.f });
        room->AddWall({ 24.f,  11.25f, 0.f }, { 24.f, -11.25f, 0.f });
        room->AddWall({ 24.f, -11.25f, 0.f }, { -24.f, -11.25f, 0.f });
        room->AddWall({ -24.f, -11.25f, 0.f }, { -24.f,  11.25f, 0.f });
        rooms.emplace_back(room);
        combo_room.addItem("Rect Long", static_cast<int>(rooms.size()));
    }
    // Rect and Rect
    {
        std::shared_ptr<RoomGeometry> room = std::make_shared<RoomGeometry>(RoomGeometry());
        room->AddWall({ -20.f,  16.f, 0.f }, { 20.f,  16.f, 0.f });
        room->AddWall({ 20.f,  16.f, 0.f }, { 20.f, -16.f, 0.f });
        room->AddWall({ 20.f, -16.f, 0.f }, { -20.f, -16.f, 0.f });
        room->AddWall({ -20.f, -16.f, 0.f }, { -20.f,  16.f, 0.f });

        room->AddWall({ -5.f,  5.f, 0.f }, { 5.f,  5.f, 0.f });
        room->AddWall({ 5.f,  5.f, 0.f }, { 5.f, -5.f, 0.f });
        room->AddWall({ 5.f, -5.f, 0.f }, { -5.f, -5.f, 0.f });
        room->AddWall({ -5.f, -5.f, 0.f }, { -5.f,  5.f, 0.f });

        rooms.emplace_back(room);
        combo_room.addItem("Rect and column", static_cast<int>(rooms.size()));
    }
    // Wall
    {
        std::shared_ptr<RoomGeometry> room = std::make_shared<RoomGeometry>(RoomGeometry());
        room->AddWall({ -1000.f, 0.f, 0.f }, { 1000, 0.f, 0.f });
        rooms.emplace_back(room);
        combo_room.addItem("Wall", static_cast<int>(rooms.size()));
    }
    // Slit
    {
        std::shared_ptr<RoomGeometry> room = std::make_shared<RoomGeometry>(RoomGeometry());
        room->AddWall({ -16.f,  0.f, 0.f }, { -0.5f,  0.f, 0.f });
        room->AddWall({ 0.5f,  0.f, 0.f }, { 16.f, 0.f, 0.f });
        rooms.emplace_back(room);
        combo_room.addItem("Slit", static_cast<int>(rooms.size()));
    }
    // L Shaped
    {
        std::shared_ptr<RoomGeometry> room = std::make_shared<RoomGeometry>(RoomGeometry());
        room->AddWall({ -24.f,  24.f, 0.f }, { -8.f,  24.f, 0.f });
        room->AddWall({ -8.f,  24.f, 0.f }, { -8.f,  -8.f, 0.f });
        room->AddWall({ -8.f,  -8.f, 0.f }, {  24.f,  -8, 0.f });
        room->AddWall({ 24.f,  -8, 0.f }, { 24.f,  -20, 0.f });
        room->AddWall({ 24.f,  -20, 0.f }, { -24.f,  -20, 0.f });
        room->AddWall({ -24.f,  -20, 0.f }, { -24.f,  24, 0.f });
        rooms.emplace_back(room);
        combo_room.addItem("L Shaped", static_cast<int>(rooms.size()));
    }
    // Two Slits
    {
        std::shared_ptr<RoomGeometry> room = std::make_shared<RoomGeometry>(RoomGeometry());
        room->AddWall({ -18.f,  0.f, 0.f }, { -6.5f,  0.f, 0.f });
        room->AddWall({ -5.5f,  0.f, 0.f }, { 5.5f, 0.f, 0.f });
        room->AddWall({ 6.5f,  0.f, 0.f }, { 18.f, 0.f, 0.f });
        rooms.emplace_back(room);
        combo_room.addItem("Two Slits", static_cast<int>(rooms.size()));
    }
    // Room with opening
    {
        std::shared_ptr<RoomGeometry> room = std::make_shared<RoomGeometry>(RoomGeometry());
        room->AddWall({ -12.f,  6.5f, 0.f }, { 12.f,  8.f, 0.f });        
        room->AddWall({ 12.f,  8.f, 0.f }, { 12.f, 1.f, 0.f });
        room->AddWall({ 12.f,  -1.f, 0.f }, { 12.f, -8.f, 0.f });
        room->AddWall({ 12.f, -8.f, 0.f }, { -12.f, -6.5f, 0.f });
        room->AddWall({ -12.f, -6.5f, 0.f }, { -12.f,  6.5f, 0.f });
        rooms.emplace_back(room);
        combo_room.addItem("Trap Room with Opening", static_cast<int>(rooms.size()));
    }
    // Small Obstructions
    {
        std::shared_ptr<RoomGeometry> room = std::make_shared<RoomGeometry>(RoomGeometry());
        room->AddWall({ -3.f,  6.f, 0.f }, { 3.f,  6.f, 0.f });
        room->AddWall({ 3.f, -6.f, 0.f }, { -3.f, -8.f, 0.f });
        rooms.emplace_back(room);
        combo_room.addItem("Two Small Obstructors", static_cast<int>(rooms.size()));
    }
    // Trees
    {
        std::shared_ptr<RoomGeometry> room = std::make_shared<RoomGeometry>(RoomGeometry());
        const nMath::Vector offsets[] = {
            { -12.f, -12.f, 0.f },
            { 2.f, -3.8f, 0.f },
            { 18.5f, -16.f, 0.f },
            { 12.f, -12.f, 0.f },
            { 9.f, -3.8f, 0.f },
            { 7.5f, -9.f, 0.f },
            { -11.f, 23.f, 0.f },
            { 7.f, -8.f, 0.f },
            { 8.5f, -16.f, 0.f },
            { 14.f, 14.f, 0.f },
            { 19.f, -19.18f, 0.f },
            { -16.f, -8.5f, 0.f },
            { -12.f, 8.5f, 0.f },
            { -11.f, -23.f, 0.f },
            { 17.f, -4.f, 0.f },
            { 6.5f, 4.f, 0.f },
            { 7.5f, 9.f, 0.f }
        };
        for (int i = 0; i < sizeof(offsets) / sizeof(offsets[0]); ++i)
        {
            const nMath::Vector& offset = offsets[i];
            room->AddWall(offset + nMath::Vector{ 1.f,  -1.f, 0.f }, offset + nMath::Vector{ 2.f,  0.f, 0.f });
            room->AddWall(offset + nMath::Vector{ 2.f,  0.f, 0.f }, offset + nMath::Vector{ 1.f, 1.f, 0.f });
            room->AddWall(offset + nMath::Vector{ 1.f, 1.f, 0.f }, offset + nMath::Vector{ 0.f, 0.f, 0.f });
            room->AddWall(offset + nMath::Vector{ 0.f, 0.f, 0.f }, offset + nMath::Vector{ 1.f,  -1.f, 0.f });
        }
        rooms.emplace_back(room);
        combo_room.addItem("Trees", static_cast<int>(rooms.size()));
    }
    // Misc
    {
        std::shared_ptr<RoomGeometry> room = std::make_shared<RoomGeometry>(RoomGeometry());
        room->AddWall({ -14.75f,  -14.f, 0.f }, { -13.f,  0.f, 0.f });
        room->AddWall({ 3.75f, -6.5f, 0.f }, { -3.f, -8.f, 0.f });
        room->AddWall({ -0.8f, 10.f, 0.f }, { 1.5f, 7.3f, 0.f });
        room->AddWall({ 9.f, -1.f, 0.f }, { 7.3f, 2.5f, 0.f });
        room->AddWall({ -1, -10.f, 0.f }, { 5.5f, 7.3f, 0.f });
        room->AddWall({ -10, -12.5f, 0.f }, { 10.f, -12.5f, 0.f });
        rooms.emplace_back(room);
        combo_room.addItem("Misc.", static_cast<int>(rooms.size()));
    }

    addAndMakeVisible(&combo_method);
    combo_method.addListener(this);
    combo_method.addItem("Direct (Line of Sight)", SoundPropagation::Method_DirectLOS);
    combo_method.addItem("Ray Casts", SoundPropagation::Method_RayCasts);
    combo_method.addItem("A* (Pathfinding)", SoundPropagation::Method_Pathfinding);
    combo_method.addItem("LOS then A*", SoundPropagation::Method_LOSAStarFallback);
    combo_method.addItem("Waves", SoundPropagation::Method_Wave);
    combo_method.addItem("Plane Waves", SoundPropagation::Method_PlaneWave);
    current_method = SoundPropagation::Method_DirectLOS;
    combo_method.setSelectedId(SoundPropagation::Method_DirectLOS);

    addAndMakeVisible(&label_method);
    label_method.setText("Method", dontSendNotification);
    label_method.attachToComponent(&combo_method, true);
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
    combo_selected_sound.addItem("File", SoundPropagationSource::SOURCE_FILE);
    combo_selected_sound.addItem("Frequency", SoundPropagationSource::SOURCE_FREQUENCY);
    combo_selected_sound.addItem("Off", SoundPropagationSource::SOURCE_OFF);

    {
        MessageManagerLock lock;
        combo_selected_sound.setSelectedId(SoundPropagationSource::SOURCE_OFF);
        combo_selected_sound.addListener(this);

        addAndMakeVisible(&label_selected_sound);
        label_selected_sound.setText("Source 1", dontSendNotification);
        label_selected_sound.attachToComponent(&combo_selected_sound, true);
    }    

    startTimerHz(60);
    initialized = true;
}

void MainComponent::shutdown()
{
    bool is_working = true;
    if (flag_update_working.compare_exchange_strong(is_working, false))
    {
        int32 counter = 1000;
        while (!flag_refresh_image.load() &&
                counter-- > 0)
        {
            Thread::sleep(1);
        }
    }
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
    if (show_pressure.load())
    {
        if (flag_refresh_image.load())
        {
            std::lock_guard<std::mutex> guard(mutex_image);
            image_spl = image_next;
            flag_refresh_image.store(false);
        }
        _g.drawImageAt(image_spl, 0, 0);
    }

    const bool draw_waves = false;

    const Rectangle<int> bounds = _g.getClipBounds();
    const float zoom_factor = 10.f;
    if (!draw_waves)
    {
        PaintRoom(_g, bounds, zoom_factor);    
        PaintSimulation(_g, bounds, zoom_factor);
        PaintEmitter(_g, bounds, zoom_factor);
    }
    else
    {
        const nMath::Vector center = ImageHelper::Center(bounds);

        _g.setColour(Colour::fromRGB(0xCF, 0xCF, 0xCF));
        _g.drawLine(Line<float>(0.f, center.y, bounds.getWidth(), center.y), 0.75f);
        _g.drawLine(Line<float>(center.x, center.y - 1, center.x, center.y - bounds.getHeight()/16.f), 0.75f);

        _g.drawText(juce::String("x"), center.x - 11, center.y, 22, 22, juce::Justification::centred);
    }
}

void MainComponent::PaintEmitter(Graphics& _g, const Rectangle<int> _bounds, const float _zoom_factor) const
{
    const nMath::Vector center = ImageHelper::Center(_bounds);

    _g.setColour(Colour::fromRGB(0xFF, 0xFF, 0xFF));
    _g.fillEllipse(receiver_x - 1.f, receiver_y - 1.f, 2, 2);

    float gain_left = 0.f;
    float gain_right = 0.f;
    nMath::Vector emitter_pos;
    {
        std::lock_guard<std::mutex> guard(*mutex_emitter_update);
        const MovingEmitter& moving_emitter = *sources[selected_source].moving_emitter;
        emitter_pos = moving_emitter.GetPosition();
        gain_left = moving_emitter.Gain(0);
        gain_right = moving_emitter.Gain(1);
    }
    nMath::Vector emitter_draw_pos{ emitter_pos.x * _zoom_factor + center.x, -emitter_pos.y * _zoom_factor + center.y, 0.f };
    _g.fillEllipse(emitter_draw_pos.x - 1.f, emitter_draw_pos.y - 1.f, 2.5, 2.5);

    Rectangle<int> meter_bounds = _bounds;
    Rectangle<int> meter_labels = meter_bounds.removeFromBottom(32);
    meter_labels.removeFromBottom(8);
    Rectangle<int> meter_right = meter_bounds.removeFromRight(20);
    meter_right.reduce(4, 8);
    Rectangle<int> meter_left = meter_bounds.removeFromRight(20);
    meter_left.reduce(4, 8);

    _g.drawRect(meter_right);
    _g.drawRect(meter_left);

    const float gain_left_db = 20.f * log10f(gain_left + FLT_EPSILON);
    const float gain_right_db = 20.f * log10f(gain_right + FLT_EPSILON);
    
    _g.setFont(12);
    _g.drawText(juce::String::formatted("%.1f", gain_right_db), meter_labels.removeFromRight(20),
        juce::Justification::centred);
    _g.drawText(juce::String::formatted("%.1f", gain_left_db), meter_labels.removeFromRight(40),
        juce::Justification::centred);

    const float gain_left_db_ratio = 1.f - gain_left_db / -96.f;
    const float gain_right_db_ratio = 1.f - gain_right_db / -96.f;

    _g.setColour(Colour::fromRGBA(0xFF, 0xFF, 0xFF, 0x80));
    meter_right.reduce(2, 2);
    meter_left.reduce(2, 2);


    _g.fillRect(meter_left.removeFromBottom(meter_left.getHeight() * gain_left_db_ratio));
    _g.fillRect(meter_right.removeFromBottom(meter_right.getHeight() * gain_right_db_ratio));    
}

void MainComponent::PaintRoom(Graphics& _g, const Rectangle<int> _bounds, const float _zoom_factor) const
{
    std::shared_ptr<RoomGeometry> room = current_room;
    if (room != nullptr)
    {
        const nMath::Vector center = ImageHelper::Center(_bounds);

        _g.setColour(Colour::fromRGB(0x77, 0x1c, 0x47));
        Path room_lines;
        auto& walls = room->Walls();
        for (const nMath::LineSegment& wall : walls)
        {
            const Line<float> drawLine(
                wall.start.x * _zoom_factor + center.x,
                -wall.start.y * _zoom_factor + center.y,
                wall.end.x * _zoom_factor + center.x,
                -wall.end.y * _zoom_factor + center.y);
            room_lines.addLineSegment(drawLine, 2.f);
        }
        _g.fillPath(room_lines);
    }
}

void MainComponent::PaintSimulation(Graphics& _g, const Rectangle<int> _bounds, const float _zoom_factor)
{
    ReadWriteControl rw = RW_NONE;
    const uint32 current_read_index = read_index;
    if (simulation_results[current_read_index].lock.compare_exchange_strong(rw, RW_READING))
    {
        if (show_ray_casts)
        {
            const PropagationResult& result = *simulation_results[current_read_index].object.result;
            if (result.intersections.size() > 0)
            {
                const nMath::Vector center = ImageHelper::Center(_bounds);
                _g.setColour(Colour::fromRGB(0x0, 0xAA, 0xAA));
                for (const nMath::LineSegment line : result.intersections)
                {
                    _g.drawLine((line.start.x * _zoom_factor) + center.x,
                        -(line.start.y * _zoom_factor) + center.y,
                        (line.end.x * _zoom_factor) + center.x,
                        -(line.end.y * _zoom_factor) + center.y);
                }
            }
        }
        if (show_grid)
        {
            const PropagationResult& result = *simulation_results[current_read_index].object.result;
            std::shared_ptr<const PropagationPlanner> planner = simulation_results[current_read_index].object.planner;
            std::shared_ptr<const PlannerAStar> astar_planner = std::dynamic_pointer_cast<const PlannerAStar>(planner);
            if (astar_planner == nullptr)
            {
                if (std::shared_ptr<const PlannerTwoStages<PlannerDirectLOS, PlannerAStar> > planner_two =
                    std::dynamic_pointer_cast<const PlannerTwoStages<PlannerDirectLOS, PlannerAStar>>(planner))
                {
                    astar_planner = planner_two->Secondary();
                }
            }
            if (astar_planner != nullptr) // TODO: planner should know its type.
            {
                const PlannerAStar::GeometryGrid& grid = astar_planner->Grid();
                const float min_extent = (float)nMath::Min(_bounds.getWidth(), _bounds.getHeight());
                _g.setColour(Colour::fromRGBA(0x77, 0x77, 0x77, 0x99));
                const int offset = (int)(min_extent / 2.f - _zoom_factor * PlannerAStar::GridDistance / 2);
                const int cellSize = (int)_zoom_factor / PlannerAStar::GridCellsPerMeter;
                for (int i = 0; i < PlannerAStar::GridResolution; ++i)
                {
                    for (int j = 0; j < PlannerAStar::GridResolution; ++j)
                    {
                        if (grid[i][j])
                        {
                            _g.fillRect(
                                cellSize * j + offset,
                                cellSize * (PlannerAStar::GridResolution - i - 1) + offset,
                                cellSize - 1,
                                cellSize - 1);
                        }
                        if (PlannerAStar::GridNodeSearched(result.cache, i, j))
                        {
                            _g.drawLine(
                                (float)(cellSize * j + offset),
                                (float)(cellSize * (PlannerAStar::GridResolution - i - 1) + offset),
                                (float)(cellSize * (j + 1) + offset),
                                (float)(cellSize * (PlannerAStar::GridResolution - (i + 1) - 1) + offset));
                        }
                    }
                }
            }
        }
        simulation_results[current_read_index].lock = RW_NONE;
    }
}

void MainComponent::ExportAsImage(const File& file, const int width, const int height, const float _zoom_factor)
{
    jassert(width == height); // non-square images not supported yet
    const File image_file = file.withFileExtension(".png");
    PNGImageFormat png_format;
    FileOutputStream file_output(image_file);
    
    Image export_image = ImageHelper::SquareImage(juce::Rectangle<int>(width, height));
    const float time_now = (float)(Time::currentTimeMillis() % ((1 + (int)test_frequency) * 1000));

    nMath::Vector emitter_pos;
    {
        std::lock_guard<std::mutex> guard(*mutex_emitter_update);
        emitter_pos = sources[selected_source].moving_emitter->Update(0);
    }
    std::shared_ptr<RoomGeometry> room = current_room;
    const PropagationPlanner::SourceConfig planner_config = {
        emitter_pos,
        test_frequency.load(),
        time_scale.load()
    };
    std::shared_ptr<PropagationPlanner> planner = PropagationPlanner::MakePlanner(current_method);
    planner->Preprocess(room);
    planner->Plan(planner_config);

    GenerateSPLImage(export_image, planner, room, time_now, _zoom_factor);
    Graphics g(export_image);
    PaintRoom(g, export_image.getBounds(), _zoom_factor * 10.f);
    png_format.writeImageToStream(export_image, file_output);
    file_output.flush();
}

void MainComponent::resized()
{
    // This is called when the MainContentComponent is resized.
    // If you add any child components, this is where you should
    // update their positions.

    const Rectangle<int> bounds = getBounds();    
    {
        std::lock_guard<std::mutex> guard(mutex_image);
        image_spl = ImageHelper::SquareImage(bounds);
        image_next = ImageHelper::SquareImage(bounds);
    }

    const int32 margin = 4;

    juce::Rectangle<int> frame = getLocalBounds();
    const int meter_size = (20 + margin) * 2;
    frame.removeFromRight(meter_size);
    frame.removeFromRight(margin);
    frame.removeFromTop(90);
    frame = frame.removeFromRight(204);

    auto frame_next = [&frame]() -> decltype(frame) 
    { 
        const int32 height = 24;
        const int32 padding = 2;
        return frame.removeFromTop(height).reduced(padding); 
    };
    
    juce::Rectangle<int>  frame_sources = frame_next();
    int sources_width = frame_sources.getWidth() / 3;
    for (int i = 0; i < 3; ++i)
    {
        button_source[i].setBounds(frame_sources.removeFromLeft(sources_width).reduced(2, 0));
    }

    // Source
    combo_selected_sound.setBounds(frame_next());
    const juce::Rectangle<int> frame_source_settings = frame_next();
    {
        juce::Rectangle<int> frame_source_file_settings = frame_source_settings;
        button_loadfile.setBounds(frame_source_file_settings.removeFromRight(32));
        label_loadfile.setBounds(frame_source_file_settings);
    }
    slider_spl_freq.setBounds(frame_source_settings);
    slider_gain.setBounds(frame_next());
    slider_freq.setBounds(frame_next());
    slider_radius.setBounds(frame_next());

    // Global
    combo_method.setBounds(frame_next());
    combo_room.setBounds(frame_next());

    juce::Rectangle<int> frame_button_l = frame_next();
    juce::Rectangle<int> frame_button_r = frame_button_l.removeFromRight(frame_button_l.getWidth() / 2);    
    button_show_ray_casts.setBounds(frame_button_l);
    button_show_pressure.setBounds(frame_button_r);
    
    frame_button_l = frame_next();
    frame_button_r = frame_button_l.removeFromRight(frame_button_l.getWidth() / 2);
    button_show_grid.setBounds(frame_button_l);
    button_show_contours.setBounds(frame_button_r);

    frame_button_l = frame_next();
    frame_button_r = frame_button_l.removeFromRight(frame_button_l.getWidth() / 2);
    button_gamma_correct.setBounds(frame_button_l);
    button_show_crests_only.setBounds(frame_button_r);

    slider_time_scale.setBounds(frame_next());

    const int atmosphere_offset_x = atmospheric_component.getBounds().getPosition().x + atmospheric_component.getWidth();
    const int atmosphere_offset_y = atmospheric_component.getHeight() + atmospheric_component.getBounds().getPosition().y;
    juce::Rectangle<int> frame_atmosphere = frame.removeFromTop(atmosphere_offset_y + 4 + 2).reduced(2);
    
    frame_atmosphere.setLeft(frame_atmosphere.getX() + (frame_atmosphere.getWidth() - atmosphere_offset_x));
    group_atmosphere.setBounds(frame_atmosphere);

    button_save_image.setBounds(frame_next());
}

// Audio Component
void MainComponent::prepareToPlay(int samplesPerBlockExpected,
    double sampleRate)
{
    (void)samplesPerBlockExpected;
    sample_rate = (float)sampleRate;
}

void MainComponent::releaseResources()
{
}

void MainComponent::mouseDown(const MouseEvent& event)
{
    const float min_extent = (float)nMath::Min(getBounds().getWidth(), getBounds().getHeight());
    if (event.getMouseDownX() < min_extent &&
        event.getMouseDownY() < min_extent)
    {
        receiver_x = event.getMouseDownX();
        receiver_y = event.getMouseDownY();
    }
}

void MainComponent::getNextAudioBlock(const AudioSourceChannelInfo& bufferToFill)
{
    bufferToFill.clearActiveBufferRegion();
    if (!initialized)
    {
        return;
    }

    const int32 source_id = selected_source.load();
    SoundPropagationSource& source = sources[source_id];
    if (source.source_type != SoundPropagationSource::SOURCE_FILE)
    {
        return;
    }

    SoundBuffer& buffer_info = source.test_buffer;
    std::shared_ptr<AudioSampleBuffer> buffer = buffer_info.buffer;
    if (buffer == nullptr ||
        buffer->getNumChannels() <= 0)
    {
        return;
    }
    if (buffer_info.index < 0)
    {
        return;
    }
    int& buffer_index = buffer_info.index;

    const int channels = bufferToFill.buffer->getNumChannels();
    int samples_remaining = bufferToFill.numSamples;
    int sample_offset = bufferToFill.startSample;
    const float gain_left = source.moving_emitter->Gain(0);
    const float gain_right = source.moving_emitter->Gain(1);

    while (samples_remaining > 0)
    {
        int buffer_samples_remaining = buffer->getNumSamples() - buffer_index;
        int samples_to_write = jmin(samples_remaining, buffer_samples_remaining);

        for (int channel = 0; channel < channels; ++channel)
        {
            nDSP::Butterworth1Pole& lpf = *atmospheric_filters[channel].get();
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

_declspec(noinline) bool WasteTime(int seed)
{
    float j = 112113.5f * (float)(seed + 1);
    for (int i = 0; i < 1000; ++i)
    {
        j = sqrtf(j);
    }

    return j > 0.f;
}

void MainComponent::GenerateWaveImage(Image& _image,
    std::shared_ptr<PropagationPlanner> planner,
    std::shared_ptr<RoomGeometry> room,
    const float _time,
    const float _zoom_factor,
    const bool _allow_timeout)
{
    const nMath::Vector center{ _image.getWidth() / 2.f, _image.getWidth() / 2.f, 0.f };
    const float inv_zoom_factor = 1.f / (_zoom_factor * 10.f);

    const bool overlay_contours = show_contours.load();
    const bool filter_crests = show_crests_only.load();
    const bool gamma_correct = flag_gamma_correct.load();
    const int extent = _image.getWidth();
    const Image::BitmapData bitmap(_image, Image::BitmapData::writeOnly);

#ifdef PROFILE_SIMULATION
    Thread::sleep(5000);
#endif
    uint8* pixel = bitmap.getPixelPointer(0, 0);

    PropagationResult result{ SoundPropagation::PRD_GAIN };

    for (int i = 0; i < extent; ++i)
    {
        const float normalized = 2 * i / (float)extent;
        for (int j = 0; j < extent; ++j)
        {
            if (_allow_timeout &&
                !flag_update_working)
            {
                flag_refresh_image.store(true);
                return; // task cancelled
            }

            const nMath::Vector pixel_to_world = { (j - center.x) * inv_zoom_factor,
                (i - center.y) * -inv_zoom_factor,
                0.f };

            float energy_canidate[2];
            {
                planner->Simulate(result, pixel_to_world, _time);
                float compression_amount = 0.25f;
                float mag_epsilon = 0.0055f * (1.f - fabsf(result.magnitude));
                energy_canidate[0] = jmax<float>(0.f, mag_epsilon + 1.f - fabsf(normalized - (compression_amount * -result.magnitude + 1.f)) / 2.f);                
                energy_canidate[0] = powf(energy_canidate[0], 600);
            }
            {
                planner->Simulate(result, pixel_to_world, 0);
                float compression_amount = 0.25f;
                float mag_epsilon = 0.0045f * (1.f - fabsf(result.magnitude));
                energy_canidate[1] = jmax<float>(0.f, mag_epsilon + 1.f - fabsf(normalized - (compression_amount * -result.magnitude + 1.f)) / 2.f);
                energy_canidate[1] = jmin<float>(1.f, powf(energy_canidate[1], 700)) * 0.6f;
            }
            float energy = jmax<float>(energy_canidate[0], energy_canidate[1]);
            uint8 colour = (uint8)jmin<uint32>(255, (uint32)(255.f*energy));
            *pixel++ = colour;
            *pixel++ = colour;
            *pixel++ = colour;
            *pixel++ = 0xFF;
        }
    }
}

void MainComponent::GenerateSPLImage(Image& _image, 
    std::shared_ptr<PropagationPlanner> planner, 
    std::shared_ptr<RoomGeometry> room, 
    const float _time,
    const float _zoom_factor,
    const bool _allow_timeout)
{
    const nMath::Vector center{ _image.getWidth() / 2.f, _image.getWidth() / 2.f, 0.f };
    const float inv_zoom_factor = 1.f / (_zoom_factor * 10.f);

    const bool overlay_contours = show_contours.load();
    const bool filter_crests = show_crests_only.load();
    const bool gamma_correct = flag_gamma_correct.load();
    const int extent = _image.getWidth();
    const Image::BitmapData bitmap(_image, Image::BitmapData::writeOnly);

#ifdef PROFILE_SIMULATION
    Thread::sleep(5000);
#endif
    uint8* pixel = bitmap.getPixelPointer(0, 0);

    // for contours
    std::vector<float> previous_row; previous_row.resize(extent);
    std::fill(previous_row.begin(), previous_row.end(), FLT_MAX);

    std::vector<float> previous_row_abs;
    std::vector<int32> previous_row_id;
    if (filter_crests)
    {
        previous_row_abs.resize(extent); previous_row_abs[0] = 0.f;
        previous_row_id.resize(extent); previous_row_id[0] = 0;
    }

    PropagationResult result{ SoundPropagation::PRD_GAIN };

    for (int i = 0; i < extent; ++i)
    {
        for (int j = 0; j < extent; ++j)
        {
#ifdef PROFILE_SIMULATION
            if (!WasteTime(j))
            {
                break;
            }
#endif
            if (_allow_timeout &&
                !flag_update_working)
            {
                flag_refresh_image.store(true);
                return; // task cancelled
            }

            const nMath::Vector pixel_to_world = { (j - center.x) * inv_zoom_factor,
                (i - center.y) * -inv_zoom_factor,
                0.f };
            planner->Simulate(result, pixel_to_world, _time);
            float energy = result.gain;
            if (filter_crests)
            {
                if (!(j > 1 && i > 1 &&
                    previous_row_abs[j - 1] > result.absolute &&
                    previous_row_abs[j - 1] > previous_row_abs[j - 2]))
                {
                    energy = 0.f;
                }
                //if (j > 1 && i > 1)
                //{
                //    float partial_x = previous_row_abs[j - 1] - result.absolute;
                //    float partial_y = previous_row_abs[j] - result.absolute;
                //    energy = energy*partial_x + energy*partial_y;
                //}
                previous_row_abs[j] = result.absolute;
                previous_row_id[j] = result.wave_id;
            }
            int contour_color = -1;

            if (overlay_contours)
            {
                const int num_countours = 6;
                float threshold = 0.5f;
                for (int c = 0; c < num_countours; ++c)
                {
                    if (energy <= threshold &&
                        ((j && previous_row[j - 1] > threshold) ||
                        (i && previous_row[j] > threshold)))
                    {
                        contour_color = 192 - (c * 32);
                        break;
                    }
                    else if (energy > threshold &&
                        ((j && previous_row[j - 1] <= threshold) ||
                        (i && previous_row[j] <= threshold)))
                    {
                        contour_color = 192 - (c * 32);
                        break;
                    }
                    threshold /= 2.f;
                }

                previous_row[j] = energy;
            }

            energy = jmax<float>(0.f, energy);
            if (gamma_correct)
            {
                energy = sqrtf(energy);
            }

            const uint8 colour = contour_color > 0 ?
                (uint8)contour_color :
                (uint8)jmin<uint32>(255, (uint32)(255.f*energy));
            *pixel++ = colour;
            *pixel++ = colour;
            *pixel++ = colour;
            *pixel++ = 0xFF;
        }
    }
}

// Animated Component
void MainComponent::update()
{
    int32 frame_time = Time::getMillisecondCounter();
    SoundPropagationSource& source = sources[selected_source];
    std::shared_ptr<PropagationPlanner> planner = source.planner;
    std::shared_ptr<MovingEmitter> moving_emitter = source.moving_emitter;

    nMath::Vector emitter_pos;
    {
        std::lock_guard<std::mutex> guard(*mutex_emitter_update);
#ifndef PROFILE_SIMULATION        
        emitter_pos = moving_emitter->Update(frame_time - start_time);
#else
        emitter_pos = moving_emitter->Update(0);
#endif
    }        
    std::shared_ptr<RoomGeometry> room = current_room; // this isn't guarunteed atomic
    const float time_now = (float)(Time::currentTimeMillis() % ((1 + (int)test_frequency) * 1000));// TODO: start at t = 0 or store in planner.
    if (room != nullptr)
    {
        const nMath::Vector center = ImageHelper::Center(getBounds());
        const float inv_zoom_factor = 1.f/10.f;
        const nMath::Vector receiever_pos = { (receiver_x - center.x) * inv_zoom_factor , (receiver_y - center.y) * -inv_zoom_factor, 0.f};

        const PropagationPlanner::SourceConfig planner_config = {
            emitter_pos,
            test_frequency.load(),
            time_scale.load()
        };
        planner = PropagationPlanner::MakePlanner(current_method);
        planner->Preprocess(room);
        planner->Plan(planner_config);

        const uint32 result_buffer_size = (uint32)simulation_results.size();
        const uint32 local_read_index = read_index;
        for (uint32 i = 0; i < result_buffer_size; ++i)
        {
            const uint32 next_write_index = (write_index + i) % result_buffer_size;
            if (local_read_index == next_write_index)
            {
                continue;
            }
            ReadWriteControl rw = RW_NONE;
            if (simulation_results[next_write_index].lock.compare_exchange_strong(rw, RW_WRITING))
            {
                write_index = next_write_index;
                break;
            }
        }
        jassert(write_index < result_buffer_size);
        ReadWriteResult& simulation_result = simulation_results[write_index];
        simulation_result.object.planner = planner;
        planner->Simulate(*simulation_result.object.result, receiever_pos, 0.f);
        
        simulation_result.lock = RW_NONE;
        read_index = write_index;
        ++write_index;        

        const float simulated_gain = simulation_result.object.result->gain;
        moving_emitter->ComputeGain(simulated_gain);
    }
    
    if (show_pressure.load() &&        
        !flag_refresh_image.load())
    {        
        bool is_working = flag_update_working.load();
        if (!is_working &&
            flag_update_working.compare_exchange_strong(is_working, true))
        {
            {
                std::lock_guard<std::mutex> guard(mutex_image);
                image_next = ImageHelper::SquareImage(getBounds());
            }

            std::thread worker = std::thread([this, planner, room, time_now] {
                const bool show_wave = false;
                std::lock_guard<std::mutex> guard(mutex_image);
                if (!show_wave)
                {
                    GenerateSPLImage(image_next, planner, room, time_now, 1.f, true);
                }
                else
                {
                    GenerateWaveImage(image_next, planner, room, time_now, 1.f, true);
                }
                flag_update_working.store(false);
                flag_refresh_image.store(true);
            });
            worker.detach();            
        }
    }

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
        sources[selected_source].moving_emitter->SetGlobalGain((float)slider_gain.getValue());
    }
    else if (slider == &slider_freq)
    {
        sources[selected_source].moving_emitter->SetFrequency((float)slider_freq.getValue());
    }
    else if (slider == &slider_spl_freq)
    {
        test_frequency = (float)slider_spl_freq.getValue();
    }
    else if (slider == &slider_time_scale)
    {
        time_scale = (float)slider_time_scale.getValue();
    }
    else if (slider == &slider_radius)
    {
        const float next_radius = (float)slider_radius.getValue();
        sources[selected_source].moving_emitter->SetRadius(next_radius);
        atmospheric_component.SetDistance(next_radius, sendNotification);
    }
}

void MainComponent::SetAtmosphericFilterCuttoff(const float cuttoff_frequency)
{
    atmospheric_filters[0]->Initialize(cuttoff_frequency, sample_rate);
    atmospheric_filters[0]->bypass = cuttoff_frequency > sample_rate / 2.f;
    atmospheric_filters[1]->Initialize(cuttoff_frequency, sample_rate);
    atmospheric_filters[1]->bypass = cuttoff_frequency > sample_rate / 2.f;
}

void MainComponent::RefreshSourceParams()
{
    const int32 source_id = selected_source.load();
    const SoundPropagationSource& source = sources[source_id];
    if (source.source_type == SoundPropagationSource::SOURCE_FILE)
    {
        label_loadfile.setText(source.test_buffer.name, dontSendNotification);

        button_loadfile.setVisible(true);
        label_loadfile.setVisible(true);
        slider_spl_freq.setVisible(false);
    }
    else
    {
        button_loadfile.setVisible(false);
        label_loadfile.setVisible(false);
        slider_spl_freq.setVisible(true);
    }

    slider_freq.setValue(source.moving_emitter->GetFrequency());
    slider_radius.setValue(source.moving_emitter->SetRadius());
    slider_gain.setValue(source.moving_emitter->GetGlobalGain());

    if (combo_selected_sound.getSelectedId() != static_cast<int>(source.source_type))
    {
        combo_selected_sound.setSelectedId(static_cast<int>(source.source_type), dontSendNotification);
    }
}

void MainComponent::comboBoxChanged(ComboBox* comboBoxThatHasChanged)
{
    if (comboBoxThatHasChanged == &combo_selected_sound)
    {
        const int32 next_id = combo_selected_sound.getSelectedId();
        if (next_id > 0)
        {
            const int32 source_id = selected_source.load();
            sources[source_id].source_type = (SoundPropagationSource::SourceType)next_id;
            RefreshSourceParams();
        }
    }
    else if (comboBoxThatHasChanged == &combo_room ||
             comboBoxThatHasChanged == &combo_method)
    {
        const uint32 next_id = combo_room.getSelectedId();
        if (next_id > 0)
        {
            current_room = rooms[next_id - 1];    
        }
        current_method = static_cast<SoundPropagation::MethodType>(combo_method.getSelectedId());        
        sources[selected_source].planner = PropagationPlanner::MakePlanner(current_method);
 
        button_show_grid.setEnabled(false);
        button_show_crests_only.setEnabled(false);
        switch (current_method)
        {
        case SoundPropagation::Method_Wave:
        case SoundPropagation::Method_PlaneWave:
            button_show_crests_only.setEnabled(show_pressure.load());
            break;
        case SoundPropagation::Method_Pathfinding:
        case SoundPropagation::Method_LOSAStarFallback:
            button_show_grid.setEnabled(true);
            break;
        default:
            break;
        }
    }
}
