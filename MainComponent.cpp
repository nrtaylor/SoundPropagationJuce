/*
  ==============================================================================
  SoundPropagation - main Juce class
  ==============================================================================
*/

#include "MainComponent.h"
#include "AtmosphericAbsorption.h"
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

    show_spl = false;
    show_ray_casts = false;
    show_grid = false;
    show_contours = false;
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
    const double default_emitter_freq = 0.08;
    const double default_emitter_radius = 10.0;
    for (int i = 0; i < 3; ++i)
    {
        button_source[i].setRadioGroupId(1001);
        button_source[i].setClickingTogglesState(true);
        addAndMakeVisible(button_source[i]);

        sources[i].planner.reset(new PlannerSpecularLOS());        
        sources[i].moving_emitter = std::make_shared<MovingEmitter>();
        sources[i].moving_emitter->SetGlobalGain((float)default_emitter_gain);
        sources[i].moving_emitter->SetFrequency((float)default_emitter_freq);
        sources[i].moving_emitter->SetRadius((float)default_emitter_radius);
        sources[i].source_type = PropagationSource::SOURCE_OFF;
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

    addAndMakeVisible(&button_show_spl);
    button_show_spl.setButtonText("Draw SPL");
    button_show_spl.onClick = [this]() 
    { 
        const bool next_show_spl = button_show_spl.getToggleState();
        button_show_contours.setEnabled(next_show_spl);
        button_gamma_correct.setEnabled(next_show_spl);
        show_spl = next_show_spl;
    };

    addAndMakeVisible(&button_show_ray_casts);
    button_show_ray_casts.setButtonText("Ray Casts");
    button_show_ray_casts.onClick = [this]() { show_ray_casts = button_show_ray_casts.getToggleState(); };

    addAndMakeVisible(&button_show_grid);
    button_show_grid.setButtonText("Draw Grid");
    button_show_grid.onClick = [this]() { show_grid = button_show_grid.getToggleState(); };

    addAndMakeVisible(&button_show_contours);
    button_show_contours.setButtonText("Contours");    
    button_show_contours.setEnabled(false);
    button_show_contours.onClick = [this]() { show_contours = button_show_contours.getToggleState(); };

    addAndMakeVisible(&button_gamma_correct);
    button_gamma_correct.setButtonText("Gamma");
    button_gamma_correct.setEnabled(false);
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
    slider_time_scale.setRange(1.0, 1000.0, 1.0);
    slider_time_scale.setTextValueSuffix(" x");
    slider_time_scale.setValue(340.0);
    slider_time_scale.setSkewFactorFromMidPoint(100.0);
    slider_time_scale.addListener(this);
    time_scale = 340.f;

    addAndMakeVisible(&label_time_scale);
    label_time_scale.setText("Time Stretch", dontSendNotification);
    label_time_scale.attachToComponent(&slider_time_scale, true);

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

    planners_refresh = true;

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
    combo_method.addItem("Specular (LOS)", SoundPropagation::Method_SpecularLOS);
    combo_method.addItem("Ray Casts", SoundPropagation::Method_RayCasts);
    combo_method.addItem("A*", SoundPropagation::Method_Pathfinding);
    combo_method.addItem("LOS then A*", SoundPropagation::Method_LOSAStarFallback);
    combo_method.addItem("Wave Equation", SoundPropagation::Method_Wave);
    current_method = SoundPropagation::Method_SpecularLOS;
    combo_method.setSelectedId(SoundPropagation::Method_SpecularLOS);

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
    combo_selected_sound.addItem("File", PropagationSource::SOURCE_FILE);
    combo_selected_sound.addItem("Frequency", PropagationSource::SOURCE_FREQUENCY);
    combo_selected_sound.addItem("Off", PropagationSource::SOURCE_OFF);

    {
        MessageManagerLock lock;
        combo_selected_sound.setSelectedId(PropagationSource::SOURCE_OFF);
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
    if (show_spl.load())
    {
        if (flag_refresh_image.load())
        {
            std::lock_guard<std::mutex> guard(mutex_image);
            image_spl = image_next;
            flag_refresh_image.store(false);
        }
        _g.drawImageAt(image_spl, 0, 0);
    }

    const Rectangle<int> bounds = _g.getClipBounds();
    const float zoom_factor = 10.f;
    PaintRoom(_g, bounds, zoom_factor);    
    PaintSimulation(_g, bounds, zoom_factor);
    PaintEmitter(_g, bounds, zoom_factor);
}

void MainComponent::PaintEmitter(Graphics& _g, const Rectangle<int> _bounds, const float _zoom_factor) const
{
    const nMath::Vector center = ImageHelper::Center(_bounds);

    _g.setColour(Colour::fromRGB(0xFF, 0xFF, 0xFF));
    _g.fillEllipse(receiver_x - 1.f, receiver_y - 1.f, 2, 2);

    nMath::Vector emitter_pos;
    {
        std::lock_guard<std::mutex> guard(*mutex_emitter_update);
        emitter_pos = sources[selected_source].moving_emitter->GetPosition();
    }
    nMath::Vector emitter_draw_pos{ emitter_pos.x * _zoom_factor + center.x, -emitter_pos.y * _zoom_factor + center.y, 0.f };
    _g.fillEllipse(emitter_draw_pos.x - 1.f, emitter_draw_pos.y - 1.f, 2.5, 2.5);
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
            std::shared_ptr<const PropagationPlanner> planner = simulation_results[current_read_index].object.planner;
            if (std::shared_ptr<const PlannerAStar> astar_planner
                = std::dynamic_pointer_cast<const PlannerAStar>(planner)) // TODO: planner should know its type.
            {
                const PlannerAStar::GeometryGrid& grid = astar_planner->Grid(); // TODO: handle PlannerTwoStages
                const float min_extent = (float)nMath::Min(_bounds.getWidth(), _bounds.getHeight());
                _g.setColour(Colour::fromRGBA(0x77, 0x77, 0x77, 0x99));
                const int offset = (int)(min_extent / 2.f - _zoom_factor * PlannerAStar::GridDistance / 2);
                const int cellSize = (int)_zoom_factor / PlannerAStar::GridCellsPerMeter;
                for (int i = 0; i < PlannerAStar::GridResolution; ++i)
                {
                    for (int j = 0; j < PlannerAStar::GridResolution; ++j)
                    {
                        if (bool value = grid[i][j])
                        {
                            _g.fillRect(
                                cellSize * j + offset,
                                cellSize * (PlannerAStar::GridResolution - i - 1) + offset,
                                cellSize - 1,
                                cellSize - 1);
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
    //std::lock_guard<std::mutex> guard(mutex_image);    
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
    button_show_spl.setBounds(frame_button_r);
    
    frame_button_l = frame_next();
    frame_button_r = frame_button_l.removeFromRight(frame_button_l.getWidth() / 2);
    button_show_grid.setBounds(frame_button_l);
    button_show_contours.setBounds(frame_button_r);

    frame_button_l = frame_next();
    frame_button_r = frame_button_l.removeFromRight(frame_button_l.getWidth() / 2);
    button_gamma_correct.setBounds(frame_button_r);

    slider_time_scale.setBounds(frame_next());

    juce::Rectangle<int> frame_atmosphere = frame.removeFromTop(116).reduced(2);
    frame_atmosphere.setLeft(frame_atmosphere.getX() - 40);
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
    PropagationSource& source = sources[source_id];
    if (source.source_type != PropagationSource::SOURCE_FILE)
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
    PropagationSource& source = sources[selected_source];
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
    
    if (show_spl.load() &&        
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
                std::lock_guard<std::mutex> guard(mutex_image);
                GenerateSPLImage(image_next, planner, room, time_now, 1.f, true);
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
    else if (slider == &slider_radius ||
             slider == &slider_temperature ||
             slider == &slider_humidity ||
             slider == &slider_pressure)
    {
        const float next_radius = (float)slider_radius.getValue();
        const double next_temperature = slider_temperature.getValue();
        const double next_humidity = slider_humidity.getValue();
        const double next_pressure = slider_pressure.getValue();
        sources[selected_source].moving_emitter->SetRadius(next_radius);
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

void MainComponent::RefreshSourceParams()
{
    const int32 source_id = selected_source.load();
    const PropagationSource& source = sources[source_id];
    if (source.source_type == PropagationSource::SOURCE_FILE)
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
            sources[source_id].source_type = (PropagationSource::SourceType)next_id;
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
        switch (current_method)
        {
        case SoundPropagation::Method_SpecularLOS:
        case SoundPropagation::Method_RayCasts:
        case SoundPropagation::Method_Wave:
            button_show_grid.setEnabled(false);
            break;
        case SoundPropagation::Method_Pathfinding:
        case SoundPropagation::Method_LOSAStarFallback:
            button_show_grid.setEnabled(true);
            break;
        default:
            break;
        }
        planners_refresh = true;
    }
}
