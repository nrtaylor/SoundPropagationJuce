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

//#define PROFILE_SIMULATION

namespace ImageHelper
{   
    Image SquareImage(const Rectangle<int>& bounds)
    {
        const int extent = jmin(bounds.getWidth(), bounds.getHeight());
        return Image(Image::ARGB, extent, extent, true);
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
    selected_test_buffer(0),
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

    ray_cast_collector = std::make_unique<RayCastCollector>();
    mutex_ray_cast_collector = std::make_unique<std::mutex>();

    mutex_emitter_update = std::make_unique<std::mutex>();

    setAudioChannels(0, 2);
    setWantsKeyboardFocus(true);

    button_source[0].setButtonText("Source 1");
    button_source[0].setToggleState(true, dontSendNotification);
    button_source[0].onClick = [this]() { label_selected_sound.setText("Source 1", dontSendNotification); };
    button_source[1].setButtonText("Source 2");
    button_source[1].onClick = [this]() { label_selected_sound.setText("Source 2", dontSendNotification); };
    button_source[2].setButtonText("Source 3");
    button_source[2].onClick = [this]() { label_selected_sound.setText("Source 3", dontSendNotification); };
    for (int i = 0; i < 3; ++i)
    {
        button_source[i].setRadioGroupId(1001);
        button_source[i].setClickingTogglesState(true);
        addAndMakeVisible(button_source[i]);
    }

    button_loadfile.setButtonText("...");
    button_loadfile.onClick = [this]() {
        FileChooser chooser("Select Sound File", File::getCurrentWorkingDirectory(), "*.wav;*.aiff");
        if (chooser.browseForFileToOpen())
        {
            AudioFormatManager format_manager; format_manager.registerBasicFormats();
            const File& file = chooser.getResult();
            SoundBufferHelper::LoadFromFile(test_buffers[0], format_manager, file);
            label_loadfile.setText(file.getFileName(), dontSendNotification);
        }
    };
    addChildComponent(button_loadfile);
    label_loadfile.setText("[None]", dontSendNotification);
    addChildComponent(label_loadfile);
    
    addAndMakeVisible(&slider_gain);
    slider_gain.setRange(0.0, 2.0, 0.05);
    slider_gain.setTextValueSuffix(" %");
    slider_gain.setValue(0.8);
    slider_gain.addListener(this);

    addAndMakeVisible(&label_gain);
    label_gain.setText("Gain", dontSendNotification);
    label_gain.attachToComponent(&slider_gain, true);

    addAndMakeVisible(&slider_freq);
    slider_freq.setRange(0, 8.0, 0.001);
    slider_freq.setTextValueSuffix(" Hz");
    slider_freq.setValue(0.2);
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

    addAndMakeVisible(&button_show_spl);
    button_show_spl.setButtonText("Draw SPL");
    button_show_spl.addListener(this);
    button_show_spl.onClick = [this]() { show_spl = button_show_spl.getToggleState(); };

    addAndMakeVisible(&button_show_ray_casts);
    button_show_ray_casts.setButtonText("Ray Casts");
    button_show_ray_casts.addListener(this);
    button_show_ray_casts.onClick = [this]() { show_ray_casts = button_show_ray_casts.getToggleState(); };

    addAndMakeVisible(&button_show_grid);
    button_show_grid.setButtonText("Draw Grid");
    button_show_grid.addListener(this);
    button_show_grid.onClick = [this]() { show_grid = button_show_grid.getToggleState(); };

    addAndMakeVisible(&button_show_contours);
    button_show_contours.setButtonText("Contours");
    button_show_contours.addListener(this);
    button_show_contours.onClick = [this]() { show_contours = button_show_contours.getToggleState(); };

    addAndMakeVisible(&button_gamma_correct);
    button_gamma_correct.setButtonText("Gamma");
    button_gamma_correct.addListener(this);
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
    moving_emitter = std::make_unique<MovingEmitter>();

    current_room = nullptr;
    addAndMakeVisible(&combo_room);    
    combo_room.addListener(this);

    planner_astar = std::make_unique<PlannerAStar>();
    planner_wave = std::make_unique<PlannerWave>();
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
    combo_method.addItem("Wave Equation", SoundPropagation::Method_Wave);
    current_method = SoundPropagation::Method_SpecularLOS;
    combo_method.setSelectedId(SoundPropagation::Method_SpecularLOS);

    addAndMakeVisible(&label_method);
    label_method.setText("Method", dontSendNotification);
    label_method.attachToComponent(&combo_method, true);

    addAndMakeVisible(&combo_compare_to_method);
    combo_compare_to_method.addListener(this);
    combo_compare_to_method.addItem("Off", SoundPropagation::Method_Off);
    combo_compare_to_method.addItem("Specular (LOS)", SoundPropagation::Method_SpecularLOS);
    combo_compare_to_method.addItem("Ray Casts", SoundPropagation::Method_RayCasts);
    combo_compare_to_method.addItem("A*", SoundPropagation::Method_Pathfinding);
    current_compare_to_method = SoundPropagation::Method_Off;
    combo_compare_to_method.setSelectedId(SoundPropagation::Method_Off);

    addAndMakeVisible(&label_compare_to_method);
    label_compare_to_method.setText("Compare to", dontSendNotification);
    label_compare_to_method.attachToComponent(&combo_compare_to_method, true);
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
    combo_selected_sound.addItem("File", SOURCE_FILE);
    combo_selected_sound.addItem("Frequency", SOURCE_FREQUENCY);
    combo_selected_sound.addItem("Off", SOURCE_OFF);

    {
        MessageManagerLock lock;
        combo_selected_sound.setSelectedId(SOURCE_OFF);
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

    if (show_ray_casts)
    {
        PaintRayCasts(_g, bounds, zoom_factor);
    }

    PaintEmitter(_g, bounds, zoom_factor);
}

void MainComponent::PaintEmitter(Graphics& _g, const Rectangle<int> _bounds, const float _zoom_factor) const
{
    const float min_extent = (float)std::min(_bounds.getWidth(), _bounds.getHeight());
    const nMath::Vector center{ min_extent / 2.f, min_extent / 2.f, 0.f };

    _g.setColour(Colour::fromRGB(0xFF, 0xFF, 0xFF));
    _g.fillEllipse(receiver_x - 1.f, receiver_y - 1.f, 2, 2);

    nMath::Vector emitter_pos;
    {
        std::lock_guard<std::mutex> guard(*mutex_emitter_update);
        emitter_pos = moving_emitter->GetPosition();
    }
    nMath::Vector emitter_draw_pos{ emitter_pos.x * _zoom_factor + center.x, -emitter_pos.y * _zoom_factor + center.y, 0.f };
    _g.fillEllipse(emitter_draw_pos.x - 1.f, emitter_draw_pos.y - 1.f, 2.5, 2.5);
}

void MainComponent::PaintRoom(Graphics& _g, const Rectangle<int> _bounds, const float _zoom_factor) const
{
    std::shared_ptr<RoomGeometry> room = current_room;
    if (room != nullptr)
    {
        const float min_extent = (float)std::min(_bounds.getWidth(), _bounds.getHeight());
        const nMath::Vector center{ min_extent / 2.f, min_extent / 2.f, 0.f };

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

        //if (show_grid.load())
        //{
        //    const std::unique_ptr<RoomGeometry::GeometryGrid>& grid = room->Grid();
        //    if (grid != nullptr)
        //    {
        //        _g.setColour(Colour::fromRGBA(0x77, 0x77, 0x77, 0x99));
        //        int offset = (int)(min_extent / 2.f - _zoom_factor * RoomGeometry::GridDistance / 2);
        //        int cellSize = 10 / RoomGeometry::GridCellsPerMeter;
        //        for (int i = 0; i < RoomGeometry::GridResolution; ++i)
        //        {
        //            for (int j = 0; j < RoomGeometry::GridResolution; ++j)
        //            {
        //                if (bool value = (*grid)[i][j])
        //                {
        //                    _g.fillRect(
        //                        cellSize * j + offset + 1,
        //                        cellSize * (RoomGeometry::GridResolution - i - 1) + offset - 1,
        //                        cellSize - 1,
        //                        cellSize - 1);
        //                }
        //            }
        //        }
        //    }
        //}
    }
}

void MainComponent::PaintRayCasts(Graphics& _g, const Rectangle<int> _bounds, const float _zoom_factor) const
{
    if (ray_cast_collector != nullptr)
    {
        std::lock_guard<std::mutex> guard(*mutex_ray_cast_collector);
        auto& ray_casts = ray_cast_collector->RayCasts();
        if (ray_casts.size() > 0)
        {
            const float min_extent = (float)std::min(_bounds.getWidth(), _bounds.getHeight());
            const nMath::Vector center{ min_extent / 2.f, min_extent / 2.f, 0.f };
            _g.setColour(Colour::fromRGB(0x0, 0xAA, 0xAA));
            for (const nMath::LineSegment line : ray_casts)
            {
                _g.drawLine((line.start.x * _zoom_factor) + center.x,
                    -(line.start.y * _zoom_factor) + center.y,
                    (line.end.x * _zoom_factor) + center.x,
                    -(line.end.y * _zoom_factor) + center.y);
            }
        }        
    }
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
    combo_compare_to_method.setBounds(frame_next());
    combo_room.setBounds(frame_next());

    juce::Rectangle<int> frame_button_l = frame_next();
    juce::Rectangle<int> frame_button_r = frame_button_l.removeFromRight(frame_button_l.getWidth() / 2);
    button_show_spl.setBounds(frame_button_l);
    button_show_ray_casts.setBounds(frame_button_r);
    
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
    const float min_extent = (float)std::min(getBounds().getWidth(), getBounds().getHeight());
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
    const uint32 selected_buffer_id = selected_test_buffer.load();
    if (selected_buffer_id < 0 ||
        selected_buffer_id >= test_buffers.size())
    {
        return;
    }
    const int channels = bufferToFill.buffer->getNumChannels();

    int samples_remaining = bufferToFill.numSamples;
    int sample_offset = bufferToFill.startSample;
    SoundBuffer& buffer_info = test_buffers[selected_buffer_id];
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

    const float gain_left = moving_emitter->Gain(0);
    const float gain_right = moving_emitter->Gain(1);    

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

// Animated Component
void MainComponent::update()
{
    int32 frame_time = Time::getMillisecondCounter();
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
    const SoundPropagation::MethodType simulation_method = current_method.load();
    if (room != nullptr)
    {
        const bool ray_casts = show_ray_casts;
        if (ray_casts)
        {
            std::lock_guard<std::mutex> guard(*mutex_ray_cast_collector);
            ray_cast_collector->Reset();
            room->SwapCollector(ray_cast_collector);
        }
        const float min_extent = (float)std::min(getBounds().getWidth(), getBounds().getHeight());
        const nMath::Vector center{ min_extent / 2.f, min_extent / 2.f, 0.f };
        const float inv_zoom_factor = 1.f/10.f;
        const nMath::Vector receiever_pos = { (receiver_x - center.x) * inv_zoom_factor , (receiver_y - center.y) * -inv_zoom_factor, 0.f};
        const float simulated_gain = room->Simulate<true>(emitter_pos, receiever_pos, simulation_method);
        moving_emitter->ComputeGain(simulated_gain);
        if (ray_casts)
        {
            room->SwapCollector(ray_cast_collector);
        }
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

            const float time_now = (float)(Time::currentTimeMillis() % ((1 + (int)test_frequency) * 1000));// TODO: start at t = 0 or store in planner.

            bool overlay_contours = show_contours.load();
            bool perform_refresh = planners_refresh.load();
            planners_refresh = false;

            std::thread worker = std::thread([this, simulation_method, emitter_pos, overlay_contours, perform_refresh, time_now] {
                std::lock_guard<std::mutex> guard(mutex_image);
                std::shared_ptr<RoomGeometry> room = current_room;
                const SoundPropagation::MethodType simulation_compare_to = current_compare_to_method.load();
                const nMath::Vector center{ image_next.getWidth() / 2.f, image_next.getWidth() / 2.f, 0.f };
                const float inv_zoom_factor = 1.f / 10.f;

                const int extent = image_next.getWidth();
                const Image::BitmapData bitmap(image_next, Image::BitmapData::writeOnly);

#ifdef PROFILE_SIMULATION
                Thread::sleep(5000);
#endif
                const bool using_planner = simulation_method == SoundPropagation::MethodType::Method_Pathfinding ||
                                           simulation_method == SoundPropagation::MethodType::Method_Wave;
                if (using_planner)
                {
                    if (simulation_method == SoundPropagation::MethodType::Method_Pathfinding)
                    {
                        if (perform_refresh)
                        {
                            planner_astar->Preprocess(*room);
                        }
                        planner_astar->Plan(emitter_pos);
                    }
                    else
                    {
                        if (perform_refresh)
                        {
                            planner_wave->Preprocess(*room);
                        }
                        planner_wave->Plan(emitter_pos, test_frequency.load(), time_scale.load());
                    }
                }
                uint8* pixel = bitmap.getPixelPointer(0, 0);

                // for contours
                std::vector<float> previous_row; previous_row.resize(extent);
                std::fill(previous_row.begin(), previous_row.end(), FLT_MAX);
                
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
                        const nMath::Vector pixel_to_world = { (j - center.x) * inv_zoom_factor,
                                                           (i - center.y) * -inv_zoom_factor,
                                                            0.f };
                        float energy = using_planner ?
                            (simulation_method == SoundPropagation::MethodType::Method_Pathfinding ?
                             planner_astar->Simulate(pixel_to_world) 
                                : planner_wave->Simulate(*room, pixel_to_world, time_now)) 
                             : room->Simulate(emitter_pos, pixel_to_world, simulation_method);
                        if (simulation_compare_to != SoundPropagation::Method_Off)
                        {
                            float compare_to_energy = room->Simulate(emitter_pos, pixel_to_world, simulation_compare_to);
                            energy = fabs(energy - compare_to_energy);                            
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
                        if (flag_gamma_correct.load())
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
        moving_emitter->SetGlobalGain((float)slider_gain.getValue());
    }
    else if (slider == &slider_freq)
    {
        moving_emitter->SetFrequency((float)slider_freq.getValue());
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

void MainComponent::buttonClicked(Button* buttonClicked)
{
    (void)buttonClicked;
}

void MainComponent::comboBoxChanged(ComboBox* comboBoxThatHasChanged)
{
    if (comboBoxThatHasChanged == &combo_selected_sound)
    {
        const int32 next_id = combo_selected_sound.getSelectedId();
        if (next_id > 0)
        {
            if (next_id == SOURCE_FILE)
            {
                selected_test_buffer = 0;
                button_loadfile.setVisible(true);
                label_loadfile.setVisible(true);
                slider_spl_freq.setVisible(false);
            }
            else
            {
                selected_test_buffer = -1;
                button_loadfile.setVisible(false);
                label_loadfile.setVisible(false);
                slider_spl_freq.setVisible(true);
            }
        }
    }
    else if (comboBoxThatHasChanged == &combo_room)
    {
        const uint32 next_id = combo_room.getSelectedId();
        if (next_id > 0)
        {
            current_room = rooms[next_id - 1];
            planners_refresh = true;
        }
    }
    else if (comboBoxThatHasChanged == &combo_method)
    {
        current_method = static_cast<SoundPropagation::MethodType>(combo_method.getSelectedId());
        planners_refresh = true;
    }
    else if (comboBoxThatHasChanged == &combo_compare_to_method)
    {
        current_compare_to_method = static_cast<SoundPropagation::MethodType>(combo_compare_to_method.getSelectedId());
    }
}
