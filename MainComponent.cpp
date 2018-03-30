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
    void LoadFromFile(SoundBuffer& buffer, AudioFormatManager& format_manager, const char* file_name)
    {
        File file = File::getCurrentWorkingDirectory().getChildFile(file_name);
        std::unique_ptr<AudioFormatReader> reader(format_manager.createReaderFor(file));

        if (reader != nullptr)
        {
            buffer.buffer = std::make_unique<AudioSampleBuffer>();
            buffer.buffer->setSize(reader->numChannels, static_cast<int>(reader->lengthInSamples));
            reader->read(buffer.buffer.get(),
                0,
                static_cast<int>(reader->lengthInSamples),
                0,
                true,
                true);
        }
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
    flag_refresh_image = false;
    flag_update_working = false;

    ray_cast_collector = std::make_unique<RayCastCollector>();
    mutex_ray_cast_collector = std::make_unique<std::mutex>();

    mutex_emitter_update = std::make_unique<std::mutex>();

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

    addAndMakeVisible(&button_show_ray_casts);
    button_show_ray_casts.setButtonText("Ray Casts");
    button_show_ray_casts.addListener(this);

    addAndMakeVisible(&slider_spl_freq);
    slider_spl_freq.setRange(20.0, 20000.0, 0.5);
    slider_spl_freq.setTextValueSuffix(" Hz");
    slider_spl_freq.setValue(1000.0);
    slider_spl_freq.setSkewFactorFromMidPoint(440.0);
    slider_spl_freq.addListener(this);

    addAndMakeVisible(&label_spl_freq);
    label_spl_freq.setText("Test Freq", dontSendNotification);
    label_spl_freq.attachToComponent(&slider_spl_freq, true);

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

    atmospheric_filters[0] = std::make_unique<nDSP::Butterworth1Pole>();
    atmospheric_filters[0]->bypass = true;
    atmospheric_filters[1] = std::make_unique<nDSP::Butterworth1Pole>();
    atmospheric_filters[1]->bypass = true;
    moving_emitter = std::make_unique<MovingEmitter>();

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
    // Square
    {
        std::shared_ptr<RoomGeometry> room = std::make_shared<RoomGeometry>(RoomGeometry());
        room->AddWall({ -12.f,  8.f, 0.f }, { 12.f,  8.f, 0.f });
        room->AddWall({ 12.f,  8.f, 0.f }, { 12.f, -8.f, 0.f });
        room->AddWall({ 12.f, -8.f, 0.f }, { -12.f, -8.f, 0.f });
        room->AddWall({ -12.f, -8.f, 0.f }, { -12.f,  8.f, 0.f });
        rooms.emplace_back(room);
        combo_room.addItem("Square", static_cast<int>(rooms.size()));
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

    addAndMakeVisible(&combo_method);
    combo_method.addListener(this);
    combo_method.addItem("Specular (LOS)", SoundPropagation::Method_SpecularLOS);
    combo_method.addItem("Ray Casts", SoundPropagation::Method_RayCasts);
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
    {
        AudioFormatManager format_manager; format_manager.registerBasicFormats();
               
        SoundBufferHelper::LoadFromFile(test_buffers[0], format_manager, "..\\..\\..\\lake_mono_2chnl.wav");
        strcpy(test_buffers[0].name, "Lake");
        combo_selected_sound.addItem(test_buffers[0].name, 1);
        SoundBufferHelper::LoadFromFile(test_buffers[1], format_manager, "..\\..\\..\\test_tones.wav");
        strcpy(test_buffers[1].name, "Two Tones");
        combo_selected_sound.addItem(test_buffers[1].name, 2);
        combo_selected_sound.addItem("Off", 3);
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
    _g.fillEllipse(center.x - 1.f, center.y - 1.f, 2, 2);

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

    const int32 new_width = getWidth();
    combo_method.setBounds(new_width - 202, 200 - 90, 200, 20);
    combo_compare_to_method.setBounds(new_width - 202, 200 - 68, 200, 20);
    combo_selected_sound.setBounds(new_width - 202, 200 - 46, 200, 20);
    combo_room.setBounds(new_width - 202, 200 - 24, 200, 20);

    slider_gain.setBounds(new_width - 202, 200, 200, 20);
    slider_freq.setBounds(new_width - 202, 200 + 22, 200, 20);
    slider_radius.setBounds(new_width - 202, 200 + 46, 200, 20);
    button_show_spl.setBounds(new_width - 202, 200 + 68, 200, 20);
    button_show_ray_casts.setBounds(new_width - 104, 200 + 68, 200, 20);
    slider_spl_freq.setBounds(new_width - 202, 200 + 90, 200, 20);
    group_atmosphere.setBounds(new_width - 244, 200 + 120, 238, 116);
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

void MainComponent::getNextAudioBlock(const AudioSourceChannelInfo& bufferToFill)
{
    bufferToFill.clearActiveBufferRegion();
    if (!initialized)
    {
        return;
    }
    const uint32 selected_buffer_id = selected_test_buffer.load();
    if (selected_buffer_id >= test_buffers.size())
    {
        return;
    }
    const int channels = bufferToFill.buffer->getNumChannels();

    int samples_remaining = bufferToFill.numSamples;
    int sample_offset = bufferToFill.startSample;
    SoundBuffer& buffer_info = test_buffers[selected_buffer_id];
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

// Animated Component
void MainComponent::update()
{
    int32 frame_time = Time::getMillisecondCounter();
    nMath::Vector emitter_pos;
    {
        std::lock_guard<std::mutex> guard(*mutex_emitter_update);
        emitter_pos = moving_emitter->Update(frame_time - start_time);
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
        const float simulated_gain = room->Simulate<true>(emitter_pos, { 0.f, 0.f, 0.f }, simulation_method);
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

            std::thread worker = std::thread([this, simulation_method, emitter_pos] {
                std::lock_guard<std::mutex> guard(mutex_image);
                std::shared_ptr<const RoomGeometry> room = current_room;
                const SoundPropagation::MethodType simulation_compare_to = current_compare_to_method.load();                
                const nMath::Vector center{ image_next.getWidth() / 2.f, image_next.getWidth() / 2.f, 0.f };
                const float inv_zoom_factor = 1.f / 10.f;

                const int extent = image_next.getWidth();
                const Image::BitmapData bitmap(image_next, Image::BitmapData::writeOnly);

                uint8* pixel = bitmap.getPixelPointer(0, 0);
                for (int i = 0; i < extent; ++i)
                {
                    for (int j = 0; j < extent; ++j)
                    {
                        const nMath::Vector pixel_to_world = { (j - center.x) * inv_zoom_factor,
                                                           (i - center.y) * -inv_zoom_factor,
                                                            0.f };
                        float energy = room->Simulate(emitter_pos, pixel_to_world, simulation_method);
                        if (simulation_compare_to != SoundPropagation::Method_Off)
                        {
                            float compare_to_energy = room->Simulate(emitter_pos, pixel_to_world, simulation_compare_to);
                            energy = fabs(energy - compare_to_energy);                            
                        }
                        const uint8 colour = (uint8)jmin<uint32>(255, (uint32)(255.f*energy));
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
    if (buttonClicked == &button_show_spl)
    {
        show_spl = button_show_spl.getToggleState();
    }
    else if (buttonClicked == &button_show_ray_casts)
    {
        show_ray_casts = button_show_ray_casts.getToggleState();
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
    else if (comboBoxThatHasChanged == &combo_room)
    {
        const uint32 next_id = combo_room.getSelectedId();
        if (next_id > 0)
        {
            current_room = rooms[next_id - 1];
        }
    }
    else if (comboBoxThatHasChanged == &combo_method)
    {
        current_method = static_cast<SoundPropagation::MethodType>(combo_method.getSelectedId());
    }
    else if (comboBoxThatHasChanged == &combo_compare_to_method)
    {
        current_compare_to_method = static_cast<SoundPropagation::MethodType>(combo_compare_to_method.getSelectedId());
    }
}
