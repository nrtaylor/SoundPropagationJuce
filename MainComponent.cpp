/*
  ==============================================================================
  SoundPropagation - main Juce class
  ==============================================================================
*/

#include "MainComponent.h"
#include "AtmosphericAbsorption.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <atomic>
#include "SignalProcessing.h"

namespace ImageHelper
{
    Image SquareImage(const Rectangle<int>& bounds)
    {
        const int extent = jmin(bounds.getWidth(), bounds.getHeight());
        return Image(Image::ARGB, extent, extent, true);
    }
}

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

struct LineSegment
{
    Vector3D<float> start;
    Vector3D<float> end;
};

class RayCastCollector
{
public:
    RayCastCollector() {}

    void Start()
    {
        collector_lock.lock();
        ray_casts.clear();
    }

    void Add(const LineSegment& ray_cast)
    {
        ray_casts.emplace_back(ray_cast);
    }

    void Finished()
    {
        collector_lock.unlock();
    }

    void Paint(Graphics& _g, const Rectangle<int> _bounds, const float _zoom_factor)
    {
        std::lock_guard<std::mutex> guard(collector_lock);
        if (ray_casts.size() > 0)
        {
            const float min_extent = (float)jmin(_bounds.getWidth(), _bounds.getHeight());
            const Vector3D<float> center(min_extent / 2.f, min_extent / 2.f, 0.f);
            _g.setColour(Colour::fromRGB(0x0, 0xAA, 0xAA));
            for (const LineSegment line : ray_casts)
            {
                _g.drawLine((line.start.x * _zoom_factor) + center.x,
                    -(line.start.y * _zoom_factor) + center.y,
                    (line.end.x * _zoom_factor) + center.x,
                    -(line.end.y * _zoom_factor) + center.y);
            }
        }
    }
private:
    std::vector<LineSegment> ray_casts;
    std::mutex collector_lock;
};

class RoomGeometry
{
public:
    RoomGeometry() : 
        bounding_box{ {},{} }
    {
        walls.reserve(4);
    }

    void AddWall(const Vector3D<float> start, const Vector3D<float> end)
    {
        walls.emplace_back(LineSegment{ start, end });
        if (walls.size() == 1)
        {
            bounding_box = LineSegment{ 
                { jmin(start.x, end.x), jmin(start.y,end.y), 0.f},
                { jmax(start.x, end.x), jmax(start.y,end.y), 0.f } };
        }
        else
        {
            if (jmin(start.x, end.x) < bounding_box.start.x)
            {
                bounding_box.start.x = jmin(start.x, end.x);
            }
            if (jmax(start.x, end.x) > bounding_box.end.x)
            {
                bounding_box.end.x = jmax(start.x, end.x);
            }
            if (jmin(start.y, end.y) < bounding_box.start.y)
            {
                bounding_box.start.y = jmin(start.y, end.y);
            }
            if (jmax(start.y, end.y) > bounding_box.end.y)
            {
                bounding_box.end.y = jmax(start.y, end.y);
            }
        }
    }

    template<bool capture_debug = false>
    float Simulate(const Vector3D<float>& source, const Vector3D<float>& receiver, const SoundPropagation::MethodType method) const
    {
        switch (method)
        {
        case SoundPropagation::Method_SpecularLOS:
            return SimulateSpecularLOS<capture_debug>(source, receiver);
        case SoundPropagation::Method_RayCasts:
            return SimulateRayCasts<capture_debug>(source, receiver);
        default:
            return 0.f;
        }
    }

    void AssignCollector(std::unique_ptr<RayCastCollector>& collector)
    {
        if (collector != nullptr)
        {
            ray_cast_collector = std::move(collector);
        }
        else if (ray_cast_collector != nullptr)
        {
            collector = std::move(ray_cast_collector);
        }
    }

    template<bool capture_debug = false>
    bool Intersects(const LineSegment& _line) const
    {
        if (capture_debug &&
            ray_cast_collector != nullptr)
        {
            ray_cast_collector->Add(_line);
        }

        if (jmin(_line.start.x, _line.end.x) > bounding_box.end.x ||
            jmax(_line.start.x, _line.end.x) < bounding_box.start.x ||
            jmin(_line.start.y, _line.end.y) > bounding_box.end.y ||
            jmax(_line.start.y, _line.end.y) < bounding_box.start.y)
        {
            return false;
        }

        const Vector3D<float> line2 = _line.end - _line.start;
        for (const LineSegment& wall : walls)
        {
            Vector3D<float> wall2 = wall.end - wall.start;
            const float numerator = wall2.y * (_line.start.x - wall.start.x) - wall2.x * (_line.start.y - wall.start.y);
            const float denominator = line2.y * wall2.x - line2.x * wall2.y;

            if (denominator != 0.f)
            {
                float r = numerator / denominator;
                if (r >= 0.f && r <= 1.f)
                {
                    const float numerator2 = line2.y * (wall.start.x - _line.start.x) - line2.x * (wall.start.y - _line.start.y);
                    const float denominator2 = wall2.y * line2.x - wall2.x * line2.y;
                    r = numerator2 / denominator2;
                    if (r >= 0.f && r <= 1.f)
                    {
                        return true;
                    }
                }
            }
        }

        return false;
    }

    void Paint(Graphics& _g, const Rectangle<int> _bounds, const float _zoom_factor) const
    {
        const float min_extent = (float)jmin(_bounds.getWidth(), _bounds.getHeight());
        const Vector3D<float> center(min_extent / 2.f, min_extent / 2.f, 0.f);

        _g.setColour(Colour::fromRGB(0x77, 0x1c, 0x47));
        Path room;
        for (const LineSegment& wall : walls)
        {
            const Line<float> drawLine(
                wall.start.x * _zoom_factor + center.x,
                -wall.start.y * _zoom_factor + center.y,
                wall.end.x * _zoom_factor + center.x,
                -wall.end.y * _zoom_factor + center.y);
            room.addLineSegment(drawLine, 2.f);
        }
        _g.fillPath(room);
    }
private:
    std::vector<LineSegment> walls;
    LineSegment bounding_box;
    std::unique_ptr<RayCastCollector> ray_cast_collector;

    template<bool capture_debug = false>
    float SimulateSpecularLOS(const Vector3D<float>& source, const Vector3D<float>& receiver) const
    {
        if (Intersects<capture_debug>(LineSegment{ source, receiver }))
        {
            return 0.f;
        }

        static const float near_field_distance = 0.75f; // around 440 hz

        const float distance = (source - receiver).length();
        if (distance < near_field_distance)
        {
            return 1.f;
        }
        const float geometric_attenuation = jmin(1.f, 1.f / distance);
        return geometric_attenuation;
    }

    template<bool capture_debug = false>
    float SimulateRayCasts(const Vector3D<float>& source, const Vector3D<float>& receiver) const
    {        
        Vector3D<float> direction = source - receiver;
        float distance = direction.length();
        if (Intersects<capture_debug>(LineSegment{ source, receiver }))
        {
            if (distance > FLT_EPSILON)
            {
                direction = direction / distance;
                Vector3D<float> ray_orth = {-direction.y, direction.x, 0.f};
                Vector3D<float> ray_orth_inv = ray_orth * -1.f;
                if ((!Intersects<capture_debug>(LineSegment{ ray_orth + receiver, receiver }) &&
                     !Intersects<capture_debug>(LineSegment{ ray_orth + receiver, source })) ||
                    (!Intersects<capture_debug>(LineSegment{ ray_orth_inv + receiver, receiver }) &&
                     !Intersects<capture_debug>(LineSegment{ ray_orth_inv + receiver, source })))
                {
                    distance += 1.f;
                }
                else if ((!Intersects<capture_debug>(LineSegment{ ray_orth + source, source }) &&
                          !Intersects<capture_debug>(LineSegment{ ray_orth + source, receiver })) ||
                         (!Intersects<capture_debug>(LineSegment{ ray_orth_inv + source, source }) &&
                          !Intersects<capture_debug>(LineSegment{ ray_orth_inv + source, receiver })))
                {
                    distance += 1.f;
                }
                else
                {
                    return 0.f;
                }
            }
            else
            {
                return 0.f;
            }            
        }

        static const float near_field_distance = 0.75f; // around 440 hz
        if (distance < near_field_distance)
        {
            return 1.f;
        }
        const float geometric_attenuation = jmin(1.f, 1.f / distance);
        return geometric_attenuation;
    }
};

class MovingEmitter
{
public:
    MovingEmitter() :
        frequency(0.2f),
        radius(10.f),
        angle(0.f)
    {
        gain_left = 0.f;
        gain_right = 0.f;
        pan_amount = 0.f;
        global_gain = 0.8f;
    }

    void Update(int32 _elapsedMs)
    {
        angle += 2 * (float)M_PI * (_elapsedMs * frequency) / 1000.f;        
        Vector3D<float> new_position(cosf(angle), sinf(angle), 0.f);        
        pan_amount = new_position.x;

        const float emitter_radius = radius.load();
        emitter.SetPosition(Vector3D<float>(new_position.x*emitter_radius, new_position.y*emitter_radius, 0.f));
    }

    // gain left/right should be pulled out as a process
    void ComputeGain(const float new_gain)
    {
        // Factor such that panned hard left/right will have the same rms as pan center.
        const float normed_loudness = new_gain * (float)M_SQRT1_2;
        gain_left.store(normed_loudness * sqrtf(1.f - pan_amount));
        gain_right.store(normed_loudness * sqrtf(1.f + pan_amount));
    }

    const Vector3D<float> GetPosition() const
    {
        return emitter.GetPosition();
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
    
    void Paint(Graphics& _g, const Rectangle<int> _bounds, const float _zoom_factor)
    {        
        const float min_extent = (float)jmin(_bounds.getWidth(), _bounds.getHeight());
        const Vector3D<float> center(min_extent / 2.f, min_extent / 2.f, 0.f);

        _g.setColour(Colour::fromRGB(0xFF, 0xFF, 0xFF));
        _g.fillEllipse(center.x - 1.f, center.y - 1.f, 2, 2);

        const Vector3D<float>& emitter_pos = emitter.GetPosition();
        Vector3D<float> emitter_draw_pos(emitter_pos.x * _zoom_factor + center.x, -emitter_pos.y * _zoom_factor + center.y, 0.f);
        _g.fillEllipse(emitter_draw_pos.x - 1.f, emitter_draw_pos.y - 1.f, 2.5, 2.5);
    }
private:
    std::atomic<float> frequency;
    std::atomic<float> global_gain;
    std::atomic<float> radius;
    float angle;    
    std::atomic<float> gain_left;
    std::atomic<float> gain_right;
    SoundEmitter emitter;
    float pan_amount;
};

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

    atmospheric_filters[0] = std::make_unique<NicDSP::Butterworth1Pole>();
    atmospheric_filters[0]->bypass = true;
    atmospheric_filters[1] = std::make_unique<NicDSP::Butterworth1Pole>();
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
        combo_room.addItem("Empty", rooms.size());
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
        combo_room.addItem("Square", rooms.size());        
    }
    // Wall
    {
        std::shared_ptr<RoomGeometry> room = std::make_shared<RoomGeometry>(RoomGeometry());
        room->AddWall({ -1000.f, 0.f, 0.f }, { 1000, 0.f, 0.f });
        rooms.emplace_back(room);
        combo_room.addItem("Wall", rooms.size());        
    }
    // Slit
    {
        std::shared_ptr<RoomGeometry> room = std::make_shared<RoomGeometry>(RoomGeometry());
        room->AddWall({ -16.f,  0.f, 0.f }, { -0.5f,  0.f, 0.f });
        room->AddWall({ 0.5f,  0.f, 0.f }, { 16.f, 0.f, 0.f });
        rooms.emplace_back(room);
        combo_room.addItem("Slit", rooms.size());        
    }
    // Two Slits
    {
        std::shared_ptr<RoomGeometry> room = std::make_shared<RoomGeometry>(RoomGeometry());
        room->AddWall({ -18.f,  0.f, 0.f }, { -6.5f,  0.f, 0.f });
        room->AddWall({ -5.5f,  0.f, 0.f }, { 5.5f, 0.f, 0.f });
        room->AddWall({ 6.5f,  0.f, 0.f }, { 18.f, 0.f, 0.f });
        rooms.emplace_back(room);
        combo_room.addItem("Two Slits", rooms.size());
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
        combo_room.addItem("Trap Room with Opening", rooms.size());
    }
    // Small Obstructions
    {
        std::shared_ptr<RoomGeometry> room = std::make_shared<RoomGeometry>(RoomGeometry());
        room->AddWall({ -3.f,  6.f, 0.f }, { 3.f,  6.f, 0.f });
        room->AddWall({ 3.f, -6.f, 0.f }, { -3.f, -8.f, 0.f });
        rooms.emplace_back(room);
        combo_room.addItem("Two Small Obstructors", rooms.size());
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

namespace SoundBufferHelper
{
    void LoadFromFile(SoundBuffer& buffer, AudioFormatManager& format_manager, const char* file_name)
    {
        File file = File::getCurrentWorkingDirectory().getChildFile(file_name);
        std::unique_ptr<AudioFormatReader> reader(format_manager.createReaderFor(file));

        if (reader != nullptr)
        {
            buffer.buffer = std::make_unique<AudioSampleBuffer>();
            buffer.buffer->setSize(reader->numChannels, reader->lengthInSamples);
            reader->read(buffer.buffer.get(),
                0,
                reader->lengthInSamples,
                0,
                true,
                true);
        }
    }
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
    std::shared_ptr<RoomGeometry> room = current_room;
    if (room != nullptr)
    {
        room->Paint(_g, bounds, zoom_factor);
    }
    if (show_ray_casts &&
        ray_cast_collector != nullptr)
    {
        ray_cast_collector->Paint(_g, bounds, zoom_factor);
    }
    moving_emitter->Paint(_g, bounds, zoom_factor);
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
            NicDSP::Butterworth1Pole& lpf = *atmospheric_filters[channel].get();
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
    moving_emitter->Update(frame_time - start_time);
    std::shared_ptr<RoomGeometry> room = current_room; // this isn't guarunteed atomic
    const SoundPropagation::MethodType simulation_method = current_method.load();
    if (room != nullptr)
    {
        const bool ray_casts = show_ray_casts;
        if (ray_casts)
        {
            ray_cast_collector->Start();
            room->AssignCollector(ray_cast_collector);
        }
        const float simulated_gain = room->Simulate<true>(moving_emitter->GetPosition(), { 0.f, 0.f, 0.f }, simulation_method);
        moving_emitter->ComputeGain(simulated_gain);
        if (ray_casts)
        {
            room->AssignCollector(ray_cast_collector);
            ray_cast_collector->Finished();
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

            std::thread worker = std::thread([this, simulation_method] {
                std::lock_guard<std::mutex> guard(mutex_image);
                std::shared_ptr<const RoomGeometry> room = current_room;
                const SoundPropagation::MethodType simulation_compare_to = current_compare_to_method.load();
                const Vector3D<float> emitter_pos = moving_emitter->GetPosition();
                const Vector3D<float> center(image_next.getWidth() / 2.f, image_next.getWidth() / 2.f, 0.f);
                const float inv_zoom_factor = 1.f / 10.f;

                const int extent = image_next.getWidth();                
                const Image::BitmapData bitmap(image_next, Image::BitmapData::writeOnly);

                uint8* pixel = bitmap.getPixelPointer(0, 0);
                for (int i = 0; i < extent; ++i)
                {
                    for (int j = 0; j < extent; ++j)
                    {
                        const Vector3D<float> pixel_to_world = { (j - center.x) * inv_zoom_factor,
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