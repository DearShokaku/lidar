#include "ColorMapper.h"
#include <algorithm>
#include <cmath>

ColorMapper::ColorMapper()
    : m_colorMapType(ColorMapType::Height)
    , m_minHeight(0.0f)
    , m_maxHeight(100.0f)
    , m_minIntensity(0.0f)
    , m_maxIntensity(255.0f)
{
}

ColorMapper::~ColorMapper()
{
}

void ColorMapper::setHeightRange(float minHeight, float maxHeight)
{
    m_minHeight = minHeight;
    m_maxHeight = maxHeight;
}

void ColorMapper::setIntensityRange(float minIntensity, float maxIntensity)
{
    m_minIntensity = minIntensity;
    m_maxIntensity = maxIntensity;
}

void ColorMapper::mapToColor(float value, float minValue, float maxValue, 
                              float& r, float& g, float& b) const
{
    float range = maxValue - minValue;
    float t = 0.0f;
    
    if (range > 0.0001f)
    {
        t = (value - minValue) / range;
        t = std::max(0.0f, std::min(1.0f, t));
    }
    
    switch (m_colorMapType)
    {
        case ColorMapType::Height:
        case ColorMapType::Rainbow:
            rainbowColorMap(t, r, g, b);
            break;
        case ColorMapType::Intensity:
            grayscaleColorMap(t, r, g, b);
            break;
        case ColorMapType::Heat:
            heatColorMap(t, r, g, b);
            break;
        case ColorMapType::Viridis:
            viridisColorMap(t, r, g, b);
            break;
        case ColorMapType::Original:
        default:
            r = g = b = 1.0f;
            break;
    }
}

void ColorMapper::mapHeightToColor(float height, float& r, float& g, float& b) const
{
    mapToColor(height, m_minHeight, m_maxHeight, r, g, b);
}

void ColorMapper::mapIntensityToColor(float intensity, float& r, float& g, float& b) const
{
    mapToColor(intensity, m_minIntensity, m_maxIntensity, r, g, b);
}

void ColorMapper::rainbowColorMap(float t, float& r, float& g, float& b)
{
    r = 0.0f;
    g = 0.0f;
    b = 0.0f;
    
    if (t < 0.2f)
    {
        float nt = t / 0.2f;
        r = 0.0f;
        g = 0.0f;
        b = 0.5f + 0.5f * nt;
    }
    else if (t < 0.4f)
    {
        float nt = (t - 0.2f) / 0.2f;
        r = 0.0f;
        g = nt;
        b = 1.0f;
    }
    else if (t < 0.6f)
    {
        float nt = (t - 0.4f) / 0.2f;
        r = 0.0f;
        g = 1.0f;
        b = 1.0f - nt;
    }
    else if (t < 0.8f)
    {
        float nt = (t - 0.6f) / 0.2f;
        r = nt;
        g = 1.0f;
        b = 0.0f;
    }
    else
    {
        float nt = (t - 0.8f) / 0.2f;
        r = 1.0f;
        g = 1.0f - nt;
        b = 0.0f;
    }
}

void ColorMapper::heatColorMap(float t, float& r, float& g, float& b)
{
    r = 0.0f;
    g = 0.0f;
    b = 0.0f;
    
    if (t < 0.33f)
    {
        float nt = t / 0.33f;
        r = 0.0f;
        g = 0.0f;
        b = nt;
    }
    else if (t < 0.66f)
    {
        float nt = (t - 0.33f) / 0.33f;
        r = nt;
        g = 0.0f;
        b = 1.0f - nt;
    }
    else
    {
        float nt = (t - 0.66f) / 0.34f;
        r = 1.0f;
        g = nt;
        b = 0.0f;
    }
}

void ColorMapper::viridisColorMap(float t, float& r, float& g, float& b)
{
    static const float viridisData[25][3] = {
        {0.267004f, 0.004874f, 0.329415f},
        {0.268510f, 0.009605f, 0.335427f},
        {0.269944f, 0.014625f, 0.341379f},
        {0.271305f, 0.019942f, 0.347269f},
        {0.272594f, 0.025563f, 0.353093f},
        {0.273809f, 0.031497f, 0.358853f},
        {0.274952f, 0.037752f, 0.364543f},
        {0.276022f, 0.044167f, 0.370164f},
        {0.277018f, 0.050344f, 0.375715f},
        {0.277941f, 0.056324f, 0.381191f},
        {0.278791f, 0.062145f, 0.386592f},
        {0.279566f, 0.067836f, 0.391917f},
        {0.280267f, 0.073417f, 0.397163f},
        {0.280894f, 0.078907f, 0.402329f},
        {0.281446f, 0.084320f, 0.407414f},
        {0.281924f, 0.089666f, 0.412415f},
        {0.282327f, 0.094955f, 0.417331f},
        {0.282656f, 0.100196f, 0.422160f},
        {0.282910f, 0.105393f, 0.426902f},
        {0.283091f, 0.110553f, 0.431554f},
        {0.283197f, 0.115680f, 0.436115f},
        {0.283229f, 0.120777f, 0.440584f},
        {0.283187f, 0.125848f, 0.444960f},
        {0.283072f, 0.130895f, 0.449241f},
        {0.282884f, 0.135920f, 0.453427f}
    };
    
    int idx = static_cast<int>(t * 24.0f);
    idx = std::max(0, std::min(24, idx));
    
    r = viridisData[idx][0];
    g = viridisData[idx][1];
    b = viridisData[idx][2];
}

void ColorMapper::grayscaleColorMap(float t, float& r, float& g, float& b)
{
    r = t;
    g = t;
    b = t;
}

void ColorMapper::mapToColorUint8(float value, float minValue, float maxValue,
                                   uint8_t& r, uint8_t& g, uint8_t& b) const
{
    float fr, fg, fb;
    mapToColor(value, minValue, maxValue, fr, fg, fb);
    r = static_cast<uint8_t>(fr * 255.0f + 0.5f);
    g = static_cast<uint8_t>(fg * 255.0f + 0.5f);
    b = static_cast<uint8_t>(fb * 255.0f + 0.5f);
}

void ColorMapper::mapHeightToColorUint8(float height, uint8_t& r, uint8_t& g, uint8_t& b) const
{
    mapToColorUint8(height, m_minHeight, m_maxHeight, r, g, b);
}

void ColorMapper::mapIntensityToColorUint8(float intensity, uint8_t& r, uint8_t& g, uint8_t& b) const
{
    mapToColorUint8(intensity, m_minIntensity, m_maxIntensity, r, g, b);
}

void ColorMapper::rainbowColorMapUint8(float t, uint8_t& r, uint8_t& g, uint8_t& b)
{
    float fr, fg, fb;
    rainbowColorMap(t, fr, fg, fb);
    r = static_cast<uint8_t>(fr * 255.0f + 0.5f);
    g = static_cast<uint8_t>(fg * 255.0f + 0.5f);
    b = static_cast<uint8_t>(fb * 255.0f + 0.5f);
}

void ColorMapper::heatColorMapUint8(float t, uint8_t& r, uint8_t& g, uint8_t& b)
{
    float fr, fg, fb;
    heatColorMap(t, fr, fg, fb);
    r = static_cast<uint8_t>(fr * 255.0f + 0.5f);
    g = static_cast<uint8_t>(fg * 255.0f + 0.5f);
    b = static_cast<uint8_t>(fb * 255.0f + 0.5f);
}

void ColorMapper::viridisColorMapUint8(float t, uint8_t& r, uint8_t& g, uint8_t& b)
{
    float fr, fg, fb;
    viridisColorMap(t, fr, fg, fb);
    r = static_cast<uint8_t>(fr * 255.0f + 0.5f);
    g = static_cast<uint8_t>(fg * 255.0f + 0.5f);
    b = static_cast<uint8_t>(fb * 255.0f + 0.5f);
}

void ColorMapper::grayscaleColorMapUint8(float t, uint8_t& r, uint8_t& g, uint8_t& b)
{
    uint8_t val = static_cast<uint8_t>(t * 255.0f + 0.5f);
    r = val;
    g = val;
    b = val;
}
