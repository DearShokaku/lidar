#ifndef COLORMAPPER_H
#define COLORMAPPER_H

#include <vector>
#include <cstdint>

enum class ColorMapType
{
    Height,
    Intensity,
    Original,
    Rainbow,
    Heat,
    Viridis
};

class ColorMapper
{
public:
    ColorMapper();
    ~ColorMapper();
    
    void setColorMapType(ColorMapType type) { m_colorMapType = type; }
    ColorMapType getColorMapType() const { return m_colorMapType; }
    
    void setHeightRange(float minHeight, float maxHeight);
    void setIntensityRange(float minIntensity, float maxIntensity);
    
    void mapToColor(float value, float minValue, float maxValue, 
                    float& r, float& g, float& b) const;
    
    void mapToColorUint8(float value, float minValue, float maxValue,
                         uint8_t& r, uint8_t& g, uint8_t& b) const;
    
    void mapHeightToColor(float height, float& r, float& g, float& b) const;
    void mapHeightToColorUint8(float height, uint8_t& r, uint8_t& g, uint8_t& b) const;
    
    void mapIntensityToColor(float intensity, float& r, float& g, float& b) const;
    void mapIntensityToColorUint8(float intensity, uint8_t& r, uint8_t& g, uint8_t& b) const;
    
    static void rainbowColorMap(float t, float& r, float& g, float& b);
    static void rainbowColorMapUint8(float t, uint8_t& r, uint8_t& g, uint8_t& b);
    
    static void heatColorMap(float t, float& r, float& g, float& b);
    static void heatColorMapUint8(float t, uint8_t& r, uint8_t& g, uint8_t& b);
    
    static void viridisColorMap(float t, float& r, float& g, float& b);
    static void viridisColorMapUint8(float t, uint8_t& r, uint8_t& g, uint8_t& b);
    
    static void grayscaleColorMap(float t, float& r, float& g, float& b);
    static void grayscaleColorMapUint8(float t, uint8_t& r, uint8_t& g, uint8_t& b);

private:
    ColorMapType m_colorMapType;
    float m_minHeight;
    float m_maxHeight;
    float m_minIntensity;
    float m_maxIntensity;
};

#endif // COLORMAPPER_H
