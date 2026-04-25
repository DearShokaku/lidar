#ifndef COLORMAPPER_H
#define COLORMAPPER_H

#include <Eigen/Core>
#include <vector>

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
    
    void mapHeightToColor(float height, float& r, float& g, float& b) const;
    void mapIntensityToColor(float intensity, float& r, float& g, float& b) const;
    
    static void rainbowColorMap(float t, float& r, float& g, float& b);
    static void heatColorMap(float t, float& r, float& g, float& b);
    static void viridisColorMap(float t, float& r, float& g, float& b);
    static void grayscaleColorMap(float t, float& r, float& g, float& b);

private:
    ColorMapType m_colorMapType;
    float m_minHeight;
    float m_maxHeight;
    float m_minIntensity;
    float m_maxIntensity;
};

#endif // COLORMAPPER_H
