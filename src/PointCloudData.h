#ifndef POINTCLOUDDATA_H
#define POINTCLOUDDATA_H

#include <vector>
#include <QVector3D>
#include <cstdint>

struct PointXYZRGBI
{
    QVector3D xyz;
    float intensity;
    float rgb[3];
    uint8_t cachedColor[3];
    bool colorValid;
    
    PointXYZRGBI() : intensity(0.0f), colorValid(false)
    {
        rgb[0] = rgb[1] = rgb[2] = 1.0f;
        cachedColor[0] = cachedColor[1] = cachedColor[2] = 255;
    }
    
    PointXYZRGBI(float x, float y, float z, float i = 0.0f)
        : xyz(x, y, z), intensity(i), colorValid(false)
    {
        rgb[0] = rgb[1] = rgb[2] = 1.0f;
        cachedColor[0] = cachedColor[1] = cachedColor[2] = 255;
    }
};

class PointCloudData
{
public:
    static const size_t BATCH_READ_SIZE = 8 * 1024 * 1024;
    
    PointCloudData();
    ~PointCloudData();
    
    bool isEmpty() const;
    size_t size() const;
    
    void reserve(size_t count);
    void addPoint(const PointXYZRGBI& point);
    void addPointsBatch(const std::vector<PointXYZRGBI>& points);
    const PointXYZRGBI& getPoint(size_t index) const;
    PointXYZRGBI& getPointRef(size_t index);
    
    const QVector3D& getMinBound() const;
    const QVector3D& getMaxBound() const;
    const QVector3D& getCenter() const;
    
    float getMinIntensity() const;
    float getMaxIntensity() const;
    float getMinHeight() const;
    float getMaxHeight() const;
    
    void clear();
    void computeBounds();
    void invalidateColors();
    
    const std::vector<PointXYZRGBI>& getPoints() const;
    std::vector<PointXYZRGBI>& getPointsRef();

private:
    std::vector<PointXYZRGBI> m_points;
    
    QVector3D m_minBound;
    QVector3D m_maxBound;
    QVector3D m_center;
    
    float m_minIntensity;
    float m_maxIntensity;
    float m_minHeight;
    float m_maxHeight;
};

#endif // POINTCLOUDDATA_H
