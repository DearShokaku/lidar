#ifndef POINTCLOUDDATA_H
#define POINTCLOUDDATA_H

#include <vector>
#include <QVector3D>

struct PointXYZRGBI
{
    QVector3D xyz;
    float intensity;
    float rgb[3];
    
    PointXYZRGBI() : intensity(0.0f)
    {
        rgb[0] = rgb[1] = rgb[2] = 1.0f;
    }
    
    PointXYZRGBI(float x, float y, float z, float i = 0.0f)
        : xyz(x, y, z), intensity(i)
    {
        rgb[0] = rgb[1] = rgb[2] = 1.0f;
    }
};

class PointCloudData
{
public:
    PointCloudData();
    ~PointCloudData();
    
    bool isEmpty() const;
    size_t size() const;
    
    void addPoint(const PointXYZRGBI& point);
    const PointXYZRGBI& getPoint(size_t index) const;
    
    const QVector3D& getMinBound() const;
    const QVector3D& getMaxBound() const;
    const QVector3D& getCenter() const;
    
    float getMinIntensity() const;
    float getMaxIntensity() const;
    float getMinHeight() const;
    float getMaxHeight() const;
    
    void clear();
    void computeBounds();
    
    const std::vector<PointXYZRGBI>& getPoints() const;

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
