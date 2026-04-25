#include "PointCloudData.h"
#include <limits>

PointCloudData::PointCloudData()
    : m_minBound(std::numeric_limits<float>::max(), 
                 std::numeric_limits<float>::max(), 
                 std::numeric_limits<float>::max())
    , m_maxBound(std::numeric_limits<float>::lowest(), 
                 std::numeric_limits<float>::lowest(), 
                 std::numeric_limits<float>::lowest())
    , m_center(0.0f, 0.0f, 0.0f)
    , m_minIntensity(std::numeric_limits<float>::max())
    , m_maxIntensity(std::numeric_limits<float>::lowest())
    , m_minHeight(std::numeric_limits<float>::max())
    , m_maxHeight(std::numeric_limits<float>::lowest())
{
}

PointCloudData::~PointCloudData()
{
}

bool PointCloudData::isEmpty() const
{
    return m_points.empty();
}

size_t PointCloudData::size() const
{
    return m_points.size();
}

void PointCloudData::addPoint(const PointXYZRGBI& point)
{
    m_points.push_back(point);
    
    if (point.xyz.x() < m_minBound.x()) m_minBound.setX(point.xyz.x());
    if (point.xyz.y() < m_minBound.y()) m_minBound.setY(point.xyz.y());
    if (point.xyz.z() < m_minBound.z()) m_minBound.setZ(point.xyz.z());
    
    if (point.xyz.x() > m_maxBound.x()) m_maxBound.setX(point.xyz.x());
    if (point.xyz.y() > m_maxBound.y()) m_maxBound.setY(point.xyz.y());
    if (point.xyz.z() > m_maxBound.z()) m_maxBound.setZ(point.xyz.z());
    
    if (point.intensity < m_minIntensity) m_minIntensity = point.intensity;
    if (point.intensity > m_maxIntensity) m_maxIntensity = point.intensity;
    
    if (point.xyz.z() < m_minHeight) m_minHeight = point.xyz.z();
    if (point.xyz.z() > m_maxHeight) m_maxHeight = point.xyz.z();
}

const PointXYZRGBI& PointCloudData::getPoint(size_t index) const
{
    return m_points[index];
}

const QVector3D& PointCloudData::getMinBound() const
{
    return m_minBound;
}

const QVector3D& PointCloudData::getMaxBound() const
{
    return m_maxBound;
}

const QVector3D& PointCloudData::getCenter() const
{
    return m_center;
}

float PointCloudData::getMinIntensity() const
{
    return m_minIntensity;
}

float PointCloudData::getMaxIntensity() const
{
    return m_maxIntensity;
}

float PointCloudData::getMinHeight() const
{
    return m_minHeight;
}

float PointCloudData::getMaxHeight() const
{
    return m_maxHeight;
}

void PointCloudData::clear()
{
    m_points.clear();
    m_minBound = QVector3D(std::numeric_limits<float>::max(), 
                            std::numeric_limits<float>::max(), 
                            std::numeric_limits<float>::max());
    m_maxBound = QVector3D(std::numeric_limits<float>::lowest(), 
                            std::numeric_limits<float>::lowest(), 
                            std::numeric_limits<float>::lowest());
    m_center = QVector3D(0.0f, 0.0f, 0.0f);
    m_minIntensity = std::numeric_limits<float>::max();
    m_maxIntensity = std::numeric_limits<float>::lowest();
    m_minHeight = std::numeric_limits<float>::max();
    m_maxHeight = std::numeric_limits<float>::lowest();
}

void PointCloudData::computeBounds()
{
    if (m_points.empty())
    {
        return;
    }
    
    m_minBound = QVector3D(std::numeric_limits<float>::max(), 
                            std::numeric_limits<float>::max(), 
                            std::numeric_limits<float>::max());
    m_maxBound = QVector3D(std::numeric_limits<float>::lowest(), 
                            std::numeric_limits<float>::lowest(), 
                            std::numeric_limits<float>::lowest());
    m_minIntensity = std::numeric_limits<float>::max();
    m_maxIntensity = std::numeric_limits<float>::lowest();
    m_minHeight = std::numeric_limits<float>::max();
    m_maxHeight = std::numeric_limits<float>::lowest();
    
    for (const auto& point : m_points)
    {
        if (point.xyz.x() < m_minBound.x()) m_minBound.setX(point.xyz.x());
        if (point.xyz.y() < m_minBound.y()) m_minBound.setY(point.xyz.y());
        if (point.xyz.z() < m_minBound.z()) m_minBound.setZ(point.xyz.z());
        
        if (point.xyz.x() > m_maxBound.x()) m_maxBound.setX(point.xyz.x());
        if (point.xyz.y() > m_maxBound.y()) m_maxBound.setY(point.xyz.y());
        if (point.xyz.z() > m_maxBound.z()) m_maxBound.setZ(point.xyz.z());
        
        if (point.intensity < m_minIntensity) m_minIntensity = point.intensity;
        if (point.intensity > m_maxIntensity) m_maxIntensity = point.intensity;
        
        if (point.xyz.z() < m_minHeight) m_minHeight = point.xyz.z();
        if (point.xyz.z() > m_maxHeight) m_maxHeight = point.xyz.z();
    }
    
    m_center = (m_minBound + m_maxBound) / 2.0f;
}

const std::vector<PointXYZRGBI>& PointCloudData::getPoints() const
{
    return m_points;
}
