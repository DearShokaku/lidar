#include "Octree.h"
#include <cmath>
#include <algorithm>
#include <iostream>

OctreeNode::OctreeNode(const QVector3D& minBound, const QVector3D& maxBound, 
                       const PointCloudData* pointCloudData)
    : m_minBound(minBound)
    , m_maxBound(maxBound)
    , m_center((minBound + maxBound) / 2.0f)
    , m_pointCloudData(pointCloudData)
    , m_isLeaf(true)
{
    QVector3D size = maxBound - minBound;
    m_radius = size.length() / 2.0f;
}

OctreeNode::~OctreeNode()
{
}

bool OctreeNode::isLeaf() const
{
    return m_isLeaf;
}

bool OctreeNode::containsPoint(const QVector3D& point) const
{
    return (point.x() >= m_minBound.x() && point.x() <= m_maxBound.x() &&
            point.y() >= m_minBound.y() && point.y() <= m_maxBound.y() &&
            point.z() >= m_minBound.z() && point.z() <= m_maxBound.z());
}

int OctreeNode::getOctantContainingPoint(const QVector3D& point) const
{
    int octant = 0;
    if (point.x() >= m_center.x()) octant |= 4;
    if (point.y() >= m_center.y()) octant |= 2;
    if (point.z() >= m_center.z()) octant |= 1;
    return octant;
}

void OctreeNode::subdivide(size_t maxPointsPerNode, size_t maxDepth, size_t currentDepth)
{
    QVector3D mid = m_center;
    
    m_children[0] = std::make_unique<OctreeNode>(
        QVector3D(m_minBound.x(), m_minBound.y(), m_minBound.z()),
        QVector3D(mid.x(), mid.y(), mid.z()),
        m_pointCloudData
    );
    
    m_children[1] = std::make_unique<OctreeNode>(
        QVector3D(m_minBound.x(), m_minBound.y(), mid.z()),
        QVector3D(mid.x(), mid.y(), m_maxBound.z()),
        m_pointCloudData
    );
    
    m_children[2] = std::make_unique<OctreeNode>(
        QVector3D(m_minBound.x(), mid.y(), m_minBound.z()),
        QVector3D(mid.x(), m_maxBound.y(), mid.z()),
        m_pointCloudData
    );
    
    m_children[3] = std::make_unique<OctreeNode>(
        QVector3D(m_minBound.x(), mid.y(), mid.z()),
        QVector3D(mid.x(), m_maxBound.y(), m_maxBound.z()),
        m_pointCloudData
    );
    
    m_children[4] = std::make_unique<OctreeNode>(
        QVector3D(mid.x(), m_minBound.y(), m_minBound.z()),
        QVector3D(m_maxBound.x(), mid.y(), mid.z()),
        m_pointCloudData
    );
    
    m_children[5] = std::make_unique<OctreeNode>(
        QVector3D(mid.x(), m_minBound.y(), mid.z()),
        QVector3D(m_maxBound.x(), mid.y(), m_maxBound.z()),
        m_pointCloudData
    );
    
    m_children[6] = std::make_unique<OctreeNode>(
        QVector3D(mid.x(), mid.y(), m_minBound.z()),
        QVector3D(m_maxBound.x(), m_maxBound.y(), mid.z()),
        m_pointCloudData
    );
    
    m_children[7] = std::make_unique<OctreeNode>(
        QVector3D(mid.x(), mid.y(), mid.z()),
        QVector3D(m_maxBound.x(), m_maxBound.y(), m_maxBound.z()),
        m_pointCloudData
    );
    
    m_isLeaf = false;
    
    if (m_pointCloudData)
    {
        const auto& points = m_pointCloudData->getPoints();
        for (size_t idx : m_indices)
        {
            if (idx < points.size())
            {
                const QVector3D& pos = points[idx].xyz;
                int octant = getOctantContainingPoint(pos);
                if (m_children[octant])
                {
                    m_children[octant]->insert(idx, maxPointsPerNode, maxDepth, currentDepth + 1);
                }
            }
        }
    }
    
    m_indices.clear();
}

void OctreeNode::insert(size_t pointIndex, size_t maxPointsPerNode, 
                        size_t maxDepth, size_t currentDepth)
{
    if (!m_pointCloudData)
    {
        return;
    }
    
    const auto& points = m_pointCloudData->getPoints();
    if (pointIndex >= points.size())
    {
        return;
    }
    
    const QVector3D& pointPos = points[pointIndex].xyz;
    
    if (!containsPoint(pointPos))
    {
        return;
    }
    
    if (m_isLeaf)
    {
        m_indices.push_back(pointIndex);
        
        if (m_indices.size() > maxPointsPerNode && currentDepth < maxDepth)
        {
            subdivide(maxPointsPerNode, maxDepth, currentDepth);
        }
    }
    else
    {
        int octant = getOctantContainingPoint(pointPos);
        if (m_children[octant])
        {
            m_children[octant]->insert(pointIndex, maxPointsPerNode, maxDepth, currentDepth + 1);
        }
    }
}

void OctreeNode::getVisibleIndices(const QVector3D& cameraPos, float viewDistance, 
                                    std::vector<size_t>& visibleIndices) const
{
    QVector3D toCenter = m_center - cameraPos;
    float distance = toCenter.length();
    
    if (distance - m_radius > viewDistance)
    {
        return;
    }
    
    if (m_isLeaf)
    {
        visibleIndices.insert(visibleIndices.end(), m_indices.begin(), m_indices.end());
    }
    else
    {
        for (const auto& child : m_children)
        {
            if (child)
            {
                child->getVisibleIndices(cameraPos, viewDistance, visibleIndices);
            }
        }
    }
}

void OctreeNode::getAllIndices(std::vector<size_t>& allIndices) const
{
    if (m_isLeaf)
    {
        allIndices.insert(allIndices.end(), m_indices.begin(), m_indices.end());
    }
    else
    {
        for (const auto& child : m_children)
        {
            if (child)
            {
                child->getAllIndices(allIndices);
            }
        }
    }
}

Octree::Octree(size_t maxPointsPerNode, size_t maxDepth)
    : m_pointCloudData(nullptr)
    , m_maxPointsPerNode(maxPointsPerNode)
    , m_maxDepth(maxDepth)
{
}

Octree::~Octree()
{
}

void Octree::build(const PointCloudData& pointCloud)
{
    if (pointCloud.isEmpty())
    {
        return;
    }
    
    m_pointCloudData = &pointCloud;
    
    m_root = std::make_unique<OctreeNode>(
        pointCloud.getMinBound(), 
        pointCloud.getMaxBound(),
        m_pointCloudData
    );
    
    const auto& points = pointCloud.getPoints();
    for (size_t i = 0; i < points.size(); ++i)
    {
        m_root->insert(i, m_maxPointsPerNode, m_maxDepth);
    }
}

void Octree::clear()
{
    m_root.reset();
    m_pointCloudData = nullptr;
}

std::vector<size_t> Octree::getVisibleIndices(const QVector3D& cameraPos, float viewDistance) const
{
    std::vector<size_t> visibleIndices;
    
    if (m_root)
    {
        m_root->getVisibleIndices(cameraPos, viewDistance, visibleIndices);
    }
    
    return visibleIndices;
}

std::vector<size_t> Octree::getAllIndices() const
{
    std::vector<size_t> allIndices;
    
    if (m_root)
    {
        m_root->getAllIndices(allIndices);
    }
    
    return allIndices;
}

std::vector<PointXYZRGBI> Octree::getVisiblePoints(const QVector3D& cameraPos, float viewDistance) const
{
    std::vector<PointXYZRGBI> visiblePoints;
    
    if (!m_root || !m_pointCloudData)
    {
        return visiblePoints;
    }
    
    std::vector<size_t> indices = getVisibleIndices(cameraPos, viewDistance);
    const auto& points = m_pointCloudData->getPoints();
    
    visiblePoints.reserve(indices.size());
    for (size_t idx : indices)
    {
        if (idx < points.size())
        {
            visiblePoints.push_back(points[idx]);
        }
    }
    
    return visiblePoints;
}

std::vector<PointXYZRGBI> Octree::getAllPoints() const
{
    std::vector<PointXYZRGBI> allPoints;
    
    if (!m_root || !m_pointCloudData)
    {
        return allPoints;
    }
    
    std::vector<size_t> indices = getAllIndices();
    const auto& points = m_pointCloudData->getPoints();
    
    allPoints.reserve(indices.size());
    for (size_t idx : indices)
    {
        if (idx < points.size())
        {
            allPoints.push_back(points[idx]);
        }
    }
    
    return allPoints;
}
