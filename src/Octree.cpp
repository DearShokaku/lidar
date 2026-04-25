#include "Octree.h"
#include <cmath>
#include <algorithm>

OctreeNode::OctreeNode(const QVector3D& minBound, const QVector3D& maxBound)
    : m_minBound(minBound)
    , m_maxBound(maxBound)
    , m_center((minBound + maxBound) / 2.0f)
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

void OctreeNode::subdivide()
{
    QVector3D mid = m_center;
    
    m_children[0] = std::make_unique<OctreeNode>(
        QVector3D(m_minBound.x(), m_minBound.y(), m_minBound.z()),
        QVector3D(mid.x(), mid.y(), mid.z())
    );
    
    m_children[1] = std::make_unique<OctreeNode>(
        QVector3D(m_minBound.x(), m_minBound.y(), mid.z()),
        QVector3D(mid.x(), mid.y(), m_maxBound.z())
    );
    
    m_children[2] = std::make_unique<OctreeNode>(
        QVector3D(m_minBound.x(), mid.y(), m_minBound.z()),
        QVector3D(mid.x(), m_maxBound.y(), mid.z())
    );
    
    m_children[3] = std::make_unique<OctreeNode>(
        QVector3D(m_minBound.x(), mid.y(), mid.z()),
        QVector3D(mid.x(), m_maxBound.y(), m_maxBound.z())
    );
    
    m_children[4] = std::make_unique<OctreeNode>(
        QVector3D(mid.x(), m_minBound.y(), m_minBound.z()),
        QVector3D(m_maxBound.x(), mid.y(), mid.z())
    );
    
    m_children[5] = std::make_unique<OctreeNode>(
        QVector3D(mid.x(), m_minBound.y(), mid.z()),
        QVector3D(m_maxBound.x(), mid.y(), m_maxBound.z())
    );
    
    m_children[6] = std::make_unique<OctreeNode>(
        QVector3D(mid.x(), mid.y(), m_minBound.z()),
        QVector3D(m_maxBound.x(), m_maxBound.y(), mid.z())
    );
    
    m_children[7] = std::make_unique<OctreeNode>(
        QVector3D(mid.x(), mid.y(), mid.z()),
        QVector3D(m_maxBound.x(), m_maxBound.y(), m_maxBound.z())
    );
    
    m_isLeaf = false;
}

void OctreeNode::insert(const PointXYZRGBI& point, size_t maxPointsPerNode, size_t maxDepth, size_t currentDepth)
{
    if (!containsPoint(point.xyz))
    {
        return;
    }
    
    if (m_isLeaf)
    {
        m_points.push_back(point);
        
        if (m_points.size() > maxPointsPerNode && currentDepth < maxDepth)
        {
            subdivide();
            
            for (const auto& p : m_points)
            {
                int octant = getOctantContainingPoint(p.xyz);
                if (m_children[octant])
                {
                    m_children[octant]->insert(p, maxPointsPerNode, maxDepth, currentDepth + 1);
                }
            }
            m_points.clear();
        }
    }
    else
    {
        int octant = getOctantContainingPoint(point.xyz);
        if (m_children[octant])
        {
            m_children[octant]->insert(point, maxPointsPerNode, maxDepth, currentDepth + 1);
        }
    }
}

void OctreeNode::getVisiblePoints(const QVector3D& cameraPos, float viewDistance, 
                                   std::vector<PointXYZRGBI>& visiblePoints) const
{
    QVector3D toCenter = m_center - cameraPos;
    float distance = toCenter.length();
    
    if (distance - m_radius > viewDistance)
    {
        return;
    }
    
    if (m_isLeaf)
    {
        for (const auto& point : m_points)
        {
            float pointDist = (point.xyz - cameraPos).length();
            if (pointDist <= viewDistance)
            {
                visiblePoints.push_back(point);
            }
        }
    }
    else
    {
        for (const auto& child : m_children)
        {
            if (child)
            {
                child->getVisiblePoints(cameraPos, viewDistance, visiblePoints);
            }
        }
    }
}

void OctreeNode::getAllPoints(std::vector<PointXYZRGBI>& allPoints) const
{
    if (m_isLeaf)
    {
        allPoints.insert(allPoints.end(), m_points.begin(), m_points.end());
    }
    else
    {
        for (const auto& child : m_children)
        {
            if (child)
            {
                child->getAllPoints(allPoints);
            }
        }
    }
}

Octree::Octree(size_t maxPointsPerNode, size_t maxDepth)
    : m_maxPointsPerNode(maxPointsPerNode)
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
    
    m_root = std::make_unique<OctreeNode>(pointCloud.getMinBound(), pointCloud.getMaxBound());
    
    const auto& points = pointCloud.getPoints();
    for (const auto& point : points)
    {
        m_root->insert(point, m_maxPointsPerNode, m_maxDepth);
    }
}

void Octree::clear()
{
    m_root.reset();
}

std::vector<PointXYZRGBI> Octree::getVisiblePoints(const QVector3D& cameraPos, float viewDistance) const
{
    std::vector<PointXYZRGBI> visiblePoints;
    
    if (m_root)
    {
        m_root->getVisiblePoints(cameraPos, viewDistance, visiblePoints);
    }
    
    return visiblePoints;
}

std::vector<PointXYZRGBI> Octree::getAllPoints() const
{
    std::vector<PointXYZRGBI> allPoints;
    
    if (m_root)
    {
        m_root->getAllPoints(allPoints);
    }
    
    return allPoints;
}
