#ifndef OCTREE_H
#define OCTREE_H

#include "PointCloudData.h"
#include <QVector3D>
#include <vector>
#include <memory>

class OctreeNode
{
public:
    OctreeNode(const QVector3D& minBound, const QVector3D& maxBound);
    ~OctreeNode();
    
    bool isLeaf() const;
    bool containsPoint(const QVector3D& point) const;
    
    void insert(const PointXYZRGBI& point, size_t maxPointsPerNode, size_t maxDepth, size_t currentDepth = 0);
    
    void getVisiblePoints(const QVector3D& cameraPos, float viewDistance, 
                          std::vector<PointXYZRGBI>& visiblePoints) const;
    
    void getAllPoints(std::vector<PointXYZRGBI>& allPoints) const;
    
    const QVector3D& getMinBound() const { return m_minBound; }
    const QVector3D& getMaxBound() const { return m_maxBound; }
    const QVector3D& getCenter() const { return m_center; }
    float getRadius() const { return m_radius; }

private:
    void subdivide();
    int getOctantContainingPoint(const QVector3D& point) const;
    
    QVector3D m_minBound;
    QVector3D m_maxBound;
    QVector3D m_center;
    float m_radius;
    
    std::vector<PointXYZRGBI> m_points;
    std::unique_ptr<OctreeNode> m_children[8];
    bool m_isLeaf;
};

class Octree
{
public:
    Octree(size_t maxPointsPerNode = 100, size_t maxDepth = 10);
    ~Octree();
    
    void build(const PointCloudData& pointCloud);
    void clear();
    
    std::vector<PointXYZRGBI> getVisiblePoints(const QVector3D& cameraPos, float viewDistance) const;
    std::vector<PointXYZRGBI> getAllPoints() const;
    
    void setMaxPointsPerNode(size_t maxPoints) { m_maxPointsPerNode = maxPoints; }
    void setMaxDepth(size_t maxDepth) { m_maxDepth = maxDepth; }
    
    size_t getMaxPointsPerNode() const { return m_maxPointsPerNode; }
    size_t getMaxDepth() const { return m_maxDepth; }
    bool isEmpty() const { return !m_root; }

private:
    std::unique_ptr<OctreeNode> m_root;
    size_t m_maxPointsPerNode;
    size_t m_maxDepth;
};

#endif // OCTREE_H
