#ifndef OPENGLWIDGET_H
#define OPENGLWIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QMatrix4x4>
#include <QPoint>
#include <QVector3D>
#include <memory>

#include "PointCloudData.h"
#include "Octree.h"
#include "ColorMapper.h"

class OpenGLWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

public:
    explicit OpenGLWidget(QWidget *parent = nullptr);
    ~OpenGLWidget();
    
    void setPointCloud(std::shared_ptr<PointCloudData> pointCloud);
    void setOctree(std::shared_ptr<Octree> octree);
    void setColorMapper(std::shared_ptr<ColorMapper> colorMapper);
    
    void resetCamera();
    void setPointSize(float size);
    float getPointSize() const { return m_pointSize; }
    
    void setUseOctree(bool use) { m_useOctree = use; update(); }
    bool getUseOctree() const { return m_useOctree; }
    
    void setViewDistance(float distance) { m_viewDistance = distance; update(); }
    float getViewDistance() const { return m_viewDistance; }

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;
    void keyPressEvent(QKeyEvent *event) override;

private:
    void renderPointCloud();
    void renderAxis();
    void setupProjection();
    
    std::shared_ptr<PointCloudData> m_pointCloud;
    std::shared_ptr<Octree> m_octree;
    std::shared_ptr<ColorMapper> m_colorMapper;
    
    QMatrix4x4 m_projectionMatrix;
    QMatrix4x4 m_modelViewMatrix;
    
    QVector3D m_cameraPosition;
    QVector3D m_cameraTarget;
    QVector3D m_cameraUp;
    
    float m_rotationX;
    float m_rotationY;
    float m_zoom;
    
    QPoint m_lastMousePos;
    Qt::MouseButton m_mouseButton;
    
    float m_pointSize;
    bool m_useOctree;
    float m_viewDistance;
    
    bool m_leftButtonPressed;
    bool m_rightButtonPressed;
    bool m_midButtonPressed;
};

#endif // OPENGLWIDGET_H
