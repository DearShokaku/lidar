#include "OpenGLWidget.h"
#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>
#include <cmath>
#include <iostream>

OpenGLWidget::OpenGLWidget(QWidget *parent)
    : QOpenGLWidget(parent)
    , m_pointCloud(nullptr)
    , m_octree(nullptr)
    , m_colorMapper(nullptr)
    , m_cameraPosition(0.0f, 0.0f, 10.0f)
    , m_cameraTarget(0.0f, 0.0f, 0.0f)
    , m_cameraUp(0.0f, 1.0f, 0.0f)
    , m_rotationX(0.0f)
    , m_rotationY(0.0f)
    , m_zoom(1.0f)
    , m_mouseButton(Qt::NoButton)
    , m_pointSize(2.0f)
    , m_useOctree(false)
    , m_viewDistance(1000.0f)
    , m_leftButtonPressed(false)
    , m_rightButtonPressed(false)
    , m_midButtonPressed(false)
    , m_cachedColorMapType(ColorMapType::Original)
    , m_colorCacheValid(false)
{
    setFocusPolicy(Qt::StrongFocus);
}

OpenGLWidget::~OpenGLWidget()
{
}

void OpenGLWidget::setPointCloud(std::shared_ptr<PointCloudData> pointCloud)
{
    m_pointCloud = pointCloud;
    m_colorCacheValid = false;
    
    if (m_pointCloud && !m_pointCloud->isEmpty())
    {
        resetCamera();
    }
    
    update();
}

void OpenGLWidget::setOctree(std::shared_ptr<Octree> octree)
{
    m_octree = octree;
    update();
}

void OpenGLWidget::setColorMapper(std::shared_ptr<ColorMapper> colorMapper)
{
    m_colorMapper = colorMapper;
    m_colorCacheValid = false;
    update();
}

void OpenGLWidget::invalidateColorCache()
{
    m_colorCacheValid = false;
    if (m_pointCloud)
    {
        m_pointCloud->invalidateColors();
    }
    update();
}

void OpenGLWidget::updateColorCache()
{
    if (!m_pointCloud || m_pointCloud->isEmpty())
    {
        m_colorCacheValid = false;
        return;
    }
    
    ColorMapType currentType = m_colorMapper ? m_colorMapper->getColorMapType() : ColorMapType::Original;
    
    if (m_colorCacheValid && m_cachedColorMapType == currentType)
    {
        return;
    }
    
    auto& points = m_pointCloud->getPointsRef();
    
    if (currentType == ColorMapType::Original)
    {
        for (auto& point : points)
        {
            point.cachedColor[0] = static_cast<uint8_t>(point.rgb[0] * 255.0f + 0.5f);
            point.cachedColor[1] = static_cast<uint8_t>(point.rgb[1] * 255.0f + 0.5f);
            point.cachedColor[2] = static_cast<uint8_t>(point.rgb[2] * 255.0f + 0.5f);
            point.colorValid = true;
        }
    }
    else if (m_colorMapper)
    {
        if (currentType == ColorMapType::Intensity)
        {
            for (auto& point : points)
            {
                m_colorMapper->mapIntensityToColorUint8(
                    point.intensity, 
                    point.cachedColor[0], 
                    point.cachedColor[1], 
                    point.cachedColor[2]
                );
                point.colorValid = true;
            }
        }
        else
        {
            for (auto& point : points)
            {
                m_colorMapper->mapHeightToColorUint8(
                    point.xyz.z(), 
                    point.cachedColor[0], 
                    point.cachedColor[1], 
                    point.cachedColor[2]
                );
                point.colorValid = true;
            }
        }
    }
    
    m_cachedColorMapType = currentType;
    m_colorCacheValid = true;
}

void OpenGLWidget::computePointColor(PointXYZRGBI& point)
{
    if (!m_colorMapper)
    {
        point.cachedColor[0] = static_cast<uint8_t>(point.rgb[0] * 255.0f + 0.5f);
        point.cachedColor[1] = static_cast<uint8_t>(point.rgb[1] * 255.0f + 0.5f);
        point.cachedColor[2] = static_cast<uint8_t>(point.rgb[2] * 255.0f + 0.5f);
        point.colorValid = true;
        return;
    }
    
    ColorMapType currentType = m_colorMapper->getColorMapType();
    
    if (currentType == ColorMapType::Original)
    {
        point.cachedColor[0] = static_cast<uint8_t>(point.rgb[0] * 255.0f + 0.5f);
        point.cachedColor[1] = static_cast<uint8_t>(point.rgb[1] * 255.0f + 0.5f);
        point.cachedColor[2] = static_cast<uint8_t>(point.rgb[2] * 255.0f + 0.5f);
    }
    else if (currentType == ColorMapType::Intensity)
    {
        m_colorMapper->mapIntensityToColorUint8(
            point.intensity,
            point.cachedColor[0],
            point.cachedColor[1],
            point.cachedColor[2]
        );
    }
    else
    {
        m_colorMapper->mapHeightToColorUint8(
            point.xyz.z(),
            point.cachedColor[0],
            point.cachedColor[1],
            point.cachedColor[2]
        );
    }
    point.colorValid = true;
}

void OpenGLWidget::resetCamera()
{
    if (!m_pointCloud || m_pointCloud->isEmpty())
    {
        m_cameraPosition = QVector3D(0.0f, 0.0f, 10.0f);
        m_cameraTarget = QVector3D(0.0f, 0.0f, 0.0f);
        m_rotationX = 0.0f;
        m_rotationY = 0.0f;
        m_zoom = 1.0f;
        update();
        return;
    }
    
    const QVector3D& center = m_pointCloud->getCenter();
    const QVector3D& minBound = m_pointCloud->getMinBound();
    const QVector3D& maxBound = m_pointCloud->getMaxBound();
    
    QVector3D size = maxBound - minBound;
    float maxDim = std::max({size.x(), size.y(), size.z()});
    float distance = maxDim * 2.0f;
    
    m_cameraTarget = center;
    m_cameraPosition = center + QVector3D(0.0f, distance * 0.5f, distance);
    m_rotationX = 0.0f;
    m_rotationY = -30.0f;
    m_zoom = 1.0f;
    m_viewDistance = maxDim * 5.0f;
    
    if (m_colorMapper)
    {
        m_colorMapper->setHeightRange(m_pointCloud->getMinHeight(), 
                                       m_pointCloud->getMaxHeight());
        m_colorMapper->setIntensityRange(m_pointCloud->getMinIntensity(), 
                                          m_pointCloud->getMaxIntensity());
    }
    
    update();
}

void OpenGLWidget::setPointSize(float size)
{
    m_pointSize = size;
    update();
}

void OpenGLWidget::initializeGL()
{
    initializeOpenGLFunctions();
    
    glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void OpenGLWidget::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
    setupProjection();
}

void OpenGLWidget::setupProjection()
{
    m_projectionMatrix.setToIdentity();
    float aspect = static_cast<float>(width()) / static_cast<float>(height());
    m_projectionMatrix.perspective(45.0f, aspect, 0.1f, 10000.0f);
}

void OpenGLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    m_modelViewMatrix.setToIdentity();
    
    QVector3D lookDir = m_cameraTarget - m_cameraPosition;
    float distance = lookDir.length();
    lookDir.normalize();
    
    QVector3D eye = m_cameraPosition;
    QVector3D target = m_cameraTarget;
    QVector3D up = m_cameraUp;
    
    QMatrix4x4 rotationMatrix;
    rotationMatrix.rotate(m_rotationY, 1.0f, 0.0f, 0.0f);
    rotationMatrix.rotate(m_rotationX, 0.0f, 1.0f, 0.0f);
    
    QVector3D rotatedLookDir = rotationMatrix * lookDir;
    rotatedLookDir.normalize();
    
    float zoomedDistance = distance / m_zoom;
    QVector3D rotatedEye = target - rotatedLookDir * zoomedDistance;
    
    QVector3D rotatedUp = rotationMatrix * up;
    rotatedUp.normalize();
    
    m_modelViewMatrix.lookAt(rotatedEye, target, rotatedUp);
    
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(m_projectionMatrix.data());
    
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(m_modelViewMatrix.data());
    
    renderAxis();
    renderPointCloud();
}

void OpenGLWidget::renderPointCloud()
{
    if (!m_pointCloud || m_pointCloud->isEmpty())
    {
        return;
    }
    
    updateColorCache();
    
    glPointSize(m_pointSize);
    glBegin(GL_POINTS);
    
    if (m_useOctree && m_octree && !m_octree->isEmpty())
    {
        std::vector<PointXYZRGBI> pointsToRender;
        pointsToRender = m_octree->getVisiblePoints(m_cameraPosition, m_viewDistance);
        
        for (const auto& point : pointsToRender)
        {
            if (point.colorValid)
            {
                glColor3ub(point.cachedColor[0], point.cachedColor[1], point.cachedColor[2]);
            }
            else
            {
                PointXYZRGBI& mutablePoint = const_cast<PointXYZRGBI&>(point);
                computePointColor(mutablePoint);
                glColor3ub(point.cachedColor[0], point.cachedColor[1], point.cachedColor[2]);
            }
            glVertex3f(point.xyz.x(), point.xyz.y(), point.xyz.z());
        }
    }
    else
    {
        const auto& points = m_pointCloud->getPoints();
        
        for (const auto& point : points)
        {
            glColor3ub(point.cachedColor[0], point.cachedColor[1], point.cachedColor[2]);
            glVertex3f(point.xyz.x(), point.xyz.y(), point.xyz.z());
        }
    }
    
    glEnd();
}

void OpenGLWidget::renderAxis()
{
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(1.0f, 0.0f, 0.0f);
    
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 1.0f, 0.0f);
    
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 1.0f);
    
    glEnd();
}

void OpenGLWidget::mousePressEvent(QMouseEvent *event)
{
    m_lastMousePos = event->pos();
    m_mouseButton = event->button();
    
    if (event->button() == Qt::LeftButton)
    {
        m_leftButtonPressed = true;
    }
    else if (event->button() == Qt::RightButton)
    {
        m_rightButtonPressed = true;
    }
    else if (event->button() == Qt::MiddleButton)
    {
        m_midButtonPressed = true;
    }
    
    setFocus();
}

void OpenGLWidget::mouseMoveEvent(QMouseEvent *event)
{
    int dx = event->x() - m_lastMousePos.x();
    int dy = event->y() - m_lastMousePos.y();
    
    if (m_leftButtonPressed)
    {
        m_rotationX += static_cast<float>(dx) * 0.5f;
        m_rotationY += static_cast<float>(dy) * 0.5f;
        
        m_rotationY = std::max(-89.0f, std::min(89.0f, m_rotationY));
        
        update();
    }
    else if (m_midButtonPressed || (m_leftButtonPressed && m_rightButtonPressed))
    {
        float panSpeed = 0.01f / m_zoom;
        
        QVector3D lookDir = m_cameraTarget - m_cameraPosition;
        lookDir.normalize();
        
        QVector3D right = QVector3D::crossProduct(lookDir, m_cameraUp);
        right.normalize();
        
        QVector3D up = m_cameraUp;
        
        QVector3D pan = -right * static_cast<float>(dx) * panSpeed + up * static_cast<float>(dy) * panSpeed;
        
        m_cameraPosition += pan;
        m_cameraTarget += pan;
        
        update();
    }
    
    m_lastMousePos = event->pos();
}

void OpenGLWidget::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        m_leftButtonPressed = false;
    }
    else if (event->button() == Qt::RightButton)
    {
        m_rightButtonPressed = false;
    }
    else if (event->button() == Qt::MiddleButton)
    {
        m_midButtonPressed = false;
    }
    
    m_mouseButton = Qt::NoButton;
}

void OpenGLWidget::wheelEvent(QWheelEvent *event)
{
    float delta = event->angleDelta().y() / 120.0f;
    float zoomFactor = 1.1f;
    
    if (delta > 0)
    {
        m_zoom *= zoomFactor;
    }
    else
    {
        m_zoom /= zoomFactor;
    }
    
    m_zoom = std::max(0.1f, std::min(10.0f, m_zoom));
    
    update();
}

void OpenGLWidget::keyPressEvent(QKeyEvent *event)
{
    float moveSpeed = 0.5f / m_zoom;
    
    QVector3D lookDir = m_cameraTarget - m_cameraPosition;
    lookDir.normalize();
    
    QVector3D right = QVector3D::crossProduct(lookDir, m_cameraUp);
    right.normalize();
    
    switch (event->key())
    {
        case Qt::Key_W:
            m_cameraPosition += lookDir * moveSpeed;
            m_cameraTarget += lookDir * moveSpeed;
            break;
        case Qt::Key_S:
            m_cameraPosition -= lookDir * moveSpeed;
            m_cameraTarget -= lookDir * moveSpeed;
            break;
        case Qt::Key_A:
            m_cameraPosition -= right * moveSpeed;
            m_cameraTarget -= right * moveSpeed;
            break;
        case Qt::Key_D:
            m_cameraPosition += right * moveSpeed;
            m_cameraTarget += right * moveSpeed;
            break;
        case Qt::Key_Q:
            m_cameraPosition += m_cameraUp * moveSpeed;
            m_cameraTarget += m_cameraUp * moveSpeed;
            break;
        case Qt::Key_E:
            m_cameraPosition -= m_cameraUp * moveSpeed;
            m_cameraTarget -= m_cameraUp * moveSpeed;
            break;
        case Qt::Key_R:
            resetCamera();
            break;
        default:
            QOpenGLWidget::keyPressEvent(event);
            return;
    }
    
    update();
}
