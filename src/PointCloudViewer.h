#ifndef POINTCLOUDVIEWER_H
#define POINTCLOUDVIEWER_H

#include <QMainWindow>
#include <QMenu>
#include <QMenuBar>
#include <QToolBar>
#include <QStatusBar>
#include <QDockWidget>
#include <QSlider>
#include <QComboBox>
#include <QCheckBox>
#include <QLabel>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <memory>

#include "PointCloudData.h"
#include "Octree.h"
#include "ColorMapper.h"
#include "OpenGLWidget.h"

class PointCloudViewer : public QMainWindow
{
    Q_OBJECT

public:
    explicit PointCloudViewer(QWidget *parent = nullptr);
    ~PointCloudViewer();

private slots:
    void openFile();
    void saveFile();
    void exitApp();
    
    void about();
    
    void colorMapChanged(int index);
    void pointSizeChanged(int value);
    void useOctreeToggled(bool checked);
    void octreeMaxPointsChanged(int value);
    void octreeMaxDepthChanged(int value);
    void viewDistanceChanged(double value);
    void rebuildOctree();
    
    void resetView();

private:
    void createMenus();
    void createToolBars();
    void createStatusBar();
    void createDockWidgets();
    
    void loadFile(const QString& filename);
    bool loadPCDFile(const QString& filename);
    bool loadLASFile(const QString& filename);
    
    void updateStatusBar();
    
    OpenGLWidget *m_openGLWidget;
    
    std::shared_ptr<PointCloudData> m_pointCloud;
    std::shared_ptr<Octree> m_octree;
    std::shared_ptr<ColorMapper> m_colorMapper;
    
    QMenu *m_fileMenu;
    QMenu *m_viewMenu;
    QMenu *m_helpMenu;
    
    QToolBar *m_fileToolBar;
    QToolBar *m_viewToolBar;
    
    QAction *m_openAction;
    QAction *m_saveAction;
    QAction *m_exitAction;
    QAction *m_resetViewAction;
    QAction *m_aboutAction;
    
    QDockWidget *m_controlDock;
    QWidget *m_controlWidget;
    
    QComboBox *m_colorMapCombo;
    QSlider *m_pointSizeSlider;
    QLabel *m_pointSizeLabel;
    QCheckBox *m_useOctreeCheck;
    QSpinBox *m_octreeMaxPointsSpin;
    QSpinBox *m_octreeMaxDepthSpin;
    QDoubleSpinBox *m_viewDistanceSpin;
    QPushButton *m_rebuildOctreeButton;
    
    QLabel *m_statusLabel;
    QLabel *m_pointsLabel;
};

#endif // POINTCLOUDVIEWER_H
