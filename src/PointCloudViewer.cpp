#include "PointCloudViewer.h"
#include "PCDReader.h"
#include "LASReader.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QPushButton>
#include <QLabel>
#include <QSplitter>
#include <iostream>

PointCloudViewer::PointCloudViewer(QWidget *parent)
    : QMainWindow(parent)
    , m_openGLWidget(nullptr)
    , m_pointCloud(std::make_shared<PointCloudData>())
    , m_octree(std::make_shared<Octree>())
    , m_colorMapper(std::make_shared<ColorMapper>())
{
    setWindowTitle(tr("LiDAR Point Cloud Viewer"));
    setMinimumSize(800, 600);
    
    createMenus();
    createToolBars();
    createStatusBar();
    createDockWidgets();
    
    m_openGLWidget = new OpenGLWidget(this);
    m_openGLWidget->setPointCloud(m_pointCloud);
    m_openGLWidget->setOctree(m_octree);
    m_openGLWidget->setColorMapper(m_colorMapper);
    
    setCentralWidget(m_openGLWidget);
    
    updateStatusBar();
}

PointCloudViewer::~PointCloudViewer()
{
}

void PointCloudViewer::createMenus()
{
    m_fileMenu = menuBar()->addMenu(tr("&File"));
    
    m_openAction = new QAction(tr("&Open..."), this);
    m_openAction->setShortcut(QKeySequence::Open);
    m_openAction->setStatusTip(tr("Open a point cloud file"));
    connect(m_openAction, &QAction::triggered, this, &PointCloudViewer::openFile);
    
    m_saveAction = new QAction(tr("&Save..."), this);
    m_saveAction->setShortcut(QKeySequence::Save);
    m_saveAction->setStatusTip(tr("Save point cloud file"));
    m_saveAction->setEnabled(false);
    connect(m_saveAction, &QAction::triggered, this, &PointCloudViewer::saveFile);
    
    m_exitAction = new QAction(tr("E&xit"), this);
    m_exitAction->setShortcut(QKeySequence::Quit);
    m_exitAction->setStatusTip(tr("Exit the application"));
    connect(m_exitAction, &QAction::triggered, this, &PointCloudViewer::exitApp);
    
    m_fileMenu->addAction(m_openAction);
    m_fileMenu->addAction(m_saveAction);
    m_fileMenu->addSeparator();
    m_fileMenu->addAction(m_exitAction);
    
    m_viewMenu = menuBar()->addMenu(tr("&View"));
    
    m_resetViewAction = new QAction(tr("&Reset View"), this);
    m_resetViewAction->setShortcut(tr("Ctrl+R"));
    m_resetViewAction->setStatusTip(tr("Reset camera to default view"));
    connect(m_resetViewAction, &QAction::triggered, this, &PointCloudViewer::resetView);
    
    m_viewMenu->addAction(m_resetViewAction);
    
    m_helpMenu = menuBar()->addMenu(tr("&Help"));
    
    m_aboutAction = new QAction(tr("&About"), this);
    m_aboutAction->setStatusTip(tr("Show the application's About box"));
    connect(m_aboutAction, &QAction::triggered, this, &PointCloudViewer::about);
    
    m_helpMenu->addAction(m_aboutAction);
}

void PointCloudViewer::createToolBars()
{
    m_fileToolBar = addToolBar(tr("File"));
    m_fileToolBar->addAction(m_openAction);
    m_fileToolBar->addAction(m_saveAction);
    
    m_viewToolBar = addToolBar(tr("View"));
    m_viewToolBar->addAction(m_resetViewAction);
}

void PointCloudViewer::createStatusBar()
{
    m_statusLabel = new QLabel(tr("Ready"));
    m_pointsLabel = new QLabel(tr("Points: 0"));
    
    statusBar()->addWidget(m_statusLabel);
    statusBar()->addPermanentWidget(m_pointsLabel);
}

void PointCloudViewer::createDockWidgets()
{
    m_controlDock = new QDockWidget(tr("Controls"), this);
    m_controlDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    
    m_controlWidget = new QWidget();
    QVBoxLayout *mainLayout = new QVBoxLayout(m_controlWidget);
    
    QGroupBox *colorGroup = new QGroupBox(tr("Color Mapping"));
    QVBoxLayout *colorLayout = new QVBoxLayout(colorGroup);
    
    m_colorMapCombo = new QComboBox();
    m_colorMapCombo->addItem(tr("By Height (Rainbow)"), static_cast<int>(ColorMapType::Height));
    m_colorMapCombo->addItem(tr("By Intensity (Grayscale)"), static_cast<int>(ColorMapType::Intensity));
    m_colorMapCombo->addItem(tr("Original Colors"), static_cast<int>(ColorMapType::Original));
    m_colorMapCombo->addItem(tr("Rainbow"), static_cast<int>(ColorMapType::Rainbow));
    m_colorMapCombo->addItem(tr("Heat Map"), static_cast<int>(ColorMapType::Heat));
    m_colorMapCombo->addItem(tr("Viridis"), static_cast<int>(ColorMapType::Viridis));
    connect(m_colorMapCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &PointCloudViewer::colorMapChanged);
    
    colorLayout->addWidget(new QLabel(tr("Color Map:")));
    colorLayout->addWidget(m_colorMapCombo);
    mainLayout->addWidget(colorGroup);
    
    QGroupBox *pointGroup = new QGroupBox(tr("Point Properties"));
    QVBoxLayout *pointLayout = new QVBoxLayout(pointGroup);
    
    QHBoxLayout *pointSizeLayout = new QHBoxLayout();
    m_pointSizeLabel = new QLabel(tr("Point Size: 2"));
    m_pointSizeSlider = new QSlider(Qt::Horizontal);
    m_pointSizeSlider->setRange(1, 10);
    m_pointSizeSlider->setValue(2);
    connect(m_pointSizeSlider, &QSlider::valueChanged,
            this, &PointCloudViewer::pointSizeChanged);
    
    pointSizeLayout->addWidget(m_pointSizeLabel);
    pointSizeLayout->addWidget(m_pointSizeSlider);
    pointLayout->addLayout(pointSizeLayout);
    mainLayout->addWidget(pointGroup);
    
    QGroupBox *octreeGroup = new QGroupBox(tr("Octree Optimization"));
    QVBoxLayout *octreeLayout = new QVBoxLayout(octreeGroup);
    
    m_useOctreeCheck = new QCheckBox(tr("Use Octree Culling"));
    m_useOctreeCheck->setChecked(false);
    connect(m_useOctreeCheck, &QCheckBox::toggled,
            this, &PointCloudViewer::useOctreeToggled);
    octreeLayout->addWidget(m_useOctreeCheck);
    
    QHBoxLayout *maxPointsLayout = new QHBoxLayout();
    maxPointsLayout->addWidget(new QLabel(tr("Max Points/Node:")));
    m_octreeMaxPointsSpin = new QSpinBox();
    m_octreeMaxPointsSpin->setRange(10, 10000);
    m_octreeMaxPointsSpin->setValue(100);
    connect(m_octreeMaxPointsSpin, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &PointCloudViewer::octreeMaxPointsChanged);
    maxPointsLayout->addWidget(m_octreeMaxPointsSpin);
    octreeLayout->addLayout(maxPointsLayout);
    
    QHBoxLayout *maxDepthLayout = new QHBoxLayout();
    maxDepthLayout->addWidget(new QLabel(tr("Max Depth:")));
    m_octreeMaxDepthSpin = new QSpinBox();
    m_octreeMaxDepthSpin->setRange(1, 20);
    m_octreeMaxDepthSpin->setValue(10);
    connect(m_octreeMaxDepthSpin, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &PointCloudViewer::octreeMaxDepthChanged);
    maxDepthLayout->addWidget(m_octreeMaxDepthSpin);
    octreeLayout->addLayout(maxDepthLayout);
    
    QHBoxLayout *viewDistanceLayout = new QHBoxLayout();
    viewDistanceLayout->addWidget(new QLabel(tr("View Distance:")));
    m_viewDistanceSpin = new QDoubleSpinBox();
    m_viewDistanceSpin->setRange(1.0, 10000.0);
    m_viewDistanceSpin->setValue(1000.0);
    m_viewDistanceSpin->setDecimals(1);
    connect(m_viewDistanceSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &PointCloudViewer::viewDistanceChanged);
    viewDistanceLayout->addWidget(m_viewDistanceSpin);
    octreeLayout->addLayout(viewDistanceLayout);
    
    m_rebuildOctreeButton = new QPushButton(tr("Rebuild Octree"));
    connect(m_rebuildOctreeButton, &QPushButton::clicked,
            this, &PointCloudViewer::rebuildOctree);
    octreeLayout->addWidget(m_rebuildOctreeButton);
    
    mainLayout->addWidget(octreeGroup);
    mainLayout->addStretch();
    
    m_controlDock->setWidget(m_controlWidget);
    addDockWidget(Qt::RightDockWidgetArea, m_controlDock);
}

void PointCloudViewer::openFile()
{
    QString fileName = QFileDialog::getOpenFileName(this,
        tr("Open Point Cloud File"),
        QString(),
        tr("Point Cloud Files (*.pcd *.las);;PCD Files (*.pcd);;LAS Files (*.las);;All Files (*)"));
    
    if (fileName.isEmpty())
    {
        return;
    }
    
    loadFile(fileName);
}

void PointCloudViewer::saveFile()
{
    QMessageBox::information(this, tr("Save"), tr("Save functionality not yet implemented."));
}

void PointCloudViewer::exitApp()
{
    close();
}

void PointCloudViewer::about()
{
    QMessageBox::about(this, tr("About LiDAR Point Cloud Viewer"),
        tr("<h2>LiDAR Point Cloud Viewer</h2>"
           "<p>A simple point cloud viewer based on Qt 6 and OpenGL.</p>"
           "<p><b>Features:</b></p>"
           "<ul>"
           "<li>Read PCD and LAS format files</li>"
           "<li>3D navigation with mouse and keyboard</li>"
           "<li>Octree-based level-of-detail rendering</li>"
           "<li>Pseudo-color mapping by height or intensity</li>"
           "</ul>"
           "<p><b>Controls:</b></p>"
           "<ul>"
           "<li>Left mouse drag: Rotate view</li>"
           "<li>Middle mouse drag or Left+Right drag: Pan</li>"
           "<li>Mouse wheel: Zoom</li>"
           "<li>WASD keys: Move camera</li>"
           "<li>Q/E keys: Move up/down</li>"
           "<li>R key: Reset view</li>"
           "</ul>"));
}

void PointCloudViewer::loadFile(const QString& filename)
{
    m_statusLabel->setText(tr("Loading: %1").arg(filename));
    QApplication::processEvents();
    
    bool success = false;
    
    if (filename.toLower().endsWith(".pcd"))
    {
        success = loadPCDFile(filename);
    }
    else if (filename.toLower().endsWith(".las"))
    {
        success = loadLASFile(filename);
    }
    else
    {
        QMessageBox::warning(this, tr("Warning"),
            tr("Unknown file format. Trying to load as PCD..."));
        success = loadPCDFile(filename);
        if (!success)
        {
            success = loadLASFile(filename);
        }
    }
    
    if (success)
    {
        m_pointCloud->computeBounds();
        
        if (m_colorMapper)
        {
            m_colorMapper->setHeightRange(m_pointCloud->getMinHeight(),
                                           m_pointCloud->getMaxHeight());
            m_colorMapper->setIntensityRange(m_pointCloud->getMinIntensity(),
                                              m_pointCloud->getMaxIntensity());
        }
        
        if (m_useOctreeCheck->isChecked())
        {
            rebuildOctree();
        }
        
        if (m_openGLWidget)
        {
            m_openGLWidget->resetCamera();
        }
        
        m_statusLabel->setText(tr("Loaded: %1").arg(filename));
        updateStatusBar();
        
        setWindowTitle(tr("%1 - LiDAR Point Cloud Viewer").arg(filename));
    }
    else
    {
        m_statusLabel->setText(tr("Failed to load: %1").arg(filename));
        QMessageBox::critical(this, tr("Error"),
            tr("Failed to load file:\n%1").arg(filename));
    }
}

bool PointCloudViewer::loadPCDFile(const QString& filename)
{
    PCDReader reader;
    return reader.read(filename.toStdString(), *m_pointCloud);
}

bool PointCloudViewer::loadLASFile(const QString& filename)
{
    LASReader reader;
    return reader.read(filename.toStdString(), *m_pointCloud);
}

void PointCloudViewer::updateStatusBar()
{
    if (m_pointCloud)
    {
        m_pointsLabel->setText(tr("Points: %1").arg(m_pointCloud->size()));
    }
    else
    {
        m_pointsLabel->setText(tr("Points: 0"));
    }
}

void PointCloudViewer::colorMapChanged(int index)
{
    if (m_colorMapper && m_colorMapCombo)
    {
        bool ok;
        int value = m_colorMapCombo->itemData(index).toInt(&ok);
        if (ok)
        {
            m_colorMapper->setColorMapType(static_cast<ColorMapType>(value));
            if (m_openGLWidget)
            {
                m_openGLWidget->update();
            }
        }
    }
}

void PointCloudViewer::pointSizeChanged(int value)
{
    if (m_pointSizeLabel)
    {
        m_pointSizeLabel->setText(tr("Point Size: %1").arg(value));
    }
    if (m_openGLWidget)
    {
        m_openGLWidget->setPointSize(static_cast<float>(value));
    }
}

void PointCloudViewer::useOctreeToggled(bool checked)
{
    if (m_openGLWidget)
    {
        m_openGLWidget->setUseOctree(checked);
    }
    
    if (checked && m_octree->isEmpty() && m_pointCloud && !m_pointCloud->isEmpty())
    {
        rebuildOctree();
    }
}

void PointCloudViewer::octreeMaxPointsChanged(int value)
{
    if (m_octree)
    {
        m_octree->setMaxPointsPerNode(static_cast<size_t>(value));
    }
}

void PointCloudViewer::octreeMaxDepthChanged(int value)
{
    if (m_octree)
    {
        m_octree->setMaxDepth(static_cast<size_t>(value));
    }
}

void PointCloudViewer::viewDistanceChanged(double value)
{
    if (m_openGLWidget)
    {
        m_openGLWidget->setViewDistance(static_cast<float>(value));
    }
}

void PointCloudViewer::rebuildOctree()
{
    if (!m_pointCloud || m_pointCloud->isEmpty())
    {
        QMessageBox::information(this, tr("Information"),
            tr("No point cloud data to build octree from."));
        return;
    }
    
    m_statusLabel->setText(tr("Building octree..."));
    QApplication::processEvents();
    
    if (m_octree)
    {
        m_octree->clear();
        m_octree->build(*m_pointCloud);
    }
    
    if (m_openGLWidget)
    {
        m_openGLWidget->update();
    }
    
    m_statusLabel->setText(tr("Octree built successfully"));
}

void PointCloudViewer::resetView()
{
    if (m_openGLWidget)
    {
        m_openGLWidget->resetCamera();
    }
}
