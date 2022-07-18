#include "PCL_App.h"

//  VTK
#if VTK_MAJOR_VERSION > 8
#include <vtkGenericOpenGLRenderWindow.h>
#endif
//error fix: static_cast: 无法从vtkObjectBase *const转换为“T”
#include <vtkRenderWindow.h>

PCL_App::PCL_App(QWidget *parent)
    : QMainWindow(parent),
    filtering_axis_(1),  // = y
    color_mode_(4),  // = Rainbow
    ui(new Ui::PCL_AppClass)
{
    ui->setupUi(this);
    this->setWindowTitle("PCL viewer");

    red = 128;
    green = 128;
    blue = 128;

    // Cloud Init
    cloud_.reset(new PointCloudXYZ);// Setup the cloud pointer
    cloud_->resize(500);// The number of points in the cloud
    // Fill the cloud with random points
    for (std::size_t i = 0; i < cloud_->size(); ++i) {
        (*cloud_)[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        (*cloud_)[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        (*cloud_)[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }
    cloud_rgb.reset(new PointCloudXYZRGB);
    pcl::copyPointCloud(*cloud_, *cloud_rgb);

    // Set up the QVTK window
#if VTK_MAJOR_VERSION > 8
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    viewer_.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "viewer", false));
    ui->qvtkWidget->setRenderWindow(viewer_->getRenderWindow());
    viewer_->setupInteractor(ui->qvtkWidget->interactor(), ui->qvtkWidget->renderWindow());
#else
    viewer_.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    ui->qvtkWidget->SetRenderWindow(viewer_->getRenderWindow());
    viewer_->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
#endif

    //int red = 255;
    //int green = 0;
    //int blue = 255;
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_, red, green, blue);//自定义点云颜色
    //viewer_->addPointCloud(cloud_, single_color, "cloud");
    //viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud");//设置点云单个点的大小

    // Connect functions
    // menu
    connect(ui->actionLoad, SIGNAL(triggered()), this, SLOT(loadFileButtonPressed()));
    connect(ui->actionSimple, SIGNAL(triggered()), this, SLOT(colorSimpleBtnPressed()));
    connect(ui->actionPlattete, SIGNAL(triggered()), this, SLOT(colorpaletteBtnPressed()));


    // Color the randomly generated cloud
    colorCloudDistances();
    viewer_->setBackgroundColor(0.1, 0.1, 0.1);
    //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_rgb);
    //viewer_->addPointCloud<pcl::PointXYZRGB>(cloud_rgb, rgb, "cloud");
    viewer_->addPointCloud(cloud_rgb, "cloud");
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
    viewer_->resetCamera();
    refreshView();
}

PCL_App::~PCL_App()
{
    delete ui;
}

void PCL_App::refreshView()
{
#if VTK_MAJOR_VERSION > 8
    ui->qvtkWidget->renderWindow()->Render();
#else
    ui->qvtkWidget->update();
#endif
}

// menu 
void PCL_App::loadFileButtonPressed() {
    QString curPath = QCoreApplication::applicationDirPath(); //获取应用程序的路径
    //调用打开文件对话框打开一个文件
    QString filename = QFileDialog::getOpenFileName(this, tr("Open point cloud"), curPath, tr("Point cloud data (*.pcd *.ply)"));
    PCL_INFO("File chosen: %s\n", filename.toStdString().c_str());
    if (filename.isEmpty())
        return;

    PointCloudXYZ::Ptr cloud_tmp(new PointCloudXYZ);
    int return_status;
    if (filename.endsWith(".pcd", Qt::CaseInsensitive))
        return_status = pcl::io::loadPCDFile(filename.toStdString(), *cloud_tmp);
    else
        return_status = pcl::io::loadPLYFile(filename.toStdString(), *cloud_tmp);
    if (return_status != 0) {
        PCL_ERROR("Error reading point cloud %s\n", filename.toStdString().c_str());
        return;
    }

    // If point cloud contains NaN values, remove them before updating the visualizer point cloud
    if (cloud_tmp->is_dense)
        pcl::copyPointCloud(*cloud_tmp, *cloud_);
    else {
        PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
        std::vector<int> vec;
        pcl::removeNaNFromPointCloud(*cloud_tmp, *cloud_, vec);
    }

    pcl::copyPointCloud(*cloud_, *cloud_rgb);
    colorCloudDistances();
    viewer_->updatePointCloud(cloud_rgb, "cloud");
    viewer_->resetCamera();

    refreshView();
}
void PCL_App::colorSimpleBtnPressed() {
    //模态对话框，动态创建，用后删除
    dlg_color_simple* dlg = new dlg_color_simple(this);
    dlg->setWindowTitle("color");
    dlg->initUi(red, green, blue);
    Qt::WindowFlags flags = dlg->windowFlags();
    dlg->setWindowFlags(flags | Qt::MSWindowsFixedSizeDialogHint);
    int ret = dlg->exec();//以模态方式显示对话框
    if (ret == QDialog::Accepted) {
        //OK按钮被按下，获取对话框上的输入
        std::cout << "dlg_color_simple accept" << std::endl;
        red = dlg->r();
        green = dlg->g();
        blue = dlg->b();
        singleColorChosen();
        delete dlg;
    }
}
void PCL_App::colorpaletteBtnPressed() {
    dlg_color_palette* dlg = new dlg_color_palette(this);
    dlg->setWindowTitle("color");
    dlg->initUi(filtering_axis_, color_mode_);
    Qt::WindowFlags flags = dlg->windowFlags();
    dlg->setWindowFlags(flags | Qt::MSWindowsFixedSizeDialogHint);
    int ret = dlg->exec();//以模态方式显示对话框
    if (ret == QDialog::Accepted) {
        //OK按钮被按下，获取对话框上的输入
        std::cout << "dlg_color_palette accept" << std::endl;
        filtering_axis_ = dlg->axis();
        color_mode_ = dlg->mode();
        paletteColorChosen();
        delete dlg;
    }
}

void PCL_App::singleColorChosen() {
    std::cout << "r:" << red << "g:" << green << std::endl;

    // Set the new color
    for (auto& point : *cloud_rgb) {
        point.r = red;
        point.g = green;
        point.b = blue;
    }
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_rgb);
    viewer_->updatePointCloud<pcl::PointXYZRGB>(cloud_rgb, rgb, "cloud");
    //viewer_->updatePointCloud(cloud_rgb, "cloud");
    viewer_->resetCamera();
    refreshView();
}
void PCL_App::paletteColorChosen() {
    switch (filtering_axis_) {
    case 0:  // x
        PCL_INFO("x filtering chosen\n");
        break;
    case 1:  // y
        PCL_INFO("y filtering chosen\n");
        break;
    default:  // z
        PCL_INFO("z filtering chosen\n");
        break;
    }
    switch (color_mode_) {
    case 0:
        PCL_INFO("Blue -> Red LUT chosen\n");
        break;
    case 1:
        PCL_INFO("Green -> Magenta LUT chosen\n");
        break;
    case 2:
        PCL_INFO("White -> Red LUT chosen\n");
        break;
    case 3:
        PCL_INFO("Grey / Red LUT chosen\n");
        break;
    default:
        PCL_INFO("Rainbow LUT chosen\n");
    }
    colorCloudDistances();
    viewer_->updatePointCloud(cloud_rgb, "cloud");
    refreshView();
}

void PCL_App::colorCloudDistances() {
    // Find the minimum and maximum values along the selected axis
    double min, max;
    // Set an initial value
    switch (filtering_axis_) {
    case 0:  // x
        min = (*cloud_rgb)[0].x;
        max = (*cloud_rgb)[0].x;
        break;
    case 1:  // y
        min = (*cloud_rgb)[0].y;
        max = (*cloud_rgb)[0].y;
        break;
    default:  // z
        min = (*cloud_rgb)[0].z;
        max = (*cloud_rgb)[0].z;
        break;
    }

    // Search for the minimum/maximum
    for (PointCloudXYZRGB::iterator cloud_it = cloud_rgb->begin(); cloud_it != cloud_rgb->end(); ++cloud_it) {
        switch (filtering_axis_) {
        case 0:  // x
            if (min > cloud_it->x) min = cloud_it->x;

            if (max < cloud_it->x) max = cloud_it->x;
            break;
        case 1:  // y
            if (min > cloud_it->y) min = cloud_it->y;

            if (max < cloud_it->y) max = cloud_it->y;
            break;
        default:  // z
            if (min > cloud_it->z) min = cloud_it->z;
            if (max < cloud_it->z) max = cloud_it->z;
            break;
        }
    }

    // Compute LUT scaling to fit the full histogram spectrum
    double lut_scale = 255.0 / (max - min);  // max is 255, min is 0

    if (min == max)  // In case the cloud is flat on the chosen direction (x,y or z)
        lut_scale = 1.0;  // Avoid rounding error in boost

    for (PointCloudXYZRGB::iterator cloud_it = cloud_rgb->begin(); cloud_it != cloud_rgb->end(); ++cloud_it) {
        int value;
        switch (filtering_axis_) {
        case 0:  // x
            value = std::lround((cloud_it->x - min) * lut_scale);  // Round the number to the closest integer
            break;
        case 1:  // y
            value = std::lround((cloud_it->y - min) * lut_scale);
            break;
        default:  // z
            value = std::lround((cloud_it->z - min) * lut_scale);
            break;
        }

        // Apply color to the cloud
        switch (color_mode_) {
        case 0:
            // Blue (= min) -> Red (= max)
            cloud_it->r = value;
            cloud_it->g = 0;
            cloud_it->b = 255 - value;
            break;
        case 1:
            // Green (= min) -> Magenta (= max)
            cloud_it->r = value;
            cloud_it->g = 255 - value;
            cloud_it->b = value;
            break;
        case 2:
            // White (= min) -> Red (= max)
            cloud_it->r = 255;
            cloud_it->g = 255 - value;
            cloud_it->b = 255 - value;
            break;
        case 3:
            // Grey (< 128) / Red (> 128)
            if (value > 128) {
                cloud_it->r = 255;
                cloud_it->g = 0;
                cloud_it->b = 0;
            }
            else {
                cloud_it->r = 128;
                cloud_it->g = 128;
                cloud_it->b = 128;
            }
            break;
        default:
            // Blue -> Green -> Red (~ rainbow)
            cloud_it->r = value > 128 ? (value - 128) * 2 : 0;  // r[128] = 0, r[255] = 255
            cloud_it->g = value < 128 ? 2 * value : 255 - ((value - 128) * 2);  // g[0] = 0, g[128] = 255, g[255] = 0
            cloud_it->b = value < 128 ? 255 - (2 * value) : 0;  // b[0] = 255, b[128] = 0
        }
    }
}
