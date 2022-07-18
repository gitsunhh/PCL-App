#pragma once

// Qt
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QFileDialog>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>

// Boost
#include <boost/math/special_functions/round.hpp>

typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;
typedef pcl::PointXYZRGB PointXYZRGB;
typedef pcl::PointCloud<PointXYZRGB> PointCloudXYZRGB;

#include "ui_PCL_App.h"

#include "dlg_color_simple.h"
#include "dlg_color_palette.h"

namespace Ui
{
    class PCL_AppClass;
}

class PCL_App : public QMainWindow
{
    Q_OBJECT

public:
    /** @brief Constructor */
    explicit PCL_App(QWidget* parent = 0);

    /** @brief Destructor */
    ~PCL_App();

public Q_SLOTS:
    //menu
    //File
    void loadFileButtonPressed();/** @brief Triggered whenever the "Load file" button is clicked */
    //void saveFileButtonPressed();
    //visual
    void colorSimpleBtnPressed();
    void colorpaletteBtnPressed();

    //Func
    /** @brief Triggered whenever a button in the "Color mode" group is clicked */
    void singleColorChosen();
    void paletteColorChosen();

protected:
    /** @brief The point cloud displayed */
    PointCloudXYZ::Ptr cloud_;
    PointCloudXYZRGB::Ptr cloud_rgb;
    /** @brief 0 = x | 1 = y | 2 = z */
    int filtering_axis_;
    /** @brief Holds the color mode for @ref colorCloudDistances */
    int color_mode_;

    unsigned int red;
    unsigned int green;
    unsigned int blue;
    unsigned int pointsize;

    /** @brief Rerender the view */
    void refreshView();

    /** @brief The PCL visualizer object */
    pcl::visualization::PCLVisualizer::Ptr viewer_;

    /** @brief Color point cloud on X,Y or Z axis using a Look-Up Table (LUT)
     * Computes a LUT and color the cloud accordingly, available color palettes are :
     *
     *  Values are on a scale from 0 to 255:
     *  0. Blue (= 0) -> Red (= 255), this is the default value
     *  1. Green (= 0) -> Magenta (= 255)
     *  2. White (= 0) -> Red (= 255)
     *  3. Grey (< 128) / Red (> 128)
     *  4. Blue -> Green -> Red (~ rainbow)
     *
     * @warning If there's an outlier in the data the color may seem uniform because of this outlier!
     * @note A boost rounding exception error will be thrown if used with a non dense point cloud
     */
    void colorCloudDistances();
private:
    /** @brief ui pointer */
    Ui::PCL_AppClass* ui;
};

