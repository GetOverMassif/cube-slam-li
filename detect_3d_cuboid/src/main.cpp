/*
 * @Author: GetOverMassif 164567487@qq.com
 * @Date: 2022-10-15 21:46:17
 * @LastEditors: GetOverMassif 164567487@qq.com
 * @LastEditTime: 2022-10-18 23:44:29
 * @FilePath: /cubeslam_ws2/src/cube_slam/detect_3d_cuboid/src/main.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
// std c
#include <stdio.h>
#include <iostream>
#include <string>

// opencv
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

// ros
#include <ros/ros.h>
#include <ros/package.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>

// ours
#include "detect_3d_cuboid/detect_3d_cuboid.h"
#include "tictoc_profiler/profiler.hpp"

using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detect_3d_cuboid");
    ros::NodeHandle nh;
    ca::Profiler::enable();
    
    // 基础数据文件夹
    std::string base_folder = ros::package::getPath("detect_3d_cuboid") + "/data/example/";
    std::string image_format;
    double x, y, w, h, prob;
    int frame_index = 0;
    bool use_lines = false;
    nh.param<std::string>("base_folder", base_folder, base_folder);
    nh.param<std::string>("image_format", image_format, "jpg");
    nh.param<int>("frame_index", frame_index, 0);
    nh.param<bool>("use_lines", use_lines, false);
    nh.param<double>("x", x, 188);
    nh.param<double>("y", y, 189);
    nh.param<double>("w", w, 201);
    nh.param<double>("h", h, 311);
    nh.param<double>("prob", prob, 0.8800);

    Matrix3d Kalib;

    // cout << "Matrix3d Kalib = " << Kalib.matrix() << endl;

    Matrix4d transToWorld;

    if (!read_Kalib_and_transToWorld(base_folder + "KT.txt", Kalib, transToWorld)) {
        return 0;
    }
    
    // 物体坐标(列x，行y，宽度w，高度h，概率prob)
    MatrixXd obj_bbox_coors(1, 5);                // hard coded
    obj_bbox_coors << x, y, w, h, prob;
    obj_bbox_coors.leftCols<2>().array() -= 1;    // change matlab coordinate to c++, minus 1

    char frame_index_c[256];
    sprintf(frame_index_c, "%04d", frame_index); // format into 4 digit

    // 读取图像
    // cv::Mat rgb_img = cv::imread(base_folder + frame_index_c + "_rgb_raw.jpg", 1);

    cv::Mat rgb_img = cv::imread(base_folder + frame_index_c + "_rgb_raw." + image_format, 1);
    // cv::Mat gray_img;
    // cv::cvtColor(rgb_img, gray_img, CV_BGR2GRAY);
    printf("img size = (%d, %d)\n", rgb_img.rows, rgb_img.cols);

    // 读取边
    // Eigen::MatrixXd all_lines_raw(100, 4); // 100 is some large frame number,   the txt edge index start from 0
    Eigen::MatrixXd all_lines_raw(100, 4); // 100 is some large frame number,   the txt edge index start from 0
    if (use_lines) {
        std::cout << "use lines" << std::endl;
        read_all_number_txt(base_folder + "edge_detection/LSD/" + frame_index_c + "_edges.txt", all_lines_raw);
    }
    
    // ****************创建 detect_3d_cuboid 对象*******************
    detect_3d_cuboid detect_cuboid_obj;
    // 设置是否绘制图像细节、最终图像，输出细节
    nh.param<bool>("whether_plot_detail_images", detect_cuboid_obj.whether_plot_detail_images, false);
    nh.param<bool>("whether_plot_final_images", detect_cuboid_obj.whether_plot_final_images, true);
    nh.param<bool>("print_details", detect_cuboid_obj.print_details, false);
    // 设置内参
    detect_cuboid_obj.set_calibration(Kalib);
    // 设置是否采样包围盒高度、相机滚转、俯仰角
    nh.param<bool>("whether_sample_bbox_height", detect_cuboid_obj.whether_sample_bbox_height, false);
    nh.param<bool>("whether_sample_cam_roll_pitch", detect_cuboid_obj.whether_sample_cam_roll_pitch, false);

    std::vector<ObjectSet> all_object_cuboids;
    // 检测长方体(RGB图像, 相机在世界坐标系的坐标, 所有边缘线, 存储长方体的vector)

    // double begin = ros::Time::now().toSec();
    // cout << "begin" << endl;
    // std::cout << "rgb_img.channels() = " << gray_img.channels() << endl;
    detect_cuboid_obj.detect_cuboid(rgb_img, transToWorld, obj_bbox_coors, all_lines_raw, all_object_cuboids);
    // ros::Duration(0.5).sleep();
    // double end = ros::Time::now().toSec();
    // printf("begin = %f,\nend = %f,\nduration = %f", begin, end, end - begin);
    
    // ca::Profiler::print_aggregated(std::cout);
    return 0;
}