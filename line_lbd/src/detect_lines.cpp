/*
 * line_detection interface
 * Copyright Shichao Yang,2018, Carnegie Mellon University
 * Email: shichaoy@andrew.cmu.edu
 *
 */

#include <line_lbd/line_descriptor.hpp>

// #include "opencv2/core/utility.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <fstream>
#include <ctime>
#include <line_lbd/line_lbd_allclass.h>

#include <ros/ros.h>

using namespace cv;
using namespace std;


int main( int argc, char** argv )
{
    // 直线检测大约耗时896ms
    /* get parameters from comand line */

    ros::init(argc, argv, "detect_lines");
    ros::NodeHandle nh; 
    
    std::string image_path, save_folder, image_format;
    bool use_LSD_algorithm;
    bool save_to_imgs;
    bool save_to_txts;
    int image_num;
    nh.param<std::string>("image_path", image_path, "$(find line_lbd)/data/example/");
    nh.param<std::string>("save_folder", save_folder, "$(find line_lbd)/data/example/");
    nh.param<bool>("use_LSD_algorithm", use_LSD_algorithm, true);
    nh.param<bool>("save_to_imgs", save_to_imgs, false);
    nh.param<bool>("save_to_txts", save_to_txts, false);
    nh.param<int>("image_num", image_num, 1);
    nh.param<std::string>("image_format", image_format, "png");
    
    int numOfOctave_ = 1;
    float Octave_ratio = 2.0;  
    
    line_lbd_detect* line_lbd_ptr = new line_lbd_detect(numOfOctave_,Octave_ratio); 
    line_lbd_ptr->use_LSD = use_LSD_algorithm;
    line_lbd_ptr->line_length_thres = 15;  // remove short edges
    
    // using my line detector class, could select LSD or edline.
    
    for (int frame_index = 0; frame_index < image_num; frame_index++)
    {
        char frame_index_c[256];
        sprintf(frame_index_c, "%06d", frame_index); // format into 4 digit

        cv::Mat raw_img = imread(image_path + frame_index_c + "." + image_format, 1);
        if( raw_img.data == NULL )
        {
            std::cout << "Error, image could not be loaded. Please, check its path \n" << image_path << std::endl;
            return -1;
        }

        std::vector<KeyLine> keylines_raw, keylines_out;
        clock_t time0 = clock();
        line_lbd_ptr->detect_raw_lines(raw_img, keylines_raw);
        line_lbd_ptr->filter_lines(keylines_raw, keylines_out);  // remove short lines

        clock_t time1 = clock();
        std::cout << time1 << "\n\n";
        std::cout << "line detection's time consumption : " << 1000 * (time1 - time0) / (double)CLOCKS_PER_SEC << "ms\n\n";


        
        if (save_to_imgs)
        {
            // show image
            if( raw_img.channels() == 1 )
            cvtColor(raw_img, raw_img, COLOR_GRAY2BGR);
            cv::Mat raw_img_cp;
            drawKeylines(raw_img, keylines_out, raw_img_cp, cv::Scalar( 0, 150, 0 ),2); // B G R
            imshow( "Line detector", raw_img_cp );
            waitKey();
            std::string img_save_name = save_folder + "saved_edges.jpg";
            cv::imwrite(img_save_name, raw_img_cp);
        }
        
        if (save_to_txts)
        {
            std::string txt_save_name = save_folder + frame_index_c + ".txt";
            ofstream resultsFile;
            resultsFile.open(txt_save_name);
            for (int j = 0; j < keylines_out.size(); j++)
            {
                resultsFile << keylines_out[j].startPointX << "\t" << keylines_out[j].startPointY  << "\t"
                            << keylines_out[j].endPointX   << "\t" << keylines_out[j].endPointY << endl;
            }
            resultsFile.close();
        }
    }
}
