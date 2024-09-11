#include <ros/ros.h> 
#include <rosbag/view.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <string.h>
#include <sstream>
#include <fstream>

using namespace std;
using namespace cv;

std::map<double, std::string> calib_imgs;

// for string delimiter
// https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c
std::vector<std::string> split(std::string s, std::string delimiter) {
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    std::string token;
    std::vector<std::string> res;

    while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos) {
        token = s.substr (pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back (token);
    }

    res.push_back (s.substr (pos_start));
    return res;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "read_img_mocap");     
    ros::NodeHandle nh_;  

    std::string delimiter = "_";
    
    cv::String path("/media/junlin/New Volume/Dataset/calibration/FLIR/*.jpg"); //select only jpg
    vector<cv::String> fn;
    cv::glob(path,fn,true); // recurse
    for (size_t k=0; k<fn.size(); ++k)
    {
        // std::cout << fn[k] << std::endl;
        std::vector<std::string> v = split(fn[k], delimiter);

        cv::Mat im = cv::imread(fn[k], CV_LOAD_IMAGE_ANYDEPTH);
        if (im.empty()) continue; //only proceed if sucsessful

        string str_time = v[2];
        double img_time = std::stod(str_time);
        // printf("img_time %.2f\n", img_time);
        calib_imgs[img_time] = fn[k];
    }

    rosbag::Bag bag_data;
    bag_data.open("/media/junlin/New Volume/Dataset/calibration/FLIR_calib.bag", rosbag::bagmode::Write);

    std::map<double, std::string>::iterator it(calib_imgs.begin()), end(calib_imgs.end());
    while(it != end) {
        // std::cout << std::fixed << std::setprecision(2) << '"' << it->first << '"' << "     " << it->second << "\n";
        
        cv::Mat next_image;
        next_image = imread(it->second, CV_LOAD_IMAGE_ANYDEPTH);
        // imshow("img", next_image);
        // waitKey(0);

        cv_bridge::CvImage img_out_msg;
        img_out_msg.header.stamp.fromSec(it->first);
        img_out_msg.header.frame_id = "img";
        img_out_msg.encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
        img_out_msg.image    = next_image;

        bag_data.write("/cam0/image_raw", img_out_msg.header.stamp, img_out_msg);
        
        ++it;
    }

    bag_data.close();
    printf("finish!\n");

    return 0;
}