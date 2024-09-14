#include <ros/ros.h> 
#include <rosbag/view.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <string.h>
#include <sstream>
#include <fstream>

using namespace std;
using namespace cv;

std::map<double, std::string> calib_imgs;
vector<double> vec_img_time;
vector<double> vec_gt_time;

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

    std::string data_dir = "/media/junlin/New Volume/Dataset/calibration/9-10/";

    rosbag::Bag bag_data;
    bag_data.open(data_dir + "SPAD_calib.bag", rosbag::bagmode::Write);

    string gt_file = data_dir + "SPAD_gt_data.txt";
    ifstream fGT;
    fGT.open(gt_file);
    while(!fGT.eof())
    {
        string s;
        getline(fGT,s);
        if(!s.empty())
        {
            char c = s.at(0);
            if(c<'0' || c>'9')      //remove first line in data.csv
                continue;
            stringstream ss;
            ss << s;
            double tmpd;
            int cnt=0;
            double data[8];
            while(ss >> tmpd)
            {
                data[cnt] = tmpd;
                cnt++;
                if(cnt ==8)
                    break;
                if(ss.peek() == ',' || ss.peek() == ' ')
                    ss.ignore();
            }

            // for (int i = 0; i < 8; i++)
            // {
            //     printf("%.3f ", data[i]);
            // }
            // printf("\n");
            // getchar();

            vec_gt_time.push_back(data[0]);

            geometry_msgs::TransformStamped gt_pose;
            gt_pose.header.stamp.fromSec(data[0]);
            gt_pose.transform.translation.x = data[1];
            gt_pose.transform.translation.y = data[2];
            gt_pose.transform.translation.z = data[3];
            gt_pose.transform.rotation.x = data[4];
            gt_pose.transform.rotation.y = data[5];
            gt_pose.transform.rotation.z = data[6];
            gt_pose.transform.rotation.w = data[7];

            bag_data.write("/vrpn_client/raw_transform", gt_pose.header.stamp, gt_pose);
        }
    }
    fGT.close();

    auto gt_minmax = minmax_element(vec_gt_time.begin(), vec_gt_time.end());
    std::cout << std::fixed << std::setprecision(3);
    cout << "gt_min = " << *gt_minmax.first << ", gt_max = " << *gt_minmax.second << '\n';

    std::string delimiter = "_";
    
    cv::String path(data_dir + "SPAD/*.bin"); //select only jpg
    vector<cv::String> fn;
    cv::glob(path,fn,true); // recurse
    for (size_t k=0; k<fn.size(); ++k)
    {
        // std::cout << fn[k] << std::endl;
        std::vector<std::string> v = split(fn[k], delimiter);

        string str_time = v[2];
        double img_time = std::stod(str_time);
        // printf("img_time %.2f\n", img_time);
        vec_img_time.push_back(img_time);
    }

    sort(vec_img_time.begin(), vec_img_time.end());

    for (int i = 3; i < vec_img_time.size(); i++)
    {
        string img_path = data_dir + "SPAD/8bit/img" + std::to_string(i - 2) + ".PNG";
        // std::cout << img_path << std::endl;
        cv::Mat im = cv::imread(img_path, CV_LOAD_IMAGE_ANYDEPTH);
        if (im.empty()) continue; //only proceed if sucsessful

        calib_imgs[vec_img_time[i]] = img_path;
    }
    
    vec_img_time.clear();

    std::map<double, std::string>::iterator it(calib_imgs.begin()), end(calib_imgs.end());
    while(it != end) {
        // std::cout << std::fixed << std::setprecision(2) << '"' << it->first << '"' << "     " << it->second << "\n";

        if (it->first < *gt_minmax.first) continue;
        if (it->first > *gt_minmax.second) continue;
        
        cv::Mat next_image;
        next_image = imread(it->second, CV_LOAD_IMAGE_ANYDEPTH);
        // imshow("img", next_image);
        // waitKey(0);

        vec_img_time.push_back(it->first);

        cv_bridge::CvImage img_out_msg;
        img_out_msg.header.stamp.fromSec(it->first);
        img_out_msg.header.frame_id = "img";
        img_out_msg.encoding = sensor_msgs::image_encodings::MONO8;
        img_out_msg.image    = next_image;

        bag_data.write("/cam0/image_raw", img_out_msg.header.stamp, img_out_msg);
        
        ++it;
    }

    // Find min, max pair
    auto img_minmax = minmax_element(vec_img_time.begin(), vec_img_time.end());
    cout << "img_min = " << *img_minmax.first << ", img_max = " << *img_minmax.second << '\n';
    cout << "gt_min = " << *gt_minmax.first << ", gt_max = " << *gt_minmax.second << '\n';

    bag_data.close();
    printf("finish!\n");

    return 0;
}