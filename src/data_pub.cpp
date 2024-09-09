#include <ros/ros.h> 
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <string.h>
#include <sstream>
#include <fstream>

using namespace std;
using namespace cv;

vector<sensor_msgs::Imu> imu_data_;
vector<int64_t> img_data_;

void LoadImg(ifstream & fImg)
{

    while(!fImg.eof())
    {
        string s;
        getline(fImg,s);
        if(!s.empty())
        {
            char c = s.at(0);
            if(c<'0' || c>'9')      //remove first line in data.csv
                continue;
            stringstream ss;
            ss << s;
            int64_t tmpd;
            int cnt=0;
            int64_t data[1];
            while(ss >> tmpd)
            {
                data[cnt] = tmpd;
                cnt++;
                if(cnt ==1)
                    break;
                if(ss.peek() == ',' || ss.peek() == ' ')
                    ss.ignore();
            }
            img_data_.push_back(data[0]);
        }
    }
    fImg.close();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_pub");     
    ros::NodeHandle nh_;  

    ros::Publisher img_pub_ = nh_.advertise<sensor_msgs::Image>("/cam0/image_raw", 10);

    //Read CSV file
    string data_folder_path_ = "/media/junlin/New Volume/Dataset/38/urban38";

    string Img_file = data_folder_path_ + "/sensor_data/stereo_stamp.csv";
    ifstream fImg;
    fImg.open(Img_file);
    LoadImg(fImg);

    int64_t img_cnt = 0;
    ros::Rate loop_rate(10);
    while((ros::ok()) && (img_cnt < img_data_.size()))
    {
        printf("img %d time %f\n", img_cnt, img_data_[img_cnt]*1e-9);

        string next_img_name = data_folder_path_ + "/image/stereo_left" + "/" + to_string(img_data_[img_cnt]) + ".png";
        cv::Mat next_image;
        next_image = imread(next_img_name, CV_LOAD_IMAGE_ANYDEPTH);

        if (!next_image.empty())
        {
            cv_bridge::CvImage img_out_msg;
            img_out_msg.header.stamp.fromNSec(img_data_[img_cnt]);
            img_out_msg.header.frame_id = "img";
            img_out_msg.encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
            img_out_msg.image    = next_image;

            imshow("img", next_image);
            waitKey(5);

            img_pub_.publish(img_out_msg.toImageMsg());
        }

        ++img_cnt;
        // if (img_cnt == 4000)
        // {
        //     break;
        // }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
