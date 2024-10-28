#include <ros/ros.h> 
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <string.h>
#include <sstream>
#include <fstream>

using namespace std;

vector<sensor_msgs::Imu> imu_msg_buffer;
vector<double> vec_imu_time;
vector<double> vec_gt_time;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "read_imu_mocap");     
    ros::NodeHandle nh_;  

    std::string data_dir = "/media/junlin/New Volume/Dataset/calibration/9-10/imu_marker/";

    rosbag::Bag bag_data;
    bag_data.open(data_dir + "calib.bag", rosbag::bagmode::Write);

    string imu_file = data_dir + "imu_data.txt";
    ifstream fIMU;
    fIMU.open(imu_file);
    while(!fIMU.eof())
    {
        string s;
        getline(fIMU,s);
        if(!s.empty())
        {
            char c = s.at(0);
            if(c<'0' || c>'9')      //remove first line in data.csv
                continue;
            stringstream ss;
            ss << s;
            double tmpd;
            int cnt=0;
            double data[7];
            while(ss >> tmpd)
            {
                data[cnt] = tmpd;
                cnt++;
                if(cnt ==7)
                    break;
                if(ss.peek() == ',' || ss.peek() == ' ')
                    ss.ignore();
            }

            // for (int i = 0; i < 7; i++)
            // {
            //     printf("%.3f ", data[i]);
            // }
            // printf("\n");
            // getchar();

            vec_imu_time.push_back(data[0]);

            sensor_msgs::Imu imu_data;
            imu_data.header.stamp.fromSec(data[0]);
            imu_data.angular_velocity.x = data[1];
            imu_data.angular_velocity.y = data[2];
            imu_data.angular_velocity.z = data[3];
            imu_data.linear_acceleration.x = data[4];
            imu_data.linear_acceleration.y = data[5];
            imu_data.linear_acceleration.z = data[6];

            bag_data.write("/imu", imu_data.header.stamp, imu_data);
        }
    }
    fIMU.close();

    string gt_file = data_dir + "gt_data.txt";
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

    // Find min, max pair
    std::cout << std::fixed << std::setprecision(3);
    auto imu_minmax = minmax_element(vec_imu_time.begin(), vec_imu_time.end());
    cout << "imu_min = " << *imu_minmax.first << ", imu_max = " << *imu_minmax.second << '\n';
    auto gt_minmax = minmax_element(vec_gt_time.begin(), vec_gt_time.end());
    cout << "gt_min = " << *gt_minmax.first << ", gt_max = " << *gt_minmax.second << '\n';

    bag_data.close();
    printf("finish!\n");

    return 0;
}