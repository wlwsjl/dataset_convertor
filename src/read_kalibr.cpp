#include "read_kalibr.hpp"
#include <fstream>

using namespace std;
using namespace Eigen;

inline Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
{
  Eigen::Vector3d n = R.col(0);
  Eigen::Vector3d o = R.col(1);
  Eigen::Vector3d a = R.col(2);

  Eigen::Vector3d ypr(3);
  double y = atan2(n(1), n(0));
  double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
  double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
  ypr(0) = y;
  ypr(1) = p;
  ypr(2) = r;

  return ypr / M_PI * 180.0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "read_kalibr");
    ros::NodeHandle nh;

    std::string dataset_name;
    nh.param<string>("/read_kalibr/dataset_name", dataset_name, "tum");
    int img_freq;
    nh.param<int>("/read_kalibr/img_freq", img_freq, 0);
    double td;
    nh.param<double>("/read_kalibr/td", td, 0.0);

    std::ofstream outFile_metrics;
    outFile_metrics.open ("/home/junlin/GNSS/eval/VI-Calib/Kalibr/" + dataset_name + "_" + to_string(img_freq) + ".txt", fstream::app);       

    Isometry3d T_cam0_imu = getTransformEigen(nh, "/read_kalibr/cam0/T_cam_imu");
    Eigen::Matrix3d R_cam0_imu = T_cam0_imu.rotation();
    Eigen::Vector3d t_I_in_cam0 = T_cam0_imu.translation();

    Isometry3d T_cam1_imu = getTransformEigen(nh, "/read_kalibr/cam1/T_cam_imu");
    Eigen::Matrix3d R_cam1_imu = T_cam1_imu.rotation();
    Eigen::Vector3d t_I_in_cam1 = T_cam1_imu.translation();

    // evaluation
    Eigen::Matrix<double, 3, 3> R0_gt;
    Eigen::Matrix<double, 3, 3> R1_gt;
    Eigen::Matrix<double, 3, 1> t0_gt;
    Eigen::Matrix<double, 3, 1> t1_gt;

    if (dataset_name == "tum")
    {
      // cam0
      R0_gt << -0.9995437905148815, 0.028917730397527503, -0.008716404751943024,
              0.007712115373400507, -0.03466077556404458, -0.9993693781149028,
              -0.02920161159270393, -0.9989806782445575, 0.0344219461163081;
      t0_gt << 0.04743637049202233, -0.04811386609771469, -0.06745319856676987;

      R1_gt << -0.9995299052133587, 0.029625673533657696, -0.007892277988740584,
              0.008262178290176347, 0.012384222604400115, -0.9998891775793887,
              -0.029524650617336078, -0.9994843422976479, -0.012623173442618733;
      t1_gt << -0.05354600105110982, -0.046371323431226796, -0.07104558663390027;
    }
    else if (dataset_name == "euroc")
    {
      // cam0
      R0_gt << 0.014851386023328694, 0.9995888529409687, -0.024526748853922843,
              -0.9998894702203773, 0.01486397509345927, 0.00033103902021396764,
              0.0006954678985826215, 0.024519121529499192, 0.9996991192372946;
      t0_gt << 0.06588555968137384, -0.017299013104475534, 0.002194765893326177;

      R1_gt << 0.012674792016469094, 0.9995772213064189, -0.02616731344020043,
              -0.9998088356565016, 0.013058678834077575, 0.014552080617529918,
              0.014887638850063773, 0.025977866587671428, 0.9995516538213637;
      t1_gt << -0.04417202627643069, -0.016952151200379708, 0.0020592896281592275;
    }
    else
    {
      printf("dataset_name error!\n");
      getchar();
    }

    Eigen::Matrix<double, 3, 3> diff_R = T_cam0_imu.rotation() * R0_gt.transpose();
    Eigen::Vector3d ypr = R2ypr(diff_R);
    double rotation_err0 = sqrt(ypr.dot(ypr) / 3.0);

    diff_R = T_cam1_imu.rotation() * R1_gt.transpose();
    ypr = R2ypr(diff_R);
    double rotation_err1 = sqrt(ypr.dot(ypr) / 3.0);

    Eigen::Vector3d te = T_cam0_imu.translation() - t0_gt;
    double translation_err0 = sqrt(te.dot(te) / 3.0);
    translation_err0 *= 100.0;

    te = T_cam1_imu.translation() - t1_gt;
    double translation_err1 = sqrt(te.dot(te) / 3.0);
    translation_err1 *= 100.0;

    double td0;
    nh.param<double>("/read_kalibr/cam0/timeshift_cam_imu", td0, 0.0);
    double td1;
    nh.param<double>("/read_kalibr/cam1/timeshift_cam_imu", td1, 0.0);
    double td_avg = 0.5 * (td0 + td1) * 1000.0;
    double time_offset_err = fabs(td_avg - td);

    double reproj_err;
    nh.param<double>("/read_kalibr/reproj_err", reproj_err, 0.0);
    double opt_time_cost;
    nh.param<double>("/read_kalibr/opt_time_cost", opt_time_cost, 0.0);
    int opt_dim;
    nh.param<int>("/read_kalibr/opt_dim", opt_dim, 0);


    outFile_metrics << std::fixed << std::setprecision(6) << rotation_err0 << " " << translation_err0 << " "
                    << rotation_err1 << " " << translation_err1 << " " << time_offset_err << " "
                    << reproj_err << " " << opt_time_cost << " " << opt_dim;
    outFile_metrics << "\n";
    outFile_metrics.close();

    return 0;
}