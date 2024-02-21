#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
int main()
{
    string sequence = "05";
    string pose_file = "/home/kaijun/Desktop/Datasets/kitti/data_odometry_poses/dataset/poses/" + sequence + ".txt";
    string time_file = "/home/kaijun/Desktop/Datasets/kitti/data_odometry_calib/dataset/sequences/" + sequence + "/times.txt";
    string calib_file = "/home/kaijun/Desktop/Datasets/kitti/data_odometry_calib/dataset/sequences/" + sequence + "/calib.txt";
    string tum_pose_file = "/home/kaijun/Desktop/Datasets/kitti/tum_pose/kitti" + sequence + "_tum.txt";
    ifstream fin_calib(calib_file);
    string line;
    vector<double> Tr;
    for (int i = 0; i < 5; i++) {
        getline(fin_calib, line);
        if(i == 4) {
            istringstream line_stream(line);
            string info;
            getline(line_stream, info, ':');
            getline(line_stream, info, ' ');
            while(getline(line_stream, info, ' ')) {
                double data;
                stringstream ss;
                ss << info;
                ss >> data;
                Tr.push_back(data);
            }
        }
    }
    Eigen::Matrix4d cam2lidar_se3;
    cam2lidar_se3 << Tr[0], Tr[1], Tr[2], Tr[3],
                     Tr[4], Tr[5], Tr[6], Tr[7],
                     Tr[8], Tr[9], Tr[10], Tr[11],
                     0, 0, 0, 1;
    cout << cam2lidar_se3 << endl;

    ifstream fin_pose(pose_file);
    ifstream fin_time(time_file);
    vector<Eigen::Matrix4d> pose_cam;
    vector<Eigen::Matrix4d> pose_lidar;
    vector<double> times;
    while(getline(fin_pose, line)) {
        istringstream line_stream(line);
        string info;
        Tr.clear();
        while(getline(line_stream, info, ' ')) {
            double data;
            stringstream ss;
            ss << info;
            ss >> data;
            Tr.push_back(data);
        }
        Eigen::Matrix4d single_pose_cam;
        single_pose_cam << Tr[0], Tr[1], Tr[2], Tr[3],
                            Tr[4], Tr[5], Tr[6], Tr[7],
                            Tr[8], Tr[9], Tr[10], Tr[11],
                            0, 0, 0, 1;
        pose_cam.push_back(single_pose_cam);
    }
    cout << pose_cam.size() << endl;

    while(getline(fin_time, line)) {
        double time;
        stringstream ss;
        ss << line;
        ss >> time;
        times.push_back(time);
    }
    cout << times.size() << endl;

    Eigen::Matrix4d pose_lidar_init = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d single_pose_lidar = pose_lidar_init;
    pose_lidar.push_back(single_pose_lidar);
    for (int i = 1; i < times.size(); i++) {
        Eigen::Matrix4d pose_cam_curr = pose_cam[i];
        Eigen::Matrix4d pose_cam_prev = pose_cam[i-1];
        Eigen::Matrix4d rel_cam2cam = pose_cam_prev.inverse() * pose_cam_curr;
        Eigen::Matrix4d rel_lidar2lidar = cam2lidar_se3.inverse() * rel_cam2cam * cam2lidar_se3;
        single_pose_lidar = single_pose_lidar * rel_lidar2lidar;
        pose_lidar.push_back(single_pose_lidar);
    }

    ofstream data_output;
    data_output.open(tum_pose_file, std::ios::ate | std::ios::out);
    for (int i = 0; i < times.size(); i++) {
        Eigen::Quaterniond q_lidar(pose_lidar[i].block<3, 3>(0, 0));
        Eigen::Vector3d t_lidar(pose_lidar[i].block<3, 1>(0, 3));
        data_output << times[i] << ' ' << t_lidar[0] << ' ' << t_lidar[1] << ' ' << t_lidar[2] << ' ' 
                    << q_lidar.x() << ' ' << q_lidar.y() << ' ' << q_lidar.z() << ' ' << q_lidar.w() << std::endl;
    }
}