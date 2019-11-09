#include <iostream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "sophus/se3.hpp"

using namespace std;

typedef vector<Sophus::SE3d> PoseType;
PoseType ReadTrajectory(const string &path);

int main(int arc, char **argv) {
    int i,j;
    string file=string(argv[1]);
    i=atoi(argv[2]);
    j=atoi(argv[3]);
    PoseType point_test=ReadTrajectory(file);
    //Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    //W = Eigen::Vector3d(point_test[i].x,point_test[i].y,point_test[i].z) * Eigen::Vector3d(point_test[j].x, point_test[j].y, point_test[j].z).transpose();
    auto W = point_test[j].inverse()*point_test[i];
    auto w= W.matrix();
    cout << "第" <<i<<"帧相机位置与第"<<j<<"帧相机位置之间的变换矩阵为："<< endl;
    cout<<w<<endl;
    return 0;
}

    PoseType ReadTrajectory(const string &path){
    ifstream fin(path);
    PoseType point;
    if (!fin){
        cerr<<"轨迹文件路径错误"<<path<<endl;
        return point;
    }
    while(!fin.eof()){
        double timestamp,tx,ty,tz,qx,qy,qz,qw;
        fin >>timestamp>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
        Sophus::SE3d p1(Eigen::Quaterniond(qx,qy,qz,qw),Eigen::Vector3d(tx,ty,tz));
        point.push_back(p1);
        //cv::Point3d point_3;
        //point_3=cv::Point3d(tx,ty,tz);
        //Eigen::Matrix<double,6,1> se3=p1.log().transpose();
        //auto se3=p1.matrix();
        //cout<<se3<<endl;
    }
    return point;
}