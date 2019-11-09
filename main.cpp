#include <iostream>
#include <fstream>
#include <unistd.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "sophus/se3.hpp"
#include <pangolin/pangolin.h>

//  添加轨迹图版本
using namespace std;

typedef vector<Sophus::SE3d> PoseType;
PoseType ReadTrajectory(const string &path);
void Trajectoryviewer(const PoseType &pt);

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
    Trajectoryviewer(point_test);
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
    void Trajectoryviewer(const PoseType &pt) {
        pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
                pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
        );

        pangolin::View &d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
                .SetHandler(new pangolin::Handler3D(s_cam));


        while (pangolin::ShouldQuit() == false) {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            d_cam.Activate(s_cam);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

            glLineWidth(2);
            for (int i = 0; i < pt.size() - 1; i++) {
                glColor3f(0.0f, 0.0f, 1.0f);  // blue for ground truth
                glBegin(GL_LINES);
                auto p1 = pt[i], p2 = pt[i + 1];
                glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
                glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
                glEnd();
            }
            pangolin::FinishFrame();
            usleep(5000);   // sleep 5 ms}
        }
    }