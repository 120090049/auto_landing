#include <opencv2/aruco.hpp> 
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>
// opencv头文件
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>


using namespace cv;
using namespace std;



int main(int argc, char** argv)
{
    // cv::Vec3d clp;
    // clp[0] = -2;
    // clp[1] = -2;
    // clp[2] = 0;
    // cv::Mat rotation_matrix_temp;
  

    // cv::Rodrigues(clp, rotation_matrix_temp);
    // cout << rotation_matrix_temp;
    Vec3d a(-2.29155, -2.15596, 0.0147755);
    Vec3d b(2.28913, 2.15385, -0.0158986);
    Mat a_r, b_r;
    Rodrigues(a, a_r);
    Rodrigues(b, b_r);
    a_r.convertTo(a_r, CV_32FC1);
    b_r.convertTo(b_r, CV_32FC1);
    // cout << a_r << endl;
    // cout << b_r;

cv::Vec3d unit_v;           
unit_v[0] = 1.;
unit_v[1] = 0.;
unit_v[2] = 0.;
cv::Mat unit_mat{unit_v};
unit_mat.convertTo(unit_mat, CV_32FC1);


cv::Mat out_a, out_b;
out_a = a_r * unit_mat;
out_b = b_r * unit_mat;
cout << out_a << endl;
cout << out_b;
return 0;
}



