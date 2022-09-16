#include <opencv2/aruco.hpp> 
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;



int main(int argc, char** argv)
{

    cv::Mat cameraMatrix, distCoeffs;
    // vector<double> camera = { 657.1548323619423, 0, 291.8582472145741,0, 647.384819351103, 391.254810476919,0, 0, 1 };
    vector<double> camera = {374.203195, 0.000000, 639.658565, 0.000000, 374.197062, 359.992852, 0, 0, 1 };
    cameraMatrix = Mat(camera);
    cameraMatrix = cameraMatrix.reshape(1,3);
    cout << cameraMatrix << endl;
    // vector<double> dist = { 0.1961793476399528, -1.38146317350581, -0.002301820186177369, -0.001054637905895881, 2.458286937422959 };
    vector<double> dist = { 0.000071, -0.000018, 0.000039, -0.000070, 0.000000 };
    distCoeffs = Mat(dist);
    distCoeffs = distCoeffs.reshape(1, 1);

    // std::string input_file_name = "/home/clp/catkin_ws/src/auto_landing/file/+.jpg";
    // t [0.084337, -0.0969064, 0.150258]
///////////////
// [370, 0, 640;
//  0, 375, 360;
//  0, 0, 1]
// [914, 53;
//  916, 184;
//  785, 185;
//  783, 53]
// [2.21851, 2.20437, -0.0128644]
/////////////////////
    // generate marker
    std::string input_file_name = "/home/clp/catkin_ws/src/auto_landing/file/-.jpg";
    //t [0.0832296, -0.0921749, 0.149788]
    ///////////////
//     [370, 0, 640;
//      0, 375, 360;
//      0, 0, 1]
// [942, 198;
//  942, 223;
//  915, 223;
//  916, 197]
// [-2.22492, -2.22287, 0.0182096]
// done!
    //////////////
    cv::Mat image, imageCopy, imageCopy2;
    image = imread(input_file_name);
    image.copyTo(imageCopy);
    image.copyTo(imageCopy2);
    std::vector<int> v_marker_id;
    std::vector<std::vector<cv::Point2f> > v_marker_corner;
    //[2.194, 2.06383, 0.0105061]
    // cv::Point2f a(617, 97);
    // cv::Point2f b(624, 206);
    // cv::Point2f c(514, 213);
    // cv::Point2f d(507, 103);

    //[2.19479, 2.06562, 0.00759637]
    // cv::Point2f a(617, 96);
    // cv::Point2f b(624, 206);
    // cv::Point2f c(514, 212);
    // cv::Point2f d(507, 103);
    // std::vector<cv::Point2f> ma;
    // ma.push_back(a);
    // ma.push_back(b);
    // ma.push_back(c);
    // ma.push_back(d);
    // v_marker_corner.push_back(ma);

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::detectMarkers(image, dictionary, v_marker_corner, v_marker_id);
    // if (v_marker_id.size() > 0)
    // {
    //     cout << "nice!" << endl;
    //     aruco::drawDetectedMarkers(imageCopy, v_marker_corner, v_marker_id, Scalar(0, 255, 0));
    //     aruco::drawDetectedMarkers(imageCopy2, v_marker_corner, v_marker_id, Scalar(0, 255, 0));
    // }    
    

    std::vector<cv::Vec3d> rvecs;
	std::vector<cv::Vec3d> tvecs;
	cv::aruco::estimatePoseSingleMarkers(v_marker_corner, 0.053, cameraMatrix, distCoeffs, rvecs, tvecs);
	for (int i = 0;i < rvecs.size();i++)
	{
		//绘制坐标轴，检查姿态估计结果
		cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.02);
        cout << "clp" << v_marker_id[i] << endl;
        if (v_marker_id[i] == 19){
            cout << v_marker_corner[i] << endl;

            cv::Mat rotation_matrix; // 旋转向量转旋转矩阵

            cv::Rodrigues(rvecs[i], rotation_matrix);
            cout << rotation_matrix << endl;
        }
	}
	imshow("pose", imageCopy);
    waitKey(0);
    // for (int i = 0;i < rvecs.size();i++)
	// {
    //     rvecs[i][0] = -rvecs[i][0];
    //     rvecs[i][2] = -rvecs[i][2];
	// 	//绘制坐标轴，检查姿态估计结果
	// 	cv::aruco::drawAxis(imageCopy2, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.02);
	// }
	// imshow("pose2", imageCopy2);

    
   
    // waitKey(0);
    // char key = (char) cv::waitKey(waitTime);
    // if (key == 27)
    //     break;
    std::cout << "done!" << std::endl;
    // cout << v_marker_corner.size();




    return 0;
}



