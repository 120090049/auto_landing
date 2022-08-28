#include <opencv2/aruco.hpp> 
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;



int main(int argc, char** argv)
{
    cv::Mat cameraMatrix, distCoeffs;
    vector<double> camera = { 657.1548323619423, 0, 291.8582472145741,0, 647.384819351103, 391.254810476919,0, 0, 1 };
    cameraMatrix = Mat(camera);
    cameraMatrix = cameraMatrix.reshape(1,3);
    vector<double> dist = { 0.1961793476399528, -1.38146317350581, -0.002301820186177369, -0.001054637905895881, 2.458286937422959 };
    distCoeffs = Mat(dist);
    distCoeffs = distCoeffs.reshape(1, 1);


    // generate marker
    std::string input_file_name = "/home/clp/catkin_ws/src/auto_landing/test/test2.png";
    
    cv::Mat image, imageCopy;
    image = imread(input_file_name);
    image.copyTo(imageCopy);

    std::vector<int> v_marker_id;
    std::vector<std::vector<cv::Point2f> > v_marker_corner;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::detectMarkers(image, dictionary, v_marker_corner, v_marker_id);
    if (v_marker_id.size() > 0)
    {
        cout << "nice!" << endl;
        aruco::drawDetectedMarkers(imageCopy, v_marker_corner, v_marker_id, Scalar(0, 255, 0));
    }    
    

    std::vector<cv::Vec3d> rvecs;
	std::vector<cv::Vec3d> tvecs;
	cv::aruco::estimatePoseSingleMarkers(v_marker_corner, 0.053, cameraMatrix, distCoeffs, rvecs, tvecs);
	for (int i = 0;i < rvecs.size();i++)
	{
		//绘制坐标轴，检查姿态估计结果
		cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.02);
	}
	imshow("pose", imageCopy);

    
   
    waitKey(0);
    // char key = (char) cv::waitKey(waitTime);
    // if (key == 27)
    //     break;
    std::cout << "done!" << std::endl;
    cout << v_marker_corner.size();




    return 0;
}



