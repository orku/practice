#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>
#include <pylon/BaslerUniversalInstantCameraArray.h>
#ifdef PYLON_WIN_BUILD
#    include <pylon/PylonGUI.h>
#endif
#include<opencv2/opencv.hpp>
#include<opencv2/calib3d.hpp>
#include<opencv2/calib3d/calib3d.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/video.hpp>
#include<opencv2/core.hpp>
#include <opencv2/core/hal/interface.h>
#include<opencv2/imgproc/imgproc.hpp>

// Namespace for using pylon objects.
using namespace Pylon;

// Namespace for using cout.
using std::cout;
using std::endl;
using std::vector;
using std::cerr;
using cv::Mat;




void BaslerCameraGrab(CBaslerUniversalInstantCamera& camera, static const uint32_t c_countOfImagesToGrab){
    camera.StartGrabbing();
    CGrabResultPtr ptrGrabResult;
    for (uint32_t i = 0; camera.IsGrabbing() && i <= c_countOfImagesToGrab; ++i)
    {
        // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
        camera.RetrieveResult(5000, ptrGrabResult, TimeoutHandling_ThrowException);
        // Image grabbed successfully?
        if (ptrGrabResult->GrabSucceeded())
        {
            // Access the image data.
            cout << "SizeX: " << ptrGrabResult->GetWidth() << endl;
            cout << "SizeY: " << ptrGrabResult->GetHeight() << endl;
            const uint8_t* pImageBuffer = (uint8_t*)ptrGrabResult->GetBuffer();
            cout << "Gray value of first pixel: " << (uint32_t)pImageBuffer[0] << endl << endl;
            // Display the grabbed image.
            Pylon::DisplayImage(0, ptrGrabResult);
        }
        else
        {
            cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << endl;
        }
    }
}

void BaslerCameraGrab(CBaslerUniversalInstantCamera& camera) {
    camera.StartGrabbing();
    CGrabResultPtr ptrGrabResult;
    while (camera.IsGrabbing())
    {
        // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
        camera.RetrieveResult(5000, ptrGrabResult, TimeoutHandling_ThrowException);
        // Image grabbed successfully?
        if (ptrGrabResult->GrabSucceeded())
        {
            // Access the image data.
            cout << "SizeX: " << ptrGrabResult->GetWidth() << endl;
            cout << "SizeY: " << ptrGrabResult->GetHeight() << endl;
            const uint8_t* pImageBuffer = (uint8_t*)ptrGrabResult->GetBuffer();
            cout << "Gray value of 90 pixel: " << (uint32_t)pImageBuffer[90] << endl << endl;
            // Display the grabbed image.
            Pylon::DisplayImage(0, ptrGrabResult);
        }
        else
        {
            cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << endl;
        }
    }
}

void create_array_of_images(std::ifstream& file, int count_of_images, vector<Mat>& img_array) {
    if (file.is_open())
    {
        for (int i = 0; i < count_of_images; ++i)
        {
            char string[20];
            file.getline(string, 20, '\n');
            cout << string << i << endl;
            img_array.push_back(cv::imread(string, CV_LOAD_IMAGE_UNCHANGED));
        }
        file.close();
    }
    else
    {
        std::cerr << "can not open file" << endl;
    }
}

void create_known_chessboadr_posiyion(vector<cv::Point3f>& object_corners, float calib_squar_edge_lenght, cv::Size board_sz) {
    for (int i = 0; i < board_sz.width; i++)
    {
        for (int j = 0; j < board_sz.height; j++)
        {
            object_corners.push_back(cv::Point3f(i * calib_squar_edge_lenght, j * calib_squar_edge_lenght, 0.0f));
        }
    }
}

int get_chessboard_corners(vector<Mat> img_arr, vector<vector<cv::Point2f>>& found_coreners, cv::Size board_sz) {
    int count_of_foud_patterns = 0;
    for (vector<Mat>::iterator i = img_arr.begin(); i != img_arr.end(); i++)
    {
        vector<cv::Point2f> bufffer;
        bool result = cv::findChessboardCorners(*i, board_sz, bufffer);
        if (result)
        {
            ++count_of_foud_patterns;
            found_coreners.push_back(bufffer);
            cv::drawChessboardCorners(*i, board_sz, bufffer, result);
            cv::imshow("CHESSBOARD", *i);
            cv::waitKey(200);
        }
        else
        {
            cout << "can not find chessboard" << endl;
        }
    }
    return count_of_foud_patterns;
}

size_t get_chessboard_corners(vector<Mat> img_arr, vector<vector<cv::Point2f>>& found_coreners, vector<vector<cv::Point3f>>& obj_points, vector<cv::Point3f> obj_corners, cv::Size board_sz, size_t success_cases) {
    while (found_coreners.size() != success_cases)
    {
        for (size_t i = 0; i != success_cases; i++)
        {
            vector<cv::Point2f> bufffer;
            bool result = cv::findChessboardCorners(img_arr[i], board_sz, bufffer);
            if (result)
            {
                found_coreners.push_back(bufffer);
                obj_points.push_back(obj_corners);
                cv::drawChessboardCorners(img_arr[i], board_sz, bufffer, result);
                cv::imshow("CHESSBOARD", img_arr[i]);
                cv::waitKey(50);
            }
            else
            {
                cout << "can not find chessboard" << endl;
            }
        }
        return found_coreners.size();
    }
}

size_t get_chessboard_corners(vector<Mat> img_arr, vector<vector<cv::Point2f>>& found_coreners, cv::Size board_sz, size_t success_cases) {
    while (found_coreners.size() != success_cases)
    {
        for (size_t i = 0; i != success_cases; i++)
        {
            vector<cv::Point2f> bufffer;
            bool result = cv::findChessboardCorners(img_arr[i], board_sz, bufffer);
            if (result)
            {
                found_coreners.push_back(bufffer);
                cv::drawChessboardCorners(img_arr[i], board_sz, bufffer, result);
                cv::imshow("CHESSBOARD", img_arr[i]);
                cv::waitKey(50);
            }
            else
            {
                cout << "can not find chessboard" << endl;
            }
        }
        return found_coreners.size();
    }
}

bool get_chessboard_corners(vector<Mat> img_arr_left, vector<Mat> img_arr_right, vector<vector<cv::Point2f>>& found_coreners_left,
    vector<vector<cv::Point2f>>& found_coreners_right, vector<vector<cv::Point3f>>& obj_points, vector <cv::Point3f> obj_corners, cv::Size board_sz, size_t success_cases_) {
    size_t success_cases_left = get_chessboard_corners(img_arr_left, found_coreners_left,obj_points, obj_corners, board_sz, success_cases_);
    size_t success_cases_right = get_chessboard_corners(img_arr_right, found_coreners_right, board_sz, success_cases_ - 1);
    if (success_cases_right == success_cases_left && obj_points.size() == success_cases_left) 
    {
        cout << "All nice!" << endl;
        return true;
    }
    else
    {
        cerr << "Nmber of images not euqal number of aray of chessboar points" << endl;
        return false;
    }

}

static const size_t c_maxCamerasToUse = 2; 
// Number of images to be grabbed.
static const uint32_t c_countOfImagesToGrab = 300;

int main(int argc, char* argv[])
{
    int max_disp = 160;
    int window_size = 3;
    int exitCode = 0;
    const size_t count_of_camers = 2;
    const float calibration_squar_edge_length = 0.03f; // in meters
    const cv::Size board_sz(7, 5);
    const size_t success_cases = 5;
    cv::Size image_size(720, 576);  
    vector<cv::Point2f > corners_1, corners_2;
    vector<vector<cv::Point2f>> img_points_left, img_points_right;
    vector<cv::Point3f> object_corners;
    vector<vector<cv::Point3f>> object_points;
    vector<Mat> left_calibration_array, right_calibration_array;   
    std::ifstream left_file, right_file;
    left_file.open("left.txt");
    right_file.open("right.txt");

    create_array_of_images(left_file, 5, left_calibration_array);
    create_array_of_images(right_file, 5, right_calibration_array);

    create_known_chessboadr_posiyion(object_corners, calibration_squar_edge_length, board_sz);
    //get_chessboard_corners(left_calibration_array, right_calibration_array, img_points_left, img_points_right,
    //    object_points, object_corners, board_sz, success_cases);
    get_chessboard_corners(left_calibration_array, img_points_left, object_points, object_corners, board_sz, success_cases);
    get_chessboard_corners(right_calibration_array, img_points_right, board_sz, success_cases);
    cout << "number of left images in array  " << img_points_left.size() << " number of right images in array " << img_points_right.size()
         << " number of arrays of chessboard points in array " << object_points.size() << endl;
    Mat camera_matrix_right, dist_coefs_right, camera_matrix_left, dist_coefs_left, R, T, E, F;
    cv::stereoCalibrate(object_points, img_points_left, img_points_right, camera_matrix_left, dist_coefs_left,
        camera_matrix_right, dist_coefs_right, cv::Size(720, 576), R, T, E, F, cv::CALIB_RATIONAL_MODEL | cv::CALIB_FIX_PRINCIPAL_POINT);

    Mat undistort_right, undistort_left, left_cam_matr, left_dist, undis_l;

    cv::calibrateCamera(object_points, img_points_left, image_size, left_cam_matr, left_dist, cv::noArray(), cv::noArray());
    
    cv::undistort(left_calibration_array[2], undistort_left, camera_matrix_left, dist_coefs_left, cv::noArray());
    cv::undistort(left_calibration_array[2], undis_l, left_cam_matr, left_dist, cv::noArray());
    cv::undistort(right_calibration_array[2], undistort_right, camera_matrix_right, dist_coefs_right, cv::noArray());
    cv::imshow("undistorted left", undistort_left);
    cv::imshow("undistorted kkbkbn", undis_l);
    cv::waitKey(300);








    cv::Ptr<cv::StereoBM> sgbm_matcher = cv::StereoBM::create(max_disp, 21);
    sgbm_matcher->setTextureThreshold(0);
    sgbm_matcher->setUniquenessRatio(0);
    //cv::Ptr<cv::StereoSGBM> sgbm_matcher = cv::StereoSGBM::create(0, max_disp, window_size);
    //sgbm_matcher->setP1(24 * window_size * window_size);
    //sgbm_matcher->setP2(96 * window_size * window_size);
    //sgbm_matcher->setPreFilterCap(63);
    //sgbm_matcher->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);

    //Mat R1, R2, P1, P2, map11, map12, map21, map22, new_camera_matrix_left, new_camera_matrix_right, Q, left_r, right_r, disp, norm_disp;

    //cv::stereoRectify(camera_matrix_left, dist_coefs_left, camera_matrix_right, dist_coefs_right, cv::Size(720, 576), R, T, R1, R2, P1, P2, Q);

    //cv::initUndistortRectifyMap(camera_matrix_left, dist_coefs_left, R1, P1, image_size, CV_16SC2, map11, map12);
    //cv::initUndistortRectifyMap(camera_matrix_right, dist_coefs_right, R2, P2, image_size, CV_16SC2, map21, map22);

    //cv::initUndistortRectifyMap(camera_matrix_left, dist_coefs_left, R1, new_camera_matrix_left, image_size, CV_16SC2, map11, map12);
    //cv::initUndistortRectifyMap(camera_matrix_right, dist_coefs_right, R2, new_camera_matrix_right, image_size, CV_16SC2, map21, map22);
    Mat left_lines, right_lines;
    cv::computeCorrespondEpilines(img_points_left, 1, F, left_lines);
    cv::computeCorrespondEpilines(img_points_right, 2, F, right_lines);
    //for (size_t i = 0; i < left_lines.size(); i++)
    //{
    //    cout << left_lines[i];
    //}
    cv::imshow("ksdjkasj", left_lines);

    //for (size_t i = 0; i < left_calibration_array.size() && i < right_calibration_array.size(); i++)
    //{

    //    cv::waitKey(200);
    //}

    if (false) {// Before using any pylon methods, the pylon runtime must be initialized. 
        PylonInitialize();

        try
        {
            CTlFactory& TlFactory = CTlFactory::GetInstance();
            DeviceInfoList_t listDevices;
            DeviceInfoList devices_list;
            TlFactory.EnumerateDevices(devices_list);
            if (devices_list.empty())
            {
                cerr << "No devuce found!" << endl;
            }

            IPylonDevice* right_dev(TlFactory.CreateDevice(devices_list[0]));
            IPylonDevice* left_dev(TlFactory.CreateDevice(devices_list[1]));

            CBaslerUniversalInstantCamera right_camera(TlFactory.CreateDevice(devices_list[0]));
            CBaslerUniversalInstantCamera left_camera(TlFactory.CreateDevice(devices_list[1]));
            cout << right_camera.GetDeviceInfo().GetFullName() << "   " << left_camera.GetDeviceInfo().GetFullName() << endl;

            right_camera.Open();
            left_camera.Open();
            right_camera.ExposureTimeRaw.SetValue(20000);
            left_camera.ExposureTimeRaw.SetValue(20000);

            //BaslerCameraGrab(right_camera, c_countOfImagesToGrab);
            //BaslerCameraGrab(left_camera, c_countOfImagesToGrab);
            cout << left_camera.ExposureTimeRaw.GetValue() << "     " << right_camera.ExposureTimeRaw.GetValue() << endl;

            CImageFormatConverter image_converter;
            image_converter.OutputPixelFormat = PixelType_BGR8packed;

            CPylonImage pylon_img_right, pylon_img_left;

            right_camera.StartGrabbing();
            left_camera.StartGrabbing();
            CGrabResultPtr ptrGrabResult_right;
            CGrabResultPtr ptrGrabResult_left;

            for (uint32_t i = 0; right_camera.IsGrabbing() && left_camera.IsGrabbing() && i <= c_countOfImagesToGrab; ++i)
            {
                // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
                right_camera.RetrieveResult(5000, ptrGrabResult_right, TimeoutHandling_ThrowException);
                left_camera.RetrieveResult(5000, ptrGrabResult_left, TimeoutHandling_ThrowException);
                // Image grabbed successfully?
                if (ptrGrabResult_right->GrabSucceeded() && ptrGrabResult_left->GrabSucceeded())
                {
                    // Access the image data.
                    cout << "we grab it" << endl;
                    //const uint8_t* pImageBuffer = (uint8_t*)ptrGrabResult_right->GetBuffer();
                    //show pylon images
                    //Pylon::DisplayImage(0, ptrGrabResult_right);
                    //Pylon::DisplayImage(1, ptrGrabResult_left);

                    image_converter.Convert(pylon_img_right, ptrGrabResult_right);
                    image_converter.Convert(pylon_img_left, ptrGrabResult_left);
                    //show opencv images
                    cv::Mat opencv_image_right(ptrGrabResult_right->GetHeight(), ptrGrabResult_right->GetWidth(), CV_8UC3, (uint8_t*)pylon_img_right.GetBuffer());
                    cv::Mat opencv_image_left(ptrGrabResult_left->GetHeight(), ptrGrabResult_left->GetWidth(), CV_8UC3, (uint8_t*)pylon_img_left.GetBuffer());
                    Mat left_img, right_img;
                    cout << ptrGrabResult_left->GetWidth() << "  " << ptrGrabResult_left->GetHeight() << " open cv" << opencv_image_left.cols << "   " << opencv_image_left.rows << endl;
                    cv::imshow("open cv window right", opencv_image_right);
                    cv::imshow("open cv window left", opencv_image_left);
                    cv::cvtColor(opencv_image_left, left_img, CV_BGR2GRAY);
                    cv::cvtColor(opencv_image_right, right_img, CV_BGR2GRAY);
                    cv::imshow("open cv converted img", left_img);

                    Mat result_sgbm, sb_8, sb_8_r;
                    //Mat opencv_image_right_r, opencv_image_left_r, result_sgbm_r;
                    //cv::remap(left_img, opencv_image_right_r, map21, map22, cv::INTER_LINEAR);
                    //cv::remap(right_img, opencv_image_left_r, map11, map12, cv::INTER_LINEAR);
                    //cv::imshow("remap img ", opencv_image_left_r);
                    sgbm_matcher->compute(left_img, right_img, result_sgbm);
                    //sgbm_matcher->compute(opencv_image_right_r, opencv_image_left_r, result_sgbm_r);
                    cv::normalize(result_sgbm, sb_8, 0, 255, CV_MINMAX, CV_8U);
                    cv::imshow("dispariry map ", sb_8);
                    cv::waitKey(1);
                }
                else
                {
                    cout << "Error: " << ptrGrabResult_right->GetErrorCode() << " " << ptrGrabResult_right->GetErrorDescription() << endl;
                }
            }   
            right_camera.Close();
            left_camera.Close();
        }
        catch (const GenericException& e)
        {
            // Error handling
            cerr << "An exception occurred." << endl
                << e.GetDescription() << endl;
            exitCode = 1;
        }

    }

    // Comment the following two lines to disable waiting on exit.
    cerr << endl << "Press enter to exit." << endl;
    while (std::cin.get() != '\n');
    // Releases all pylon resources. 
    PylonTerminate();

    return exitCode;
}

