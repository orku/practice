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
#include "Grab.h"

// Namespace for using pylon objects.
using namespace Pylon;

// Namespace for using cout.
using namespace std;

// Number of images to be grabbed.
static const uint32_t c_countOfImagesToGrab = 100;

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
            img_array.push_back(cv::imread(string, CV_8UC3));
        }
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
            cv::imshow("sds", *i);
            cv::waitKey(200);
        }
        else
        {
            cout << "can not find chessboard" << endl;
        }
    }
    return count_of_foud_patterns;
}

int get_chessboard_corners(vector<Mat> img_arr, vector<vector<cv::Point2f>>& found_coreners, cv::Size board_sz, int success_cases) {
    int count_of_foud_patterns = 0;
    while (count_of_foud_patterns != success_cases)
    {
        for (vector<Mat>::iterator i = img_arr.begin(); i != img_arr.end() && count_of_foud_patterns != success_cases; i++)
        {
            vector<cv::Point2f> bufffer;
            bool result = cv::findChessboardCorners(*i, board_sz, bufffer);
            if (result)
            {
                ++count_of_foud_patterns;
                found_coreners.push_back(bufffer);
                cv::drawChessboardCorners(*i, board_sz, bufffer, result);
                cv::imshow("sds", *i);
                cv::waitKey(200);
            }
            else
            {
                cout << "can not find chessboard" << endl;
            }
        }
        return count_of_foud_patterns;
    }
}

bool get_chessboard_corners_stereo(vector<Mat> img_arr_left, vector<Mat> img_arr_right, vector<vector<cv::Point2f>>& found_coreners_left,
    vector<vector<cv::Point2f>>& found_coreners_right, cv::Size board_sz, int success_cases_) {
    int success_cases_left = get_chessboard_corners(img_arr_left, found_coreners_left, board_sz, success_cases_);
    int success_cases_right = get_chessboard_corners(img_arr_right, found_coreners_right, board_sz, success_cases_);
    if (success_cases_right == success_cases_left)
    {
        cout << "All nice!" << endl;
        return true;
    }
    else
    {
        cout << " Something get wrong" << endl;
        return false;
    }

}


// Limits the amount of cameras used for grabbing.
// It is important to manage the available bandwidth when grabbing with multiple cameras.
// This applies, for instance, if two GigE cameras are connected to the same network adapter via a switch.
// To manage the bandwidth, the GevSCPD interpacket delay parameter and the GevSCFTD transmission delay
// parameter can be set for each GigE camera device.
// The "Controlling Packet Transmission Timing with the Interpacket and Frame Transmission Delays on Basler GigE Vision Cameras"
// Application Notes (AW000649xx000)
// provide more information about this topic.
// The bandwidth used by a FireWire camera device can be limited by adjusting the packet size.
static const size_t c_maxCamerasToUse = 2; 

int main(int argc, char* argv[])
{
    // The exit code of the sample application.
    int exitCode = 0;
    const size_t count_of_camers = 2;
    const float calibration_squar_edge_length = 0.03f; // in meters
    const cv::Size_<int> board_sz = cv::Size(7, 5);
    std::vector<cv::Point2f > corners_1, corners_2;
    std::vector<std::vector<cv::Point2f>> img_points_left, img_points_right;
    std::vector<cv::Point3f> object_corners;
    std::vector<std::vector<cv::Point3f>> object_points;

    cout << " create_known_board_position " << endl;
    for (int i = 0; i < board_sz.width; ++i) {
        for (int j = 0; j < board_sz.height; ++j) {
            //cout << i * calibration_squar_edge_length << ' ' << j * calibration_squar_edge_length << endl;
            object_corners.push_back(cv::Point3f(i * calibration_squar_edge_length, j * calibration_squar_edge_length, 0.0f));
        }
    }



    // Before using any pylon methods, the pylon runtime must be initialized. 
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
        right_camera.ExposureTimeRaw.SetValue(15000);
        left_camera.ExposureTimeRaw.SetValue(15000);

        //BaslerCameraGrab(right_camera, c_countOfImagesToGrab);
        //BaslerCameraGrab(left_camera, c_countOfImagesToGrab);
        cout << left_camera.ExposureTimeRaw.GetValue() << "     " << right_camera.ExposureTimeRaw.GetValue() << endl;

        CImageFormatConverter image_converter;
        image_converter.OutputPixelFormat = PixelType_BGR8packed;

        CPylonImage pylon_img;


        right_camera.StartGrabbing();
        CGrabResultPtr ptrGrabResult;
        for (uint32_t i = 0; right_camera.IsGrabbing() && i <= c_countOfImagesToGrab; ++i)
        {
            // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
            right_camera.RetrieveResult(5000, ptrGrabResult, TimeoutHandling_ThrowException);
            // Image grabbed successfully?
            if (ptrGrabResult->GrabSucceeded())
            {
                // Access the image data.
                cout << "SizeX: " << ptrGrabResult->GetWidth() << endl;
                cout << "SizeY: " << ptrGrabResult->GetHeight() << endl;
                const uint8_t* pImageBuffer = (uint8_t*)ptrGrabResult->GetBuffer();
                //cout << "Gray value of first pixel: " << (uint32_t)pImageBuffer[0] << endl << endl;
                // Display the grabbed image.
                Pylon::DisplayImage(0, ptrGrabResult);
                image_converter.Convert(pylon_img, ptrGrabResult);
                cout << " here some shit" << endl;
                cv::Mat opencv_image(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3 , (uint8_t*)pylon_img.GetBuffer());
                cout << "it works" << endl;
                cv::namedWindow("open cv window", CV_WINDOW_NORMAL);
                cv::imshow("open cv window", opencv_image);
                cv::waitKey(1);
                cout << "let's wait" << endl;
                bool result_right = cv::findChessboardCorners(opencv_image, board_sz, corners_1);
                if (result_right) {
                    cv::Mat opencv_image_with_corners(opencv_image);
                    cv::drawChessboardCorners(opencv_image_with_corners, board_sz, corners_1, result_right);
                    cv::imshow("with corners", opencv_image);
                    cv::waitKey(100000);
                    cout << "we find it" << endl;
                }

            }
            else
            {
                cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << endl;
            }
        }

    }
    catch (const GenericException& e)
    {
        // Error handling
        cerr << "An exception occurred." << endl
            << e.GetDescription() << endl;
        exitCode = 1;
    }

    // Comment the following two lines to disable waiting on exit.
    cerr << endl << "Press enter to exit." << endl;
    while (cin.get() != '\n');

    // Releases all pylon resources. 
    PylonTerminate();

    return exitCode;
}

