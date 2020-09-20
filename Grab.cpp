#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>
#include <pylon/BaslerUniversalInstantCameraArray.h>
#ifdef PYLON_WIN_BUILD
#    include <pylon/PylonGUI.h>
#endif

// Namespace for using pylon objects.
using namespace Pylon;

// Namespace for using cout.
using namespace std;

// Number of images to be grabbed.
static const uint32_t c_countOfImagesToGrab = 100;


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

        right_camera.StartGrabbing();
        CGrabResultPtr ptrGrabResult;
        //TODO допилить это всё
        while (right_camera.IsGrabbing())
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
                cout << "Gray value of first pixel: " << (uint32_t)pImageBuffer[0] << endl << endl;


                // Display the grabbed image.
                Pylon::DisplayImage(1, ptrGrabResult);

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

