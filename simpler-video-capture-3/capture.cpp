/*
 *
 *  Example by Sam Siewert 
 *
 *  Updated 10/29/16 for OpenCV 3.1
 *
 *  Updated to use C++ API only
 *
 *  Based on:
 *  https://docs.opencv.org/3.0-beta/modules/videoio/doc/reading_and_writing_video.html
 *  https://docs.opencv.org/3.3.0/d8/dfe/classcv_1_1VideoCapture.html
 *
 */
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

#define DEVICE_ID "/dev/video0"

int main( int argc, char** argv )
{
    VideoCapture capture_interface(DEVICE_ID);
    Mat capture_frame;

    if(argc > 1)
    {
        // Set up camera for specified device
        capture_interface.release();
        capture_interface.open(argv[1]);
        if (!capture_interface.isOpened()) { // check if we succeeded
          cout << "Failed to open input stream " << argv[1] << "!" << endl;
          exit(-1);
        } else {
          cout << "Successfully opened input stream " << argv[1] << "!" << endl;
        }
        namedWindow(argv[1], WINDOW_AUTOSIZE | CV_GUI_NORMAL);
    }
    else
    {
        // Set up camera for default device
        capture_interface.open(DEVICE_ID);
        if (!capture_interface.isOpened()) { // check if we succeeded
          cout << "Failed to open input stream " << DEVICE_ID << "!" << endl;
          exit(-1);
        } else {
          cout << "Successfully opened input stream " << DEVICE_ID << "!" << endl;
        }
        namedWindow(DEVICE_ID, WINDOW_AUTOSIZE | CV_GUI_NORMAL);
    }

    while(1)
    {
        capture_interface >> capture_frame;
     
        if(capture_frame.empty())
        {
            cout << "First frame error" << endl; continue;
        };

        if(argc > 1)
            imshow(argv[1], capture_frame);
        else
            imshow(DEVICE_ID, capture_frame);

        char c = cvWaitKey(33);
        if( c == 'q' ) break;
    }

    capture_interface.release();

    destroyAllWindows();

    return 0;    
};
