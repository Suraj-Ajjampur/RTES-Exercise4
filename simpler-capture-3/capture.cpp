/*
 *
 *  Example by Sam Siewert 
 *
 *  Updated 10/29/16 for OpenCV 3.1
 *
 *  Updated to use C++ API only
 *
 *  Based on: Learning OpenCV3
 *
 *  For some reason, the camera is often not closed out properly with C++ API,
 *  so you can try just waiting a few minutes, or:
 *
 *  1) unplug and replug, then do ls /dev/video*
 *  2) touch /dev/video0 until you get not error
 *  3) USB reset - https://askubuntu.com/questions/645/how-do-you-reset-a-usb-device-from-the-command-line
 *  4) sudo modprobe uvcvideo
 *
 */
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define DEFAULT_CAMERA "/dev/video0"

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
    VideoCapture capture_interface;
    Mat capture_frame;
    namedWindow("Example3", WINDOW_AUTOSIZE);

    if(argc == 1)
    {
        capture_interface.open(DEFAULT_CAMERA);
    }
    else
    {
        capture_interface.open(argv[1]);
    }

    if(!capture_interface.isOpened())
    {
        if(argc == 1)
            cerr << "Could not open camera interface " << DEFAULT_CAMERA << endl;
        else
            cerr << "Could not open camera interface " << argv[1] << endl;
        return -1;
    }

    while(1)
    {
        capture_interface >> capture_frame;
     
        if(capture_frame.empty())
        {
            cout << "First frame error" << endl; continue;
        };

        imshow("Example3", capture_frame);

        char c = cvWaitKey(33);
        if( c == 'q' ) break;
    }

    capture_interface.release();

    destroyAllWindows();

    cout << "Releasing resources" << endl;

    return 0;    
};
