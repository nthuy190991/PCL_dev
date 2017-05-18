#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
    std::string imageName( "D:/Lenna.png" ); // by default
    if( argc > 1)
    {
        imageName = argv[1];
    }
    cv::Mat image;
    image = imread( imageName, IMREAD_COLOR ); // Read the file
    if( image.empty() )                      // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }
    cv::namedWindow( "Display window", WINDOW_AUTOSIZE ); // Create a window for display.
    cv::imshow( "Display window", image );                // Show our image inside it.
    cv::waitKey(0); // Wait for a keystroke in the window

    return 0;
}