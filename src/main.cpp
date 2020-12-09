#include <iostream>
#include <opencv2/opencv.hpp> 

#define KEY_ESC 27 // Key to close the program

int main(int argc, char **argv)
{
    //TODO: read video device number from command line input
    // Initialize video device capture
    std::string device = "/dev/video4";
    int api_preference = cv::CAP_V4L;
    cv::VideoCapture cap(device, api_preference);
    if (!cap.isOpened()) {
        printf("Cannot open video stream: %s\n", device.c_str());
        return -1;
    }

    // Video capture properties
    cap.set(cv::CAP_PROP_FPS, 60);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 2560);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 960);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

    // Calibration properties
    cv::Size chess_board_size(9,6);
    int chess_board_flags = cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK;

    // Print some basic properties of the device
    double fps = cap.get(cv::CAP_PROP_FPS);
    int width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    printf("Captured device: %s\n", device.c_str());
    printf("    Framerate: %5.1f\n", fps);
    printf("    Image width:  %d\n", width);
    printf("    Image height: %d\n", height);

    // Set width of single image (divide concatenated stereo image by two)
    width /= 2;

    // Capture images
    while(1)
    {
        // Attempt to read the next image
        cv::Mat img;
        if (!cap.read(img)) {
            printf("Error reading image from %s\n", device.c_str());
            break;
        }

        // Split the concatenated stereo image into left and right images
        cv::Mat imgl = img(cv::Rect(0, 0, width, height));
        cv::Mat imgr = img(cv::Rect(width, 0, width, height));

        // Convert image to grayscale and refine the detected corners
        cv::Mat imgl_gray;
        cv::cvtColor(imgl, imgl_gray, cv::COLOR_BGR2GRAY);

        // Downsample the image for chessboard finding only
        cv::Mat imgl_gray_downsampled;
        cv::resize(imgl_gray, imgl_gray_downsampled, cv::Size(imgl.cols/4,imgl.rows/4));

        // Find chessboard pattern
        std::vector<cv::Point2f> pts;
        bool success = findChessboardCorners(imgl_gray_downsampled, chess_board_size, pts, chess_board_flags);

        if (success) {
            // Scale points back to full resolution
            for (int i = 0; i < pts.size(); ++i)
                pts[i] *= 4;

            // Refine the detected corners
            cv::cornerSubPix(imgl_gray, pts, cv::Size(31,31), cv::Size(-1,-1),
                             cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 30, 0.001));
            
            // Render the chessboard corners on the image
            cv::drawChessboardCorners(imgl, chess_board_size, cv::Mat(pts), success);
        }

        // Display the calibration image
        cv::imshow("Calibration Image", imgl);

        // Break on 'esc' key
        if(cv::waitKey(1) == KEY_ESC) 
            break;
    }

    return 0;
}