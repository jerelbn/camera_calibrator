#include <iostream>
#include <opencv2/opencv.hpp> 

#define KEY_BREAK 27        // ESC closes the program
#define KEY_ADD_PTS 32      // SPACEBAR adds detected points to the calibration buffer
#define KEY_REMOVE_PTS 8    // BACKSPACE removes last added set of points from the calibration buffer
#define KEY_CLEAR_PTS 255   // DELETE clears all points from the calibration buffer
#define KEY_CALIBRATE 13    // RETURN runs the calibration routine on the collected points
#define KEY_RESTART 114     // R restarts the calibration collection process
#define DOWNSAMPLE_FACTOR 4 // Dividing factor for downsampling the image
static const cv::Scalar OPENCV_RED = cv::Scalar(0,0,255);

enum Pattern : int {
    CHECKERBOARD = 0,
    ASYMMETRIC_CIRCLES = 1
};

int main(int argc, char **argv)
{
    // Read command line inputs
    if (argc == 2 && strcmp(argv[1], "--help") == 0) {
        printf("\n**Camera Calibrator Help**\n\n");
        printf("Usage: ./camera_calibrator <video_device_number> <pattern_type> <save_filename>\n");
        printf("\npattern_type:\n    0 - checkerboard\n    1 - asymmetric circle grid\n");
        printf("\nControls:\n");
        printf("    ESC         closes the program\n");
        printf("    SPACEBAR    adds detected points to the calibration buffer\n");
        printf("    BACKSPACE   removes last added set of points from the calibration buffer\n");
        printf("    DELETE      clears all points from the calibration buffer\n");
        printf("    RETURN      runs the calibration routine on the collected points\n");
        printf("    R           restarts the calibration collection process\n");
        return 0;
    }
    if (argc < 4) {
        printf("Not enough input arguments.\n");
        return -1;
    }
    std::string device = "/dev/video" + std::string(argv[1]);
    Pattern pattern = (Pattern)std::strtol(argv[2], argv, 10);
    std::string filename = argv[3];

    // Initialize video device capture
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

    // Calibration properties and variables
    int flags;
    cv::Size board_size;
    float shape_separation;
    std::vector<std::vector<cv::Point3f>> pts_obj(1);
    switch (pattern) {
        case Pattern::CHECKERBOARD:
            flags = cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK;
            board_size = cv::Size(9,6);
            shape_separation = 23;
            for(int i = 0; i < board_size.height; ++i)
                    for(int j = 0; j < board_size.width; ++j)
                        pts_obj[0].push_back(cv::Point3f(j*shape_separation, i*shape_separation, 0));
            break;
        case Pattern::ASYMMETRIC_CIRCLES:
            flags = cv::CALIB_CB_ASYMMETRIC_GRID + cv::CALIB_CB_CLUSTERING;
            board_size = cv::Size(4,11);
            shape_separation = 20;
            for(int i = 0; i < board_size.height; ++i)
                    for(int j = 0; j < board_size.width; ++j)
                        pts_obj[0].push_back(cv::Point3f((2*j + i%2)*shape_separation, i*shape_separation, 0));
            break;
        default:
            std::cout << "Select a valid pattern:\n  0 - checkerboard\n  1 - asymmetric circles\n";
            break;
    }

    std::vector<std::vector<cv::Point2f>> pts_cal;
    int iFixedPoint = board_size.width - 1;
    cv::Mat camera_matrix = cv::Mat::eye(3,3,CV_64F);
    cv::Mat dist_coeffs = cv::Mat::zeros(8,1,CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;
    bool calibrating = true;
    cv::Mat imgl_undistorted, imgl_diff;
    cv::FileStorage fs;

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
        cv::resize(imgl_gray, imgl_gray_downsampled, cv::Size(imgl.cols,imgl.rows)/DOWNSAMPLE_FACTOR);

        // Find chessboard pattern
        std::vector<cv::Point2f> pts;
        bool success;
        switch (pattern) {
            case Pattern::CHECKERBOARD:
                success = cv::findChessboardCorners(imgl_gray_downsampled, board_size, pts, flags);
                break;
            case Pattern::ASYMMETRIC_CIRCLES:
                success = cv::findCirclesGrid(imgl_gray_downsampled, board_size, pts, flags);
                break;
            default:
                std::cout << "Select a valid pattern:\n  0 - checkerboard\n  1 - asymmetric circles\n";
                break;
        }

        if (success) {
            // Scale points back to full resolution
            for (int i = 0; i < pts.size(); ++i)
                pts[i] *= DOWNSAMPLE_FACTOR;

            // Refine the detected corners for checkerboard patterns
            if (pattern == Pattern::CHECKERBOARD) {
                cv::cornerSubPix(imgl_gray, pts, cv::Size(31,31), cv::Size(-1,-1),
                                 cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 30, 0.0001));
            }
            
            // Render the chessboard corners on the image
            cv::drawChessboardCorners(imgl, board_size, cv::Mat(pts), success);
        }

        // Draw while collecting calibration points
        if (calibrating) {
            // Draw number of calibration points collected on the image
            cv::putText(imgl, "Cal size: " + std::to_string(pts_cal.size()), cv::Point(10,30), cv::FONT_HERSHEY_SIMPLEX, 1, OPENCV_RED, 2, cv::LINE_AA);

            // Display the calibration image
            cv::imshow("Calibration Image", imgl);
        }
        else {
            if (!camera_matrix.empty()) {
                // Show difference between regular and undistored images
                cv::undistort(imgl, imgl_undistorted, camera_matrix, dist_coeffs, cv::noArray());
                cv::absdiff(imgl, imgl_undistorted, imgl_diff);
                cv::imshow("Calibration Image", imgl_diff);
            }
        }

        // Break on 'esc' key
        int key = cv::waitKey(10);
        if(key == KEY_BREAK) {
            break;
        }
        else if (key == KEY_ADD_PTS) {
            if (pts.size() > 0)
                pts_cal.push_back(pts);
        }
        else if (key == KEY_REMOVE_PTS) {
            if (pts_cal.size() > 0)
                pts_cal.pop_back();
        }
        else if (key == KEY_CLEAR_PTS) {
            pts_cal.clear();
        }
        else if (key == KEY_CALIBRATE) {
            if (pts_cal.size() > 1) {
                // Run calibration routine
                std::cout << "\nComputing camera intrinsic parameters...\n";
                pts_obj.resize(pts_cal.size(),pts_obj[0]);
                cv::calibrateCameraRO(pts_obj, pts_cal, imgl.size(), iFixedPoint, camera_matrix, dist_coeffs, rvecs, tvecs, cv::noArray());
                std::cout << "\nCamera Matrix = \n" << camera_matrix << std::endl;
                std::cout << "\nDistortion Coefficients = \n" << dist_coeffs << std::endl;
                calibrating = false;

                // Save result to file (overwrites same filename)
                fs.open(filename, cv::FileStorage::WRITE);
                fs << "camera_matrix" << camera_matrix;
                fs << "dist_coeffs" << dist_coeffs;
                fs.release();
            }
        }
        else if (key == KEY_RESTART) {
            if (!calibrating) {
                calibrating = true;
                pts_cal.clear();
                camera_matrix.release();
                dist_coeffs.release();
                rvecs.clear();
                tvecs.clear();
            }
        }
    }

    return 0;
}