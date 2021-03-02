#include <iostream>
#include <opencv2/opencv.hpp> 

#define KEY_BREAK 27        // ESC closes the program
#define KEY_ADD_PTS 32      // SPACEBAR adds detected points to the calibration buffer
#define KEY_REMOVE_PTS 8    // BACKSPACE removes last added set of points from the calibration buffer
#define KEY_CLEAR_PTS 255   // DELETE clears all points from the calibration buffer
#define KEY_CALIBRATE 13    // RETURN runs the calibration routine on the collected points
#define KEY_RESTART 114     // R restarts the calibration collection process
static const cv::Scalar OPENCV_RED = cv::Scalar(0,0,255);

enum Pattern : int {
    CHECKERBOARD = 0,
    ASYMMETRIC_CIRCLES = 1
};

enum Calibration : int {
    MONO = 0,
    MONOL = 1,
    MONOR = 2,
    STEREO = 3
};

std::vector<cv::Point2f> getBoardCorners(cv::Mat &img, cv::Mat &img_gray, int downsample_factor, const Pattern &pattern, const cv::Size &board_size, int flags) {
    // Convert image to grayscale and refine the detected corners
    cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);

    // Blur to reduce noise
    cv::Mat img_blur;
    cv::medianBlur(img_gray, img_blur, 5);

    // Downsample the image for chessboard finding only
    cv::Mat img_downsampled;
    cv::resize(img_blur, img_downsampled, cv::Size(img.cols,img.rows)/downsample_factor);

    // Find chessboard pattern
    std::vector<cv::Point2f> pts;
    bool success;
    switch (pattern) {
        case Pattern::CHECKERBOARD:
            success = cv::findChessboardCorners(img_downsampled, board_size, pts, flags);
            break;
        case Pattern::ASYMMETRIC_CIRCLES:
            success = cv::findCirclesGrid(img_downsampled, board_size, pts, flags);
            break;
        default:
            std::cout << "Select a valid pattern:\n  0 - checkerboard\n  1 - asymmetric circles\n";
            break;
    }

    if (success) {
        // Scale points back to full resolution
        for (int i = 0; i < pts.size(); ++i)
            pts[i] *= downsample_factor;

        // Refine the detected corners for checkerboard patterns
        if (pattern == Pattern::CHECKERBOARD) {
            cv::cornerSubPix(img_blur, pts, cv::Size(31,31), cv::Size(-1,-1),
                                cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 30, 0.0001));
        }
        
        // Render the chessboard corners on the image
        cv::drawChessboardCorners(img, board_size, cv::Mat(pts), success);
    }

    return pts;
}

int main(int argc, char **argv)
{
    // Read command line inputs
    if (argc == 2 && strcmp(argv[1], "--help") == 0) {
        printf("\n**Camera Calibrator Help**\n\n");
        printf("Usage: ./camera_calibrator <video_device_number> <pattern_type> <calibration_type>\n");
        printf("\npattern_type:\n    0 - checkerboard\n    1 - asymmetric circle grid\n");
        printf("\ncalibration_type:\n    0 - monocular\n    1 - monocular left\n    2 - monocular right\n    3 - stereo\n");
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
    Calibration calib_type = (Calibration)std::strtol(argv[3], argv, 10);
    std::string filename;
    switch (calib_type) {
        case Calibration::MONO:
            filename = "cam_mono.yaml";
            break;
        case Calibration::MONOL:
            filename = "cam_left.yaml";
            break;
        case Calibration::MONOR:
            filename = "cam_right.yaml";
            break;
        case Calibration::STEREO:
            filename = "cam_stereo.yaml";
            break;
    }

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
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

    // Calibration properties and variables
    int flags;
    int downsample_factor;
    cv::Size board_size;
    float shape_separation;
    std::vector<std::vector<cv::Point3f>> pts_obj(1);
    switch (pattern) {
        case Pattern::CHECKERBOARD:
            flags = cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK;
            downsample_factor = 4;
            board_size = cv::Size(9,6);
            shape_separation = 0.0227;
            for(int i = 0; i < board_size.height; ++i)
                    for(int j = 0; j < board_size.width; ++j)
                        pts_obj[0].push_back(cv::Point3f(j*shape_separation, i*shape_separation, 0));
            break;
        case Pattern::ASYMMETRIC_CIRCLES:
            flags = cv::CALIB_CB_ASYMMETRIC_GRID + cv::CALIB_CB_CLUSTERING;
            downsample_factor = 1;
            board_size = cv::Size(4,11);
            shape_separation = 0.020;
            for(int i = 0; i < board_size.height; ++i)
                    for(int j = 0; j < board_size.width; ++j)
                        pts_obj[0].push_back(cv::Point3f((2*j + i%2)*shape_separation, i*shape_separation, 0));
            break;
        default:
            std::cout << "Select a valid pattern:\n  0 - checkerboard\n  1 - asymmetric circles\n";
            break;
    }

    std::vector<std::vector<cv::Point2f>> ptsl_cal;
    std::vector<std::vector<cv::Point2f>> ptsr_cal;
    int iFixedPoint = board_size.width - 1;
    cv::Mat Kl;
    cv::Mat Kr;
    cv::Mat Dl;
    cv::Mat Dr;
    cv::Mat R_lr;
    cv::Mat T_rlr;
    cv::Mat Emat;
    cv::Mat Fmat;
    std::vector<cv::Mat> rvecsl;
    std::vector<cv::Mat> rvecsr;
    std::vector<cv::Mat> tvecsl;
    std::vector<cv::Mat> tvecsr;
    bool calibrating = true;
    cv::Mat imgl_undistorted;
    cv::Mat imgr_undistorted;
    cv::Mat imgl_diff;
    cv::Mat imgr_diff;
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

    // Load intrinsic parameters for stereo calibration
    if (calib_type == Calibration::STEREO) {
        fs = cv::FileStorage("cam_left.yaml", cv::FileStorage::READ);
        fs["Kl"] >> Kl;
        fs["Dl"] >> Dl;
        fs = cv::FileStorage("cam_right.yaml", cv::FileStorage::READ);
        fs["Kr"] >> Kr;
        fs["Dr"] >> Dr;
    }

    // Capture images
    while(1)
    {
        // Attempt to read the next image
        cv::Mat img;
        if (!cap.read(img)) {
            printf("Error reading image from %s\n", device.c_str());
            break;
        }

        // Stereo camera needs image split, while pure mono camera does not
        cv::Mat imgl;
        cv::Mat imgr;
        if (calib_type == Calibration::MONO) {
            imgl = img;
        }
        else {
            // Split the concatenated stereo image into left and right images
            imgl = img(cv::Rect(0, 0, width, height));
            imgr = img(cv::Rect(width, 0, width, height));
        }

        // Find chessboard pattern
        cv::Mat imgl_gray;
        cv::Mat imgr_gray;
        std::vector<cv::Point2f> ptsl;
        std::vector<cv::Point2f> ptsr;
        ptsl = getBoardCorners(imgl, imgl_gray, downsample_factor, pattern, board_size, flags);
        if (calib_type == Calibration::MONOR || calib_type == Calibration::STEREO)
            ptsr = getBoardCorners(imgr, imgr_gray, downsample_factor, pattern, board_size, flags);

        // Draw while collecting calibration points
        if (calibrating) {
            // Draw number of calibration points collected on the image
            cv::putText(imgl, "Cal size: " + std::to_string(ptsl_cal.size()), cv::Point(10,30), cv::FONT_HERSHEY_SIMPLEX, 1, OPENCV_RED, 2, cv::LINE_AA);
            if (calib_type == Calibration::MONOR || calib_type == Calibration::STEREO)
                cv::putText(imgr, "Cal size: " + std::to_string(ptsr_cal.size()), cv::Point(10,30), cv::FONT_HERSHEY_SIMPLEX, 1, OPENCV_RED, 2, cv::LINE_AA);

            // Display the calibration image
            if (calib_type == Calibration::MONO || calib_type == Calibration::MONOL)
                cv::imshow("Calibration Image", imgl);
            else if (calib_type == Calibration::MONOR)
                cv::imshow("Calibration Image", imgr);
            else if (calib_type == Calibration::STEREO) {
                cv::Mat imgs;
                cv::hconcat(imgl, imgr, imgs);
                cv::imshow("Calibration Image", imgs);
            }
        }
        else {
            // Show difference between regular and undistored images
            if (calib_type != Calibration::STEREO) {
                if (!Kl.empty()) {
                    cv::undistort(imgl_gray, imgl_undistorted, Kl, Dl);
                    cv::absdiff(imgl_gray, imgl_undistorted, imgl_diff);
                    cv::imshow("Calibration Image", imgl_diff);
                }
                if (!Kr.empty()) {
                    cv::undistort(imgr_gray, imgr_undistorted, Kr, Dr);
                    cv::absdiff(imgr_gray, imgr_undistorted, imgr_diff);
                    cv::imshow("Calibration Image", imgr_diff);
                }
            }
            else if (calib_type == Calibration::STEREO) {
                // Compute transforms between cameras
                cv::Mat Rl, Rr, Pl, Pr, Q;
                cv::stereoRectify(Kl, Dl, Kr, Dr, imgl.size(), R_lr, T_rlr, Rl, Rr, Pl, Pr, Q);

                // Create maps for rectified images
                cv::Mat lmap1, lmap2, rmap1, rmap2;
                cv::initUndistortRectifyMap(Kl, Dl, Rl, Pl, imgl.size(), CV_32F, lmap1, lmap2);
                cv::initUndistortRectifyMap(Kr, Dr, Rr, Pr, imgr.size(), CV_32F, rmap1, rmap2);

                // Create rectified images
                cv::Mat imgl_rect, imgr_rect;
                cv::remap(imgl_gray, imgl_rect, lmap1, lmap2, cv::INTER_LINEAR);
                cv::remap(imgr_gray, imgr_rect, rmap1, rmap2, cv::INTER_LINEAR);

                // Stereo matching
                cv::Mat img_lines;
                cv::hconcat(imgl_rect, imgr_rect, img_lines);
                cv::cvtColor(img_lines, img_lines, cv::COLOR_GRAY2BGR);
                for(int j = img_lines.rows/20; j < img_lines.rows; j += img_lines.rows/20 )
                    cv::line(img_lines, cv::Point(0, j), cv::Point(img_lines.cols, j), cv::Scalar(0, 0, 255), 1, 8);
                cv::imshow("Calibration Image", img_lines);
            }
        }

        // Break on 'esc' key
        int key = cv::waitKey(10);
        if(key == KEY_BREAK) {
            break;
        }
        else if (key == KEY_ADD_PTS) {
            std::cout << ptsr.size() << std::endl;
            if (calib_type == Calibration::MONO || calib_type == Calibration::MONOL) {
                if (ptsl.size() == board_size.width*board_size.height)
                    ptsl_cal.push_back(ptsl);
            }
            else if (calib_type == Calibration::MONOR) {
                if (ptsr.size() == board_size.width*board_size.height)
                    ptsr_cal.push_back(ptsr);
            }
            else if (calib_type == Calibration::STEREO) {
                if (ptsl.size() == board_size.width*board_size.height && ptsr.size() == board_size.width*board_size.height) {
                    ptsl_cal.push_back(ptsl);
                    ptsr_cal.push_back(ptsr);
                }
            }
        }
        else if (key == KEY_REMOVE_PTS) {
            if (ptsl_cal.size() > 0)
                ptsl_cal.pop_back();
            if (ptsr_cal.size() > 0)
                ptsr_cal.pop_back();
        }
        else if (key == KEY_CLEAR_PTS) {
            ptsl_cal.clear();
            ptsr_cal.clear();
        }
        else if (key == KEY_CALIBRATE) {
            if (calib_type == Calibration::MONO || calib_type == Calibration::MONOL) {
                if (ptsl_cal.size() > 1) {
                    // Run calibration routine
                    if (calib_type == Calibration::MONO)
                        std::cout << "\nComputing camera intrinsic parameters...\n";
                    else
                        std::cout << "\nComputing left camera intrinsic parameters...\n";
                    pts_obj.resize(ptsl_cal.size(),pts_obj[0]);
                    cv::calibrateCameraRO(pts_obj, ptsl_cal, imgl.size(), iFixedPoint, Kl, Dl, rvecsl, tvecsl, cv::noArray());
                    if (calib_type == Calibration::MONO) {
                        std::cout << "\nK = \n" << Kl << std::endl;
                        std::cout << "\nD = \n" << Dl << "\n\n";
                    }
                    else {
                        std::cout << "\nKl = \n" << Kl << std::endl;
                        std::cout << "\nDl = \n" << Dl << "\n\n";
                    }
                    calibrating = false;

                    // Save result to file (overwrites same filename)
                    fs.open(filename, cv::FileStorage::WRITE);
                    if (calib_type == Calibration::MONO) {
                        fs << "K" << Kl;
                        fs << "D" << Dl;
                    }
                    else {
                        fs << "Kl" << Kl;
                        fs << "Dl" << Dl;
                    }
                    fs.release();
                }
            }
            else if (calib_type == Calibration::MONOR) {
                if (ptsr_cal.size() > 1) {
                    // Run calibration routine
                    std::cout << "\nComputing right camera intrinsic parameters...\n";
                    pts_obj.resize(ptsr_cal.size(),pts_obj[0]);
                    cv::calibrateCameraRO(pts_obj, ptsr_cal, imgr.size(), iFixedPoint, Kr, Dr, rvecsr, tvecsr, cv::noArray());
                    std::cout << "\nKr = \n" << Kr << std::endl;
                    std::cout << "\nDr = \n" << Dr << "\n\n";
                    calibrating = false;

                    // Save result to file (overwrites same filename)
                    fs.open(filename, cv::FileStorage::WRITE);
                    fs << "Kr" << Kr;
                    fs << "Dr" << Dr;
                    fs.release();
                }
            }
            else if (calib_type == Calibration::STEREO){
                if (ptsl_cal.size() > 1 && ptsr_cal.size() > 1) {
                    // Ensure proper container sizes
                    if (ptsl_cal.size() != ptsr_cal.size()) {
                        std::cout << "Left and right calibration point containers are not the same size!\n";
                        return -1;
                    }

                    // Run calibration routine
                    std::cout << "\nComputing stereo camera extrinsic parameters...\n";
                    pts_obj.resize(ptsr_cal.size(),pts_obj[0]);
                    cv::stereoCalibrate(pts_obj, ptsl_cal, ptsr_cal, Kl, Dl, Kr, Dr, imgl.size(), R_lr, T_rlr, Emat, Fmat, cv::CALIB_FIX_INTRINSIC);
                    std::cout << "\nR = \n" << R_lr << std::endl;
                    std::cout << "\nT = \n" << T_rlr << std::endl;
                    std::cout << "\nE = \n" << Emat << std::endl;
                    std::cout << "\nF = \n" << Fmat << std::endl;
                    calibrating = false;

                    // Save result to file (overwrites same filename)
                    fs.open(filename, cv::FileStorage::WRITE);
                    fs << "R" << R_lr;
                    fs << "T" << T_rlr;
                    fs << "E" << Emat;
                    fs << "F" << Fmat;
                    fs.release();
                }
            }
        }
        else if (key == KEY_RESTART) {
            if (!calibrating) {
                calibrating = true;
                ptsl_cal.clear();
                ptsr_cal.clear();
                Kl.release();
                Kr.release();
                Dl.release();
                Dr.release();
                R_lr.release();
                T_rlr.release();
                Emat.release();
                Fmat.release();
                rvecsl.clear();
                rvecsr.clear();
                tvecsl.clear();
                tvecsr.clear();
            }
        }
    }

    return 0;
}