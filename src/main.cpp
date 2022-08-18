/*!
 * @authors Ben Knight (bknight@i3drobotics.com)
 * @date 2021-05-26
 * @copyright Copyright (c) I3D Robotics Ltd, 2021
 * 
 * @file main.cpp
 * @brief Example application using I3DR Stereo Vision C++ API
 */

#include <iostream>
#include <phase/stereocamera/stereocamera.h>
#include <phase/stereomatcher/stereomatcher.h>
#include <phase/calib/stereocalibration.h>
#include <phase/utils.h>
#include <opencv2/highgui.hpp>

int main() {  
    bool license_valid = I3DR::Phase::StereoI3DRSGM::isLicenseValid();
    if (license_valid){
        std::cout << "I3DRSGM license accepted" << std::endl;
    } else {
        std::cout << "Missing or invalid I3DRSGM license" << std::endl;
    }

    std::string data_folder = "../../data";
    std::string left_yaml = data_folder + "/left.yaml";
    std::string right_yaml = data_folder + "/right.yaml";
    std::string left_image_file = data_folder + "/left.png";
    std::string right_image_file = data_folder + "/right.png";

    std::string left_serial = "0815-0000";
    std::string right_serial = "0815-0001";
    I3DR::Phase::CameraDeviceType dev_type = I3DR::Phase::CameraDeviceType::DEVICE_TYPE_GENERIC_PYLON; // DEVICE_TYPE_GENERIC_PYLON / DEVICE_TYPE_TITANIA
    bool use_test_images = true;
    bool enable_vis = false;
    int waitkey_delay = 1;
    float downsample_factor = 1.0f;

    I3DR::Phase::CameraInterfaceType interface_type;
    if (use_test_images){
        interface_type = I3DR::Phase::CameraInterfaceType::INTERFACE_TYPE_VIRTUAL;
    } else {
        interface_type = I3DR::Phase::CameraInterfaceType::INTERFACE_TYPE_USB;
    }

    I3DR::Phase::CameraDeviceInfo device_info = I3DR::Phase::CameraDeviceInfo(
        left_serial.c_str(), right_serial.c_str(), "virtual-camera",
        dev_type, interface_type
    );

    std::cout << "device info created" << std::endl;
    
    I3DR::Phase::StereoMatcherType matcher_type;
    if (license_valid){
        matcher_type = I3DR::Phase::StereoMatcherType::STEREO_MATCHER_I3DRSGM;
    } else {
        matcher_type = I3DR::Phase::StereoMatcherType::STEREO_MATCHER_BM;
    }
    I3DR::Phase::AbstractStereoCamera* cam = I3DR::Phase::createStereoCamera(device_info);
    I3DR::Phase::StereoCameraCalibration cal = I3DR::Phase::StereoCameraCalibration::calibrationFromYAML(left_yaml.c_str(), right_yaml.c_str());
    if (!cal.isValid()) {
        std::cerr << "Failed to load valid calibration" << std::endl;
        return 1;
    }
    I3DR::Phase::AbstractStereoMatcher* matcher = I3DR::Phase::createStereoMatcher(matcher_type);

    if (use_test_images) {
        cam->setTestImagePaths(
            left_image_file.c_str(),
            right_image_file.c_str()
        );
    }
    
    std::cout << "Connecting to camera..." << std::endl;
    bool ret = cam->connect();
    std::cout << "Camera connected: " << ret << std::endl;
    if (ret) {
        cam->setDownsampleFactor(downsample_factor);
        cam->startCapture();
        std::cout << "Running non-threaded camera capture..." << std::endl;
        cv::Mat left_img, right_img;
        for (int i = 0; i < 3; i++) {
            std::cout << "Waiting for result..." << std::endl;
            I3DR::Phase::CameraReadResult read_result = cam->read();
            if (read_result.valid) {
                std::cout << "Stereo read result received" << std::endl;
                std::cout << "Framerate: " << cam->getFrameRate() << std::endl;
                I3DR::Phase::StereoImagePair rect_pair = cal.rectify(read_result.left, read_result.right);
                I3DR::Phase::StereoMatcherComputeResult compute_result = matcher->compute(rect_pair.left, rect_pair.right);
                if (compute_result.valid){
                    cv::Mat disp_image_left = I3DR::Phase::scaleImage(read_result.left, 0.25);
                    cv::Mat disp_image_right = I3DR::Phase::scaleImage(read_result.right, 0.25);
                    cv::Mat norm_disp = I3DR::Phase::normaliseDisparity(compute_result.disparity);
                    cv::Mat disp_image_disparity = I3DR::Phase::scaleImage(norm_disp, 0.25);
                    if (enable_vis){
                        cv::imshow("left", disp_image_left);
                        cv::imshow("right", disp_image_right);
                        cv::imshow("disparity", disp_image_disparity);
                        cv::waitKey(1);
                    }
                } else {
                    std::cerr << "Failed to compute disparity" << std::endl;
                }
            }
            else {
                std::cerr << "Failed to read stereo result" << std::endl;
                break;
            }
        }
        cam->disconnect();
    }
    ret = cam->connect();
    if (ret) {
        cam->startCapture();
        std::cout << "Running split threaded camera capture..." << std::endl;
        for (int i = 0; i < 3; i++) {
            cam->startReadThread();
            std::cout << "Waiting for result..." << std::endl;
            while (cam->isReadThreadRunning()) {}
            I3DR::Phase::CameraReadResult cam_result = cam->getReadThreadResult();
            if (cam_result.valid) {
                cv::Mat left_rect_image, right_rect_image;
                cal.rectify(
                    cam_result.left,
                    cam_result.right,
                    left_rect_image,
                    right_rect_image);
                matcher->startComputeThread(left_rect_image, right_rect_image);
                std::cout << "Stereo result received" << std::endl;
                std::cout << "Framerate: " << cam->getFrameRate() << std::endl;
                cv::Mat disp_image_left = I3DR::Phase::scaleImage(left_rect_image, 0.25);
                cv::Mat disp_image_right = I3DR::Phase::scaleImage(right_rect_image, 0.25);
                if (enable_vis){
                    cv::imshow("left", disp_image_left);
                    cv::imshow("right", disp_image_right);
                    cv::waitKey(waitkey_delay);
                }
                std::cout << "Waiting for result..." << std::endl;
                while (matcher->isComputeThreadRunning()) {
                    //continue camera capture while compute is running
                    cam->startReadThread();
                    while (cam->isReadThreadRunning()) {}
                    cam_result = cam->getReadThreadResult();
                    if (cam_result.valid) {
                        cal.rectify(
                            cam_result.left,
                            cam_result.right,
                            left_rect_image,
                            right_rect_image);
                        disp_image_left = I3DR::Phase::scaleImage(left_rect_image, 0.25);
                        disp_image_right = I3DR::Phase::scaleImage(right_rect_image, 0.25);
                        if (enable_vis){
                            cv::imshow("left", disp_image_left);
                            cv::imshow("right", disp_image_right);
                            cv::waitKey(waitkey_delay);
                        }
                    }
                }
                I3DR::Phase::StereoMatcherComputeResult match_result = matcher->getComputeThreadResult();
                if (match_result.valid) {
                    std::cout << "Match result received" << std::endl;
                    cv::Mat disp_image_disparity = I3DR::Phase::scaleImage(
                        I3DR::Phase::normaliseDisparity(match_result.disparity), 0.25);
                    if (enable_vis){
                        cv::imshow("disparity", disp_image_disparity);
                        cv::waitKey(waitkey_delay);
                    }
                }
                else {
                    std::cout << "Failed to compute stereo match" << std::endl;
                    break;
                }
            }
            else {
                std::cout << "Failed to read stereo result" << std::endl;
                break;
            }
        }

        cam->disconnect();
        std::cout << "Camera disconnected" << std::endl;
    }
    return 0;
}