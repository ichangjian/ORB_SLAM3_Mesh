#include <signal.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <filesystem>

#include <opencv2/opencv.hpp>

#include <librealsense2/rs.hpp>
#include "librealsense2/rsutil.h"

using namespace std;

bool b_continue_session;

void exit_loop_handler(int s)
{
    cout << "Finishing session" << endl;
    b_continue_session = false;
}

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile> &streams);
bool profile_changed(const std::vector<rs2::stream_profile> &current, const std::vector<rs2::stream_profile> &prev);

void interpolateData(const std::vector<double> &vBase_times,
                     std::vector<rs2_vector> &vInterp_data, std::vector<double> &vInterp_times,
                     const rs2_vector &prev_data, const double &prev_time);

rs2_vector interpolateMeasure(const double target_time,
                              const rs2_vector current_data, const double current_time,
                              const rs2_vector prev_data, const double prev_time);

static rs2_option get_sensor_option(const rs2::sensor &sensor)
{
    // Sensors usually have several options to control their properties
    //  such as Exposure, Brightness etc.

    std::cout << "Sensor supports the following options:\n"
              << std::endl;

    // The following loop shows how to iterate over all available options
    // Starting from 0 until RS2_OPTION_COUNT (exclusive)
    for (int i = 0; i < static_cast<int>(RS2_OPTION_COUNT); i++)
    {
        rs2_option option_type = static_cast<rs2_option>(i);
        // SDK enum types can be streamed to get a string that represents them
        std::cout << "  " << i << ": " << option_type;

        // To control an option, use the following api:

        // First, verify that the sensor actually supports this option
        if (sensor.supports(option_type))
        {
            std::cout << std::endl;

            // Get a human readable description of the option
            const char *description = sensor.get_option_description(option_type);
            std::cout << "       Description   : " << description << std::endl;

            // Get the current value of the option
            float current_value = sensor.get_option(option_type);
            std::cout << "       Current Value : " << current_value << std::endl;

            // To change the value of an option, please follow the change_sensor_option() function
        }
        else
        {
            std::cout << " is not supported" << std::endl;
        }
    }

    uint32_t selected_sensor_option = 0;
    return static_cast<rs2_option>(selected_sensor_option);
}
std::queue<cv::Mat> rgbs;
std::queue<cv::Mat> depths;
std::queue<double> tmsts;
int save_num = 0;
std::atomic<bool> flag_exit = {false};
std::string save_path = "./";
void save_data()
{
    while (true)
    {

        if (tmsts.size() == 0 && flag_exit == true)
        {
            break;
        }
        else if (tmsts.size() > 0)
        {
            cv::Mat rgb = rgbs.front();
            cv::Mat depth = depths.front();
            double tmst = tmsts.front();
            cv::imwrite(save_path + "/rgb/" + std::to_string(tmst) + ".png", rgb);
            cv::imwrite(save_path + "/depth/" + std::to_string(tmst) + ".png", depth);
            save_num++;
            rgbs.pop();
            depths.pop();
            tmsts.pop();
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    return;
}

void save_camera(std::string _save_path, int _width, int _height, int _fps, std::vector<double> _intri, std::vector<double> _dist)
{
    std::ofstream file(_save_path + "/camera.txt");
    std::string head = "\nCamera.type: \"PinHole\"\n\n# Camera resolution\nCamera.width: " +
                       std::to_string(_width) + "\nCamera.height: " + std::to_string(_height) +
                       "\n# Camera frames per second\nCamera.fps: " + std::to_string(_fps) + "\n";
    std::string intri = "\n# Right Camera calibration and distortion parameters (OpenCV)\nCamera1.fx : " +
                        std::to_string(_intri[0]) + "\nCamera1.fy : " +
                        std::to_string(_intri[1]) + "\nCamera1.cx : " +
                        std::to_string(_intri[2]) + "\nCamera1.cy : " +
                        std::to_string(_intri[3]) + "\n";
    std::string dist = "\n# distortion parameters\nCamera1.k1 : " +
                       std::to_string(_dist[0]) + "\nCamera1.k2 : " +
                       std::to_string(_dist[1]) + "\nCamera1.p1 : " +
                       std::to_string(_dist[2]) + "\nCamera1.p2 : " +
                       std::to_string(_dist[3]) + "\nCamera1.k3 : " +
                       std::to_string(_dist[4]) + "\n";
    file << head;
    file << intri;
    file << dist;
    file.close();
}
bool CheckDir(const std::string &_path)
{
    if (!std::filesystem::exists(_path))
    {
        std::cout << _path << " isnt exist.\n";
        return false;
    }
    if (std::filesystem::create_directories(_path + "/rgb") && std::filesystem::create_directories(_path + "/depth"))
    {
        return true;
    }
    else
    {
        std::cout << _path << " cant create rgb/depth.\n";
        return false;
    }
    return 0;
}

void test()
{
    filesystem::path path("./a/rgb");
    std::vector<std::string> rgb_files;
    {
        if (!exists(path))
        { // 目录不存在直接返回
            return;
        }
        auto begin = filesystem::recursive_directory_iterator(path); // 获取文件系统迭代器
        auto end = filesystem::recursive_directory_iterator();       // end迭代器
        for (auto it = begin; it != end; it++)
        {
            const string spacer(it.depth() * 2, ' '); // 这个是用来排版的空格
            auto &entry = *it;
            if (filesystem::is_regular_file(entry))
            {
                // 文件
                cout << spacer << "File:" << entry.path();
                rgb_files.push_back(entry.path().string());
                cout << " (" << filesystem::file_size(entry) << " bytes )" << endl;
            }
            else if (filesystem::is_directory(entry))
            {
                cout << spacer << "Dir:" << entry.path() << endl;
            }
        }
    }
    sort(rgb_files.begin(), rgb_files.end());
    for (auto line : rgb_files)
    {
        std::cout << line << "\n";
        int index1 = line.find_last_of("/") + 1;
        int index2 = line.find_last_of(".png");
        std::string tm = line.substr(index1, line.length() - index1 - 4);

        std::cout << std::stod(tm) << "\n";
    }
}
int main(int argc, char **argv)
{
    std::cout << "version 0.2\n";
    if (argc != 2)
    {
        cerr << endl
             << "Usage: ./SVRS path_to_save"
             << endl;
        return -1;
    }
    save_path = std::string(argv[1]);
    if (!CheckDir(save_path))
        return -1;
    string file_name;
    bool bFileName = false;

    if (argc == 4)
    {
        file_name = string(argv[argc - 1]);
        bFileName = true;
    }

    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
    b_continue_session = true;

    double offset = 0; // ms

    rs2::context ctx;
    rs2::device_list devices = ctx.query_devices();
    rs2::device selected_device;
    if (devices.size() == 0)
    {
        std::cerr << "No device connected, please connect a RealSense device" << std::endl;
        return 0;
    }
    else
        selected_device = devices[0];
    std::cout << "devices.size()" << devices.size() << "\n";
    std::vector<rs2::sensor> sensors = selected_device.query_sensors();
    int index = 0;
    std::cout << "sensors.size()" << sensors.size() << "\n";
    // We can now iterate the sensors and print their names
    // for (rs2::sensor sensor : sensors)
    {
        //     std::cout << index << "index sensor\n";
        //     if (sensor.supports(RS2_CAMERA_INFO_NAME))
        //     {
        // ++index;
        //         if (index == 1)
        //         {
        //             std::cout<<index<<"A\n";
        //             sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
        //             std::cout<<index<<"A\n";
        //             // sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_LIMIT, 50000);
        //             std::cout<<index<<"A\n";
        //             sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1); // emitter on for depth information
        //             std::cout<<index<<"A\n";
        //         }
        // std::cout << "===============================================\n";
        // std::cout << "               " << index << " : " << sensor.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
        // std::cout << "===============================================\n";
        // get_sensor_option(sensor);
        //         if (index == 2)
        //         {
        //             // RGB camera
        //             sensor.set_option(RS2_OPTION_EXPOSURE, 80.f);
        //         }

        //         if (index == 3)
        //         {
        // sensor.set_option(RS2_OPTION_ENABLE_MOTION_CORRECTION, 0);
        //         }
        //     }
    }
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // return 0;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    // RGB stream
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);

    // Depth stream
    // cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    // IMU stream
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F); //, 250); // 63
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);  //, 400);

    // IMU callback
    std::mutex imu_mutex;
    std::condition_variable cond_image_rec;

    vector<double> v_accel_timestamp;
    vector<rs2_vector> v_accel_data;
    vector<double> v_gyro_timestamp;
    vector<rs2_vector> v_gyro_data;

    double prev_accel_timestamp = 0;
    rs2_vector prev_accel_data;
    double current_accel_timestamp = 0;
    rs2_vector current_accel_data;
    vector<double> v_accel_timestamp_sync;
    vector<rs2_vector> v_accel_data_sync;

    cv::Mat imCV, depthCV;
    int width_img, height_img;
    double timestamp_image = -1.0;
    bool image_ready = false;
    int count_im_buffer = 0; // count dropped frames

    // start and stop just to get necessary profile
    rs2::pipeline_profile pipe_profile = pipe.start(cfg);
    pipe.stop();

    // Align depth and RGB frames
    // Pipeline could choose a device that does not have a color stream
    // If there is no color stream, choose to align depth to another stream
    rs2_stream align_to = find_stream_to_align(pipe_profile.get_streams());

    // Create a rs2::align object.
    // rs2::align allows us to perform alignment of depth frames to others frames
    // The "align_to" is the stream type to which we plan to align depth frames.
    rs2::align align(align_to);
    rs2::frameset fsSLAM;

    auto imu_callback = [&](const rs2::frame &frame)
    {
        std::unique_lock<std::mutex> lock(imu_mutex);

        if (rs2::frameset fs = frame.as<rs2::frameset>())
        {
            count_im_buffer++;

            double new_timestamp_image = fs.get_timestamp() * 1e-3;
            if (abs(timestamp_image - new_timestamp_image) < 0.001)
            {
                count_im_buffer--;
                return;
            }

            if (profile_changed(pipe.get_active_profile().get_streams(), pipe_profile.get_streams()))
            {
                // If the profile was changed, update the align object, and also get the new device's depth scale
                pipe_profile = pipe.get_active_profile();
                align_to = find_stream_to_align(pipe_profile.get_streams());
                align = rs2::align(align_to);
            }

            // Align depth and rgb takes long time, move it out of the interruption to avoid losing IMU measurements
            fsSLAM = fs;

            /*
            //Get processed aligned frame
            auto processed = align.process(fs);


            // Trying to get both other and aligned depth frames
            rs2::video_frame color_frame = processed.first(align_to);
            rs2::depth_frame depth_frame = processed.get_depth_frame();
            //If one of them is unavailable, continue iteration
            if (!depth_frame || !color_frame) {
                cout << "Not synchronized depth and image\n";
                return;
            }


            imCV = cv::Mat(cv::Size(width_img, height_img), CV_8UC3, (void*)(color_frame.get_data()), cv::Mat::AUTO_STEP);
            depthCV = cv::Mat(cv::Size(width_img, height_img), CV_16U, (void*)(depth_frame.get_data()), cv::Mat::AUTO_STEP);

            cv::Mat depthCV_8U;
            depthCV.convertTo(depthCV_8U,CV_8U,0.01);
            cv::imshow("depth image", depthCV_8U);*/

            timestamp_image = fs.get_timestamp() * 1e-3;
            image_ready = true;

            while (v_gyro_timestamp.size() > v_accel_timestamp_sync.size())
            {
                int index = v_accel_timestamp_sync.size();
                double target_time = v_gyro_timestamp[index];

                v_accel_data_sync.push_back(current_accel_data);
                v_accel_timestamp_sync.push_back(target_time);
            }

            lock.unlock();
            cond_image_rec.notify_all();
        }
    };

    pipe_profile = pipe.start(cfg, imu_callback);

    rs2::stream_profile cam_stream = pipe_profile.get_stream(RS2_STREAM_COLOR);

    rs2_intrinsics intrinsics_cam = cam_stream.as<rs2::video_stream_profile>().get_intrinsics();
    width_img = intrinsics_cam.width;
    height_img = intrinsics_cam.height;
    std::cout << " fx = " << intrinsics_cam.fx << std::endl;
    std::cout << " fy = " << intrinsics_cam.fy << std::endl;
    std::cout << " cx = " << intrinsics_cam.ppx << std::endl;
    std::cout << " cy = " << intrinsics_cam.ppy << std::endl;
    std::cout << " height = " << intrinsics_cam.height << std::endl;
    std::cout << " width = " << intrinsics_cam.width << std::endl;
    std::cout << " Coeff = " << intrinsics_cam.coeffs[0] << ", " << intrinsics_cam.coeffs[1] << ", " << intrinsics_cam.coeffs[2] << ", " << intrinsics_cam.coeffs[3] << ", " << intrinsics_cam.coeffs[4] << ", " << std::endl;
    std::cout << " Model = " << intrinsics_cam.model << std::endl;
    std::vector<double> intri;
    intri.push_back(intrinsics_cam.fx);
    intri.push_back(intrinsics_cam.fy);
    intri.push_back(intrinsics_cam.ppx);
    intri.push_back(intrinsics_cam.ppy);
    std::vector<double> dist;
    dist.push_back(intrinsics_cam.coeffs[0]);
    dist.push_back(intrinsics_cam.coeffs[1]);
    dist.push_back(intrinsics_cam.coeffs[2]);
    dist.push_back(intrinsics_cam.coeffs[3]);
    dist.push_back(intrinsics_cam.coeffs[4]);
    save_camera(save_path, intrinsics_cam.width, intrinsics_cam.height, 30, intri, dist);
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD, true, 0, file_name);
    // float imageScale = SLAM.GetImageScale();

    double timestamp;
    cv::Mat im, depth;

    double t_resize = 0.f;
    double t_track = 0.f;
    rs2::frameset fs;
    std::thread save_thread(&save_data);
    bool flag_start_save = false;
    while (true)
    {

        std::chrono::steady_clock::time_point time_Start_Process = std::chrono::steady_clock::now();
        {
            std::unique_lock<std::mutex> lk(imu_mutex);
            if (!image_ready)
                cond_image_rec.wait(lk);

            fs = fsSLAM;

            if (count_im_buffer > 1)
                std::cout << count_im_buffer - 1 << " dropped frs\n";
            count_im_buffer = 0;

            timestamp = timestamp_image;
            im = imCV.clone();
            depth = depthCV.clone();
            image_ready = false;
        }

        std::chrono::steady_clock::time_point time_End_Process = std::chrono::steady_clock::now();
        // std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(time_End_Process - time_Start_Process).count() << "fps\n";
        // Perform alignment here
        auto processed = align.process(fs);

        // Trying to get both other and aligned depth frames
        rs2::video_frame color_frame = processed.first(align_to);
        rs2::depth_frame depth_frame = processed.get_depth_frame();

        im = cv::Mat(cv::Size(width_img, height_img), CV_8UC3, (void *)(color_frame.get_data()), cv::Mat::AUTO_STEP);
        depth = cv::Mat(cv::Size(width_img, height_img), CV_16U, (void *)(depth_frame.get_data()), cv::Mat::AUTO_STEP);
        cv::Mat rgb;
        std::vector<cv::Mat> BGR;
        std::vector<cv::Mat> RGB;
        RGB.resize(3);
        cv::split(im, BGR);
        RGB[0] = BGR[2];
        RGB[1] = BGR[1];
        RGB[2] = BGR[0];
        cv::merge(RGB, rgb);
        // im.convertTo(rgb, cv::COLOR_BGR2RGB);
        // std::cout << rgb.size() << rgb.type() << "rgb\n";
        // std::cout << im.size() << im.type() << "im\n";
        // std::cout << std::to_string(timestamp) << "\n";
        cv::Mat depthCV_8U;
        depth.convertTo(depthCV_8U, CV_8UC1, 0.01);
        cv::Mat frame;
        frame.create(height_img, width_img + width_img, CV_8UC3); // 创建一个大的矩阵用于存放两张图像
        // // 将第一张图和第二张图摆放在合适的位置
        cv::Mat roi1 = frame(cv::Rect(0, 0, width_img, height_img));
        rgb.copyTo(roi1);
        cv::Mat roi2 = frame(cv::Rect(width_img, 0, width_img, height_img));
        // depthCV_8U.convertTo(depthCV_8U, cv::COLOR_GRAY2BGR);
        RGB[0] = depthCV_8U;
        RGB[1] = depthCV_8U;
        RGB[2] = depthCV_8U;
        merge(RGB, depthCV_8U);
        depthCV_8U.copyTo(roi2);
        if (flag_start_save)
        {
            cv::putText(frame, "Press Esc to exit.", cv::Point(width_img / 5, height_img / 5), 1, 2, cv::Scalar(255, 0, 0));
        }
        else
        {

            cv::putText(frame, "Press s to save. Press Esc to exit.", cv::Point(width_img / 5, height_img / 5), 1, 2, cv::Scalar(255, 0, 0));
        }
        if (flag_start_save)
        {
            rgbs.push(rgb.clone());
            depths.push(depth.clone());
            tmsts.push(timestamp);
        }
        cv::imshow("frame", frame);
        int s = cv::waitKey(10);
        if (s == 27)
        {
            flag_exit = true;
            std::cout << "wait " << tmsts.size() << "\n";
            if (save_thread.joinable())
                save_thread.join();
            std::cout << "save " << save_num << " picture.\n";
            return 0;
        }
        else if (s == 115)
        {
            flag_start_save = true;
            std::cout << "saving data...\n";
        }
    }
    std::cout << "System shutdown!\n";
}

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile> &streams)
{
    // Given a vector of streams, we try to find a depth stream and another stream to align depth with.
    // We prioritize color streams to make the view look better.
    // If color is not available, we take another stream that (other than depth)
    rs2_stream align_to = RS2_STREAM_ANY;
    bool depth_stream_found = false;
    bool color_stream_found = false;
    for (rs2::stream_profile sp : streams)
    {
        rs2_stream profile_stream = sp.stream_type();
        if (profile_stream != RS2_STREAM_DEPTH)
        {
            if (!color_stream_found) // Prefer color
                align_to = profile_stream;

            if (profile_stream == RS2_STREAM_COLOR)
            {
                color_stream_found = true;
            }
        }
        else
        {
            depth_stream_found = true;
        }
    }

    if (!depth_stream_found)
        throw std::runtime_error("No Depth stream available");

    if (align_to == RS2_STREAM_ANY)
        throw std::runtime_error("No stream found to align with Depth");

    return align_to;
}

bool profile_changed(const std::vector<rs2::stream_profile> &current, const std::vector<rs2::stream_profile> &prev)
{
    for (auto &&sp : prev)
    {
        // If previous profile is in current (maybe just added another)
        auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile &current_sp)
                                { return sp.unique_id() == current_sp.unique_id(); });
        if (itr == std::end(current)) // If it previous stream wasn't found in current
        {
            return true;
        }
    }
    return false;
}