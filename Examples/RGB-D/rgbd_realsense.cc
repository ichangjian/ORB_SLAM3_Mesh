/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <filesystem>
#include <opencv2/core/core.hpp>

#include <System.h>

using namespace std;

void LoadImages(const string &strPathSequence, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if (argc != 4)
    {
        cerr << endl
             << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strPathSequence = string(argv[3]);
    LoadImages(strPathSequence, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if (vstrImageFilenamesRGB.empty())
    {
        cerr << endl
             << "No images found in provided path." << endl;
        return 1;
    }
    else if (vstrImageFilenamesD.size() != vstrImageFilenamesRGB.size())
    {
        cerr << endl
             << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, true);
    float imageScale = SLAM.GetImageScale();

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl
         << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl
         << endl;

    // Main loop
    cv::Mat imRGB, imD;
    for (int ni = 0; ni < nImages; ni++)
    {
        // Read image and depthmap from file
        imRGB = cv::imread(vstrImageFilenamesRGB[ni], cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        imD = cv::imread(vstrImageFilenamesD[ni], cv::IMREAD_UNCHANGED);     //,cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];

        if (imRGB.empty())
        {
            cerr << endl
                 << "Failed to load image at: "
                 << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

        if (imageScale != 1.f)
        {
            int width = imRGB.cols * imageScale;
            int height = imRGB.rows * imageScale;
            cv::resize(imRGB, imRGB, cv::Size(width, height));
            cv::resize(imD, imD, cv::Size(width, height));
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB, imD, tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

        vTimesTrack[ni] = ttrack;

        // Wait to load the next frame
        double T = 0;
        if (ni < nImages - 1)
            T = vTimestamps[ni + 1] - tframe;
        else if (ni > 0)
            T = tframe - vTimestamps[ni - 1];

        // if (ttrack < T)
            usleep(5e5);
    }
    getchar();
    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for (int ni = 0; ni < nImages; ni++)
    {
        totaltime += vTimesTrack[ni];
    }
    cout << "-------" << endl
         << endl;
    cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
    cout << "mean tracking time: " << totaltime / nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

std::vector<std::string> GetFiles(const std::string &_path)
{
    std::vector<std::string> rgb_files;

    filesystem::path path(_path);
    if (exists(path))
    {
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
    return rgb_files;
}

void LoadImages(const string &strPathSequence, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    vstrImageFilenamesRGB = GetFiles(strPathSequence + "/rgb");
    vstrImageFilenamesD = GetFiles(strPathSequence + "/depth");
    for (auto line : vstrImageFilenamesRGB)
    {
        int index = line.find_last_of("/") + 1;
        std::string tm = line.substr(index, line.length() - index - 4);
        vTimestamps.push_back(std::stod(tm));
    }
}
