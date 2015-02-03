/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2014, Itseez Inc, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Itseez Inc or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#include "opencv2/datasets/msm_epfl.hpp"

#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>



#include <opencv2/features2d/features2d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/xfeatures2d/cuda.hpp>


#include <opencv2/xfeatures2d/nonfree.hpp>


#include <cstdio>

#include <string>
#include <vector>
#include <iostream>
using namespace std;
using namespace cv;
using namespace cv::cuda;
using namespace cv::xfeatures2d;
using namespace cv::datasets;

int main(int argc, char *argv[])
{
    string path = "/home/nicolas/code/data/fountain/";
    Ptr<MSM_epfl> dataset = MSM_epfl::create();
    dataset->load(path);

    // ***************
    // dataset contains all information for each image.
    // For example, let output dataset size and first object.
    printf("dataset size: %u\n", (unsigned int)dataset->getTrain().size());
    MSM_epflObj *example = static_cast<MSM_epflObj *>(dataset->getTrain()[0].get());
    printf("first image:\nname: %s\n", example->imageName.c_str());

  

for(int i=0;i<dataset->getTrain().size();i++)
{
    MSM_epflObj *example1 = static_cast<MSM_epflObj *>(dataset->getTrain()[i].get());
    cv::Mat im1 = cv::imread(path+"png/"+example1->imageName.c_str(), IMREAD_GRAYSCALE);

    cv::Mat scaled;
    cv::resize(im1, scaled, cv::Size(), 0.3, 0.3);
    cout << "im1 size: " << im1.cols << "x" << im1.rows << std::endl;


    cv::cuda::printShortCudaDeviceInfo(cv::cuda::getDevice());

    // SiftDescriptorExtractor extractor;
    // SiftDescriptorExtractor detector;  
    //Ptr<SIFT_Impl> pt;// featureDetector = FeatureDetector::create("SIFT");
    vector<KeyPoint> keypoints1;
    Mat descriptors1, descriptors2;
    Ptr<SIFT> sift = SIFT::create(5100,3,.04,10.0,1.6);
    sift->detectAndCompute(scaled, Mat(), keypoints1, descriptors1,0);
    // SIFT_Impl sift(1,1,1.0,1.01.0);
    // vector<KeyPoint> keypoints1;
    // sift(scaled, scaled, keypoints1);
    
    // sift.downloadKeypoints(keypoints1GPU, keypoints1);
     drawKeypoints(scaled, keypoints1, scaled);




    cv::imshow("input (scaled)", scaled);
    cv::waitKey();
}
    return 0;
}
