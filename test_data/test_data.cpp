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



//#include <opencv2/features2d/features2d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/xfeatures2d/cuda.hpp>



#include <cstdio>

#include <string>
#include <vector>
#include <iostream>
using namespace std;
using namespace cv;
using namespace cv::cuda;
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

    printf("\nbounding:\n");
    for (int i=0; i<2; ++i)
    {
        for (int j=0; j<3; ++j)
        {
            printf("%f ", example->bounding(i, j));
        }
        printf("\n");
    }

    printf("\ncamera:\n");
    for (int i=0; i<3; ++i)
    {
        for (int j=0; j<3; ++j)
        {
            printf("%f ", example->camera.mat1(i, j));
        }
        printf("\n");
    }
    printf("\n");

    for (int i=0; i<3; ++i)
    {
        printf("%f ", example->camera.mat2[i]);
    }
    printf("\n\n");

    for (int i=0; i<3; ++i)
    {
        for (int j=0; j<3; ++j)
        {
            printf("%f ", example->camera.mat3(i, j));
        }
        printf("\n");
    }
    printf("\n");

    for (int i=0; i<3; ++i)
    {
        printf("%f ", example->camera.mat4[i]);
    }
    printf("\n\n");

    printf("image width: %u, height: %u\n", example->camera.imageWidth, example->camera.imageHeight);

    printf("\nP:\n");
    for (int i=0; i<3; ++i)
    {
        for (int j=0; j<4; ++j)
        {
            printf("%f ", example->p(i, j));
        }
        printf("\n");
    }




/*cv::Ptr<cv::DescriptorExtractor>deEx=cv::DescriptorCalculator::create("SIFT");

std::cout << "before computing, feats size " << keypoints.size() << std::endl;
// code to print out 10 features

cv::Mat desc;
deEx->compute(input, keypoints, desc);

std::cout << "after computing, feats size " << keypoints.size() << std::endl;
*/
  for(int i=0;i<dataset->getTrain().size();i++)
{
    MSM_epflObj *example1 = static_cast<MSM_epflObj *>(dataset->getTrain()[i].get());
  cv::Mat im1 = cv::imread(path+"png/"+example1->imageName.c_str(), IMREAD_GRAYSCALE);

  cv::Mat scaled;
  cv::resize(im1, scaled, cv::Size(), 0.3, 0.3);
  cout << "im1 size: " << im1.cols << "x" << im1.rows << std::endl;

    // vector<KeyPoint> kpts1, kpts2;
    // Mat desc1, desc2;
    // Ptr<SIFT> akaze = SIFT::create();
    // akaze->detectAndCompute(im1, noArray(), kpts1, desc1);
    // drawKeypoints(scaled, kpts1, scaled);



//cv::Ptr<cv::FeatureDetector> dect = cv::FeatureDetector::create("MSER");

//std::vector<cv::KeyPoint> keypoints;
//dect->detect(im1, keypoints);


    cv::cuda::printShortCudaDeviceInfo(cv::cuda::getDevice());

    SURF_CUDA surf(10);
    GpuMat keypoints1GPU, keypoints2GPU;
    GpuMat descriptors1GPU, descriptors2GPU;
    GpuMat img1;
    img1.upload(scaled);
    surf(img1, GpuMat(), keypoints1GPU, descriptors1GPU);
    vector<KeyPoint> keypoints1, keypoints2;
    vector<float> descriptors1, descriptors2;
    surf.downloadKeypoints(keypoints1GPU, keypoints1);

    drawKeypoints(scaled, keypoints1, scaled);




  cv::imshow("input (scaled)", scaled);
//  //cv::imshow("result", im1);
  cv::waitKey();
}
    return 0;
}
