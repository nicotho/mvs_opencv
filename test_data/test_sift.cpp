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



  

for(int i=1;i<dataset->getTrain().size();i++)
{
    MSM_epflObj *example0 = static_cast<MSM_epflObj *>(dataset->getTrain()[i-1].get());
    MSM_epflObj *example1 = static_cast<MSM_epflObj *>(dataset->getTrain()[i].get());
    cv::Mat im0 = cv::imread(path+"png/"+example0->imageName.c_str(), IMREAD_GRAYSCALE);
    cv::Mat im1 = cv::imread(path+"png/"+example1->imageName.c_str(), IMREAD_GRAYSCALE);

    cv::Mat scaled;
    cv::resize(im0, im0, cv::Size(), 0.3, 0.3);
    cv::resize(im1, im1, cv::Size(), 0.3, 0.3);





    // detecting keypoints & computing descriptors
    // SiftDescriptorExtractor extractor;
    // SiftDescriptorExtractor detector;  
    //Ptr<SIFT_Impl> pt;// featureDetector = FeatureDetector::create("SIFT");
    vector<KeyPoint> keypoints1,keypoints2;
    Mat descriptors1, descriptors2;
    Ptr<SIFT> sift = SIFT::create(50,5,.08,10.0,1.6);
    sift->detectAndCompute(im0, Mat(), keypoints1, descriptors1,0);
    sift->detectAndCompute(im1, Mat(), keypoints2, descriptors2,0);



    GpuMat keypoints1GPU, keypoints2GPU;
    GpuMat descriptors1GPU, descriptors2GPU;
    keypoints1GPU.upload(keypoints1);keypoints2GPU.upload(keypoints2);    
    descriptors1GPU.upload(descriptors1);descriptors2GPU.upload(descriptors2);    



    // // matching descriptors
    //Ptr<DescriptorMatcher> matcher= DescriptorMatcher::createBFMatcher(cv2.NORM_L2); 
    cv::Ptr<cv::cuda::DescriptorMatcher> matcher = cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_L2);
    cv::cuda::GpuMat d_matches;    
    matcher->matchAsync(descriptors1GPU, descriptors2GPU, d_matches);


    vector<DMatch> matches;
    vector<DMatch> matches1;


    matcher->matchConvert(d_matches, matches);



    // drawing the results
    Mat img_matches;
    drawMatches(Mat(im0), keypoints1, Mat(im1), keypoints2, matches, img_matches);

    // namedWindow("matches", 0);
    imshow("matches", img_matches);
    waitKey(0);


}
    return 0;
}
