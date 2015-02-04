
#include <cstdio>
#include <string>
#include <vector>
#include <iostream>



#include "opencv2/datasets/msm_epfl.hpp"


#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>



#include <opencv2/features2d/features2d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/xfeatures2d/cuda.hpp>
#include "opencv2/calib3d/calib3d.hpp"


using namespace std;
using namespace cv;
using namespace cv::cuda;
using namespace cv::datasets;


int main(int argc, char *argv[])
{



    string path = "/home/nicolas/code/data/fountain/";
    Ptr<MSM_epfl> dataset = MSM_epfl::create();
    dataset->load(path);   
    MSM_epflObj *example2 = static_cast<MSM_epflObj *>(dataset->getTrain()[2].get());
    MSM_epflObj *example3 = static_cast<MSM_epflObj *>(dataset->getTrain()[3].get());
    cv::Mat im1 = cv::imread(path+"png/"+example2->imageName.c_str(), IMREAD_GRAYSCALE);
    cv::Mat im2 = cv::imread(path+"png/"+example3->imageName.c_str(), IMREAD_GRAYSCALE);


	cv::resize(im1, im1, cv::Size(), 0.25, 0.25);
	cv::resize(im2, im2, cv::Size(), 0.25, 0.25);

	//Test projection matrix against camera matrix
	Mat cameraMatrix,rotMatrix,transVect;
	decomposeProjectionMatrix( example2->p, cameraMatrix,  rotMatrix,  transVect);
	
	//compare(example2->camera.mat1,cameraMatrix,cameraMatrix,CMP_EQ);
	cout<<example2->camera.mat1<<endl;
	cout<<cameraMatrix<<endl;
	cout<<"ROT COMPARE\n";
	cout<<example2->camera.mat3<<endl;
	cout<<rotMatrix.t()<<endl;
	//convertPointsFromHomogeneous(transVect,transVect);
	cout<<"TRANS COMPARE\n";
	cout<<example2->camera.mat4[0]<<endl;
	cout<<transVect<<endl;


	///match points 


    cv::cuda::printShortCudaDeviceInfo(cv::cuda::getDevice());

    SURF_CUDA surf;
    surf.hessianThreshold=10.0;
    GpuMat img1, img2;
	img1.upload(im1);img2.upload(im2);
    // detecting keypoints & computing descriptors
    GpuMat keypoints1GPU, keypoints2GPU;
    GpuMat descriptors1GPU, descriptors2GPU;
    surf(img1, GpuMat(), keypoints1GPU, descriptors1GPU);
    surf(img2, GpuMat(), keypoints2GPU, descriptors2GPU);

    cout << "FOUND " << keypoints1GPU.cols << " keypoints on first image" << endl;
    cout << "FOUND " << keypoints2GPU.cols << " keypoints on second image" << endl;

    // // matching descriptors
    //Ptr<DescriptorMatcher> matcher= DescriptorMatcher::createBFMatcher(cv2.NORM_L2); 
    cv::Ptr<cv::cuda::DescriptorMatcher> matcher = cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_L2);
    //BFMatcher_CUDA matcher(surf.defaultNorm());
    cv::cuda::GpuMat d_matches;
    //matcher->matchAsync(descriptors1GPU, descriptors2GPU, trainIdx, distance);
	matcher->matchAsync(descriptors1GPU, descriptors2GPU, d_matches);

    // // downloading results
    vector<KeyPoint> keypoints1, keypoints2;
    vector<float> descriptors1, descriptors2;
    vector<DMatch> matches;
    vector<DMatch> matches1;

    surf.downloadKeypoints(keypoints1GPU, keypoints1);
    surf.downloadKeypoints(keypoints2GPU, keypoints2);
    surf.downloadDescriptors(descriptors1GPU, descriptors1);
    surf.downloadDescriptors(descriptors2GPU, descriptors2);
    // BFMatcher_CUDA::matchDownload(trainIdx, distance, matches);
	 matcher->matchConvert(d_matches, matches);
    // // drawing the results
    Mat img_matches;
    drawMatches(Mat(img1), keypoints1, Mat(img2), keypoints2, matches, img_matches);

    // namedWindow("matches", 0);
    imshow("matches", img_matches);
    waitKey(0);
	// convertPointsToHomogeneous(InputArray src, OutputArray dst)
	// convertPointsFromHomogeneous(InputArray src, OutputArray dst)


	//TRIANGULATION
 	//C++: void triangulatePoints(InputArray projMatr1, InputArray projMatr2, InputArray projPoints1, InputArray projPoints2, OutputArray points4D)
	// Python: cv2.triangulatePoints(projMatr1, projMatr2, projPoints1, projPoints2[, points4D]) → points4D
	// C: void cvTriangulatePoints(CvMat* projMatr1, CvMat* projMatr2, CvMat* projPoints1, CvMat* projPoints2, CvMat* points4D)
	// Parameters:	
	// projMatr1 – 3x4 projection matrix of the first camera.
	// projMatr2 – 3x4 projection matrix of the second camera.
	// projPoints1 – 2xN array of feature points in the first image. In case of c++ version it can be also a vector of feature points or two-channel matrix of size 1xN or Nx1.
	// projPoints2 – 2xN array of corresponding points in the second image. In case of c++ version it can be also a vector of feature points or two-channel matrix of size 1xN or Nx1.
	// points4D – 4xN array of reconstructed points in homogeneous coordinates.
}