
//opencv core data structures
#include <opencv2/core.hpp>


//feature includes
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/xfeatures2d/cuda.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>


#include "mvs.hpp"

using namespace cv;
using namespace std;

class mvs_cloud : public mvs{


	typedef Matx<float,3,3> Matrix3x3f;
	typedef Vec<float,3>    Vec3f;

public : 

	mvs_cloud(){}
	~mvs_cloud(){}

	void compute_img_features()
	{
		int type = 0;
		int blockSize = 2;
		int apertureSize = 3;
		double k = 0.04;
		cout << "Computing feature points... "; cout.flush();
		features.resize(nimages);

		Mat keypoints;
		Mat descriptors;
		Mat dst;
		for (int i=0; i<nimages; i++)
		{
		
	

					cout<<"Harris corner detector\n";		

					cornerHarris( images[i], dst, blockSize, apertureSize, k, BORDER_DEFAULT );
				    // for( size_t j = 0; j < dst.rows; j++ ) {
				    //      for( size_t k = 0; k < dst.cols; k++ ) {
				    //           // Observe the type used in the template
				    //            if(dst.at<uchar>(j,k) >150) 
				    //            	features[i].push_back(KeyPoint(Point2f(j,k),.0,-1, dst.at<uchar>(j,k))) ;
				    //      }

			//drawKeypoints(image, keypoints
    		cout<<"Found "<<features[i].size()<<" interest points\n";

			// Sort the harris corners by decreasing cornerness
			//sort(features[i].begin(), features[i].end(), greater< ImageCorner<float> >());
			//cout << features[i].size() << " "; cout.flush();
		}

	}
	void compute_cloud(){}


protected :

	vector<vector<KeyPoint>>   features;
	vector<Point3f>    points; 

	//feature selection parameters
	float tol_epi,tol_dist,tol_reproj,rcond_tol,window ;


};