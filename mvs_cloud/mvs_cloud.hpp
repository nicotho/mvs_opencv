
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
using namespace cv::xfeatures2d;

class mvs_cloud : public mvs{


	typedef Matx<float,3,3> Mat3;
	typedef Vec<float,3>    Vec3;

public : 

	mvs_cloud(){}
	~mvs_cloud(){}

	void compute_img_features_sift()
        {
                features.resize(nimages);

                
                for (int i=0; i<nimages; i++)
                {
                        
                       
                       
                        Mat keypoints;
                        Mat descriptors;        
                        Mat dst;
                        Mat viz;
                        double contrast_threshold=0.01,edge_threshold=3.0,sigma=1.2;
                        int nbr_levels=5;
			
                        Ptr<SIFT> sift = SIFT::create(10000,nbr_levels,contrast_threshold,edge_threshold,sigma);
                        sift->detectAndCompute(images[i], Mat(), features[i],descriptors,0);
                        sort(features[i].begin(), features[i].end(), [](const KeyPoint & a, const KeyPoint & b) -> bool
                        { 
                            return a.response> b.response; 
                        });
                        //sort(features[i].begin(), features[i].end(), greater< KeyPoint<float> >());
                        //response
                        //cv::resize(dst, dst, cv::Size(), 0.3, 0.3);                                        
                         //cv::drawKeypoints(images[i], features[i], viz);
                      //  cv::resize(viz,viz, cv::Size(), 0.25, 0.25);  
                         //cv::imshow("input (scaled)", viz);                                        
                         //cv::waitKey();
                        //drawKeypoints(image, keypoints
                        cout<<"Found "<<features[i].size()<<" interest points\n";

                }                
                
                
                
        }
	void compute_img_features_harris()
	{
                //int type = 0;
                
                int blockSize = 2;
                int apertureSize = 3;
                double k = 0.06;
                cout << "Computing feature points... "; cout.flush();
                features.resize(nimages);

                
                for (int i=0; i<nimages; i++)
                {
                        cv::resize(images[i], images[i], cv::Size(), 0.25, 0.25);      
                       
                       
                        Mat keypoints;
                        Mat descriptors;        
                        Mat dst;
                        Mat viz;
                        double min, max;
                        cout<<"Harris corner detector\n";		
                        cornerHarris( images[i], dst, blockSize, apertureSize, k, BORDER_DEFAULT );
                        normalize( dst, dst, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
                        cv::minMaxLoc(dst, &min, &max);
                        cout <<"Min "<<min<< " MAX "<<max<<endl;

                        for( int j = 0; j < dst.rows; j++ ) 
                                for( int k = 0; k < dst.cols; k++ ) 
                                {
                                        if(dst.at<float>(j,k) >100.0) 
                                                features[i].push_back(KeyPoint(Point2f(j,k),.0,-1, dst.at<float>(j,k))) ;
                              

                                }
                        
                        //cv::resize(dst, dst, cv::Size(), 0.3, 0.3);                                        
                        cv::drawKeypoints(images[i], features[i], viz);
                      //  cv::resize(viz,viz, cv::Size(), 0.25, 0.25);  
                        cv::imshow("input (scaled)", viz);                                        
                        cv::waitKey();
                        //drawKeypoints(image, keypoints
                        cout<<"Found "<<features[i].size()<<" interest points\n";

		}

	}
	
	
	void compute_match(int i1, int i2)
        {
            
            int  matching_size =3;
            Mat im1 = images[i1];
            Mat im2=images[i2];
            Size sz1 = images[i1].size();
            Size sz2 = images[i2].size();
            Mat im3(sz1.height, sz1.width+sz2.width, CV_8UC1);
            // Move right boundary to the left.
            im3.adjustROI(0, 0, 0, -sz2.width);
            im1.copyTo(im3);
            // Move the left boundary to the right, right boundary to the right.
            im3.adjustROI(0, 0, -sz1.width, sz2.width);
            im2.copyTo(im3);
            // restore original ROI.
            im3.adjustROI(0, 0, sz1.width, 0);
            imshow("im3", im3);
            cv::waitKey();
  
            cout << "Matching features between image " << i1+1 << " and image " << i2+1 << "... "; cout.flush();
            const camera<double> &camera1 = cameras[i1], &camera2 = cameras[i2];  
            Mat3 F = camera2.fundamental(camera1);
            Vec3 dir = camera1.direction();
            // Array to store the best correspondence if any
            vector< pair<int,Vec3> > corresp(features[i1].size());
            std::fill (corresp.begin(),corresp.end(),make_pair(-1,Vec3(0,0,0)) );
            
            // For each feature in image 1
            int c1 = 0;
            for (vector< KeyPoint >::iterator cit1 = features[i1].begin() ; cit1 != features[i1].end() ; ++cit1, c1++)
            {
                if (cit1->pt.x < matching_size || cit1->pt.x > im1.cols-matching_size || cit1->pt.y < matching_size ||  cit1->pt.y > im1.rows-matching_size) continue;
                
                
                // Compute the epipolar line in image 2
                Vec3 e = F * Vec3( cit1->pt.x, cit1->pt.y, 1.f );
                float n = sqrt(e[0]   * e[0]+ e[1]* e[1]);

                
                 circle( im3, Point( cit1->pt.x ,cit1->pt.y), 5,  Scalar( 255),3 );

    
                // Look for most similar feature in the epipolar line
                corresp[c1].first = -1;
                float min_score = 1;//(2*matching_size+1)*(2*matching_size+1)*10.*10.;
                int c2 = 0;
                float  epipolar_tol(2.5f);
                for (vector< KeyPoint >::iterator cit2 = features[i2].begin() ; cit2 != features[i2].end() ; ++cit2, c2++)
                {
                    if (cit2->pt.x < matching_size || cit2->pt.x > im2.cols-matching_size || cit2->pt.y < matching_size ||  cit2->pt.y > im2.rows-matching_size) continue;

                        // If the distance to the epipolar line exceeds a threshold, skip it
                        if ( fabs(cit2->pt.x * e[0] + cit2->pt.y * e[1] + e[3]) > n * epipolar_tol ) continue;

                        
                        circle( im3, Point( cit1->pt.x ,cit1->pt.y), 5,  Scalar( 255),3 );

    
                        imshow("im3", im3);
                    cout<<"Tolerance met\n";
                        cv::waitKey();
                    
                        // Compute 3D position
                        //Triangulation<float> triang;
                        //triang.add(camera1, cit1->pos);
                        //triang.add(camera2, cit2->pos);
                        ///float error, rcond;
                        //bool in_front;
                        //Vector3 p = triang.compute(error,in_front,rcond);
                        //if (!in_front || error > reproj_tol || rcond < rcond_tol) continue;

                        //DrawCircle(Point2(cit2->pos)+Point2(image1.width(),0),5,Blue);

                        // Estimate homography between the two views and compute the matching score
//                         Homography<float> H(camera1, camera2, p, dir);
//                         float score = matching_score<float>(image1, image2, camera1, camera2, cit1->pos, H, matching_size);
//                         if (score < min_score)
//                         {
//                                 min_score = score;
//                                 corresp[c1] = make_pair(c2,p);
//                         }
                }
                
                
                
            }
            
    
    
    
	  
                
            
                
        }

protected :

	vector<vector<KeyPoint>>   features;
	vector<Point3f>    points; 

	//feature selection parameters
	float tol_epi,tol_dist,tol_reproj,rcond_tol,window ;


};