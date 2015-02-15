


#include<list>
//opencv core data structures
#include <opencv2/core.hpp>


//feature includes
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/xfeatures2d/cuda.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/viz.hpp>


#include "mvs.hpp"
#include "triangulation.hpp"
#include "homography.hpp"
#include "matching_score.hpp"

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
                descriptors.resize(nimages);
                
                for (int i=0; i<nimages; i++)
                {
                        
                       
                       
                 
                          
                        Mat dst;
                        Mat viz;
                         double contrast_threshold=0.001,edge_threshold=3.0,sigma=1.2;
                        //double contrast_threshold=0.005,edge_threshold=3.0,sigma=1.2;
                        int nbr_levels=5;

                       // cv::Mat greyMat, colorMat;
                        //cv::cvtColor(colorMat, greyMat, cv::COLOR_BGR2GRAY);

                        Ptr<SIFT> sift = SIFT::create(50000,nbr_levels,contrast_threshold,edge_threshold,sigma);
                        sift->detectAndCompute(images[i], Mat(), features[i],descriptors[i],0);
                        
//                         sort(features[i].begin(), features[i].end(), [](const KeyPoint & a, const KeyPoint & b) -> bool
//                         { 
//                             return a.response> b.response; 
//                         });
                        //sort(features[i].begin(), features[i].end(), greater< KeyPoint<float> >());
                        //response
                        //cv::resize(dst, dst, cv::Size(), 0.3, 0.3);                                        
                      //  cv::drawKeypoints(images[i], features[i], viz);
                        //cv::resize(viz,viz, cv::Size(), 0.25, 0.25);  
                      //  cv::imshow("input (scaled)", viz);                                        
                     //   cv::waitKey();
                        //drawKeypoints(image, keypoints
                        cout<<"Found "<<features[i].size()<<" interest points\n";
                        cout<<"Computed "<<descriptors[i].size()<<" descriptors\n";
                        

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
                        
//                         cv::resize(dst, dst, cv::Size(), 0.3, 0.3);                                        
                       cv::drawKeypoints(images[i], features[i], viz);
//                        cv::resize(viz,viz, cv::Size(), 0.25, 0.25);  
                       cv::imshow("input (scaled)", viz);                                        
                       cv::waitKey();
                        //drawKeypoints(image, keypoints
                        cout<<"Found "<<features[i].size()<<" interest points\n";

		}

	}
	template <typename T>
	static float distancePointLine(const cv::Point_<T> point, const cv::Vec<T,3>& line)
        {
        //Line is given as a*x + b*y + c = 0
        return std::abs(line(0)*point.x + line(1)*point.y + line(2))
            / std::sqrt(line(0)*line(0)+line(1)*line(1));
        }
	
	void test_fundamental(int i1, int i2)
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
            

            
            
            
            cout << "Matching features between image " << i1+1 << " and image " << i2+1 << "... \n"; cout.flush();
            const camera<double> &camera1 = cameras[i1], &camera2 = cameras[i2];  
            
            Vec3 epi= camera2.epipole(camera1);
            Mat3 F = camera2.fundamental(camera1);
             
            std::vector<KeyPoint>& f1=features[i1];
            std::vector<KeyPoint>& f2=features[i2];
                      

            int c1 = 0;
            for (vector< KeyPoint >::iterator cit1 = features[i1].begin()+1 ; cit1 != features[i1].end() ; ++cit1, c1++)
            {
                // Compute the epipolar line in image 2
                Vec3 e = F* Vec3( cit1->pt.x, cit1->pt.y, 1.f );
                float n = sqrt(e[0]   * e[0]+ e[1]* e[1]);
               
                float min_score = 1;//(2*matching_size+1)*(2*matching_size+1)*10.*10.;
                int c2 = 0;
                float  epipolar_tol(2.5f);
                circle( im3, Point( cit1->pt.x, cit1->pt.y), 5,  Scalar( 255),3 );
                cv::line(im3,cv::Point(im1.cols,-e[2]/e[1]),cv::Point(im1.cols+im1.cols-1,-(e[2]+e[0]*(im1.cols-1))/e[1]), cv::Scalar(255));
                for (vector< KeyPoint >::iterator cit2 = features[i2].begin() ; cit2 != features[i2].end() ; ++cit2, c2++)
                {
                    
                    
                    
                    
                        float tmp =distancePointLine(cit2->pt, e);
                        std::cout<<"Distance "<<tmp<<endl;
                        if( tmp > .1)
                        {
                            //The point match is no inlier
                            continue;
                        }
                 
                 
                        cout<<"Tolerance met\n";                 

                        circle( im3, Point( cit2->pt.x+im1.cols ,cit2->pt.y), 5,  Scalar( 255),3 );                       
                        cout<<cit1->pt<<"  "<<cit2->pt<<endl;
                        
                        imshow("im3", im3);
                        cv::waitKey();
                 
                }
                cout << "Matching features between image " << i1+1 << " and image " << i2+1 << "... \n"; cout.flush();
                


                    
            }
            
        }
	
	
	std::vector< pair<int,Vec3> >compute_match(int i1, int i2)
        {
            int VIZ=0;
            int  matching_size =5;
            Mat im1 = images[i1];
            Mat im2=images[i2];
            Size sz1 = images[i1].size();
            Size sz2 = images[i2].size();
            Mat im3(sz1.height, sz1.width+sz2.width, CV_8UC1);
            if(VIZ)
            {

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

            }
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
                //std::cout<<" c1 "<<c1<<endl; 
                if (cit1->pt.x < matching_size || cit1->pt.x > im1.cols-matching_size || cit1->pt.y < matching_size ||  cit1->pt.y > im1.rows-matching_size) continue;
                
                
                // Compute the epipolar line in image 2
                Vec3 e = F * Vec3( cit1->pt.x, cit1->pt.y, 1.f );
                
                float n = sqrt(e[0]   * e[0]+ e[1]* e[1]);

                Vec3 dir = camera1.direction();
                
                if(VIZ)
                {
                    circle( im3, Point( cit1->pt.x ,cit1->pt.y), 5,  Scalar( 255),3);
                    imshow("im3", im3);
                    cv::waitKey(0);
                }
                // Look for most similar feature in the epipolar line
                corresp[c1].first = -1;
                float min_score = 1;//(2*matching_size+1)*(2*matching_size+1)*10.*10.;
                int c2 = 0;
                float  epipolar_tol(2.5f);
                float  reproj_tol(2.0);
                
                for (vector< KeyPoint >::iterator cit2 = features[i2].begin() ; cit2 != features[i2].end() ; ++cit2, c2++)
                {
                    if (cit2->pt.x < matching_size || cit2->pt.x > im2.cols-matching_size || cit2->pt.y < matching_size ||  cit2->pt.y > im2.rows-matching_size) continue;

                        // distance to the epipolar line filtering
                        if(distancePointLine(cit2->pt, e) > epipolar_tol) continue;
                        
                   

                        double error, rcond;
                        bool infront=true;
                        Triangulation<double> triang;
                        triang.add(camera1,camera2);                                             
                        Vec3 M=  triang.LinearLSTriangulation(Vec3(cit1->pt.x,cit1->pt.y,1.0), Vec3(cit2->pt.x,cit2->pt.y,1.0),error,infront);
                        if (!infront || error > reproj_tol ) continue;



                        // Estimate homography between the two views and compute the matching score                       
                        Homography<double> H(camera1, camera2, M, dir);                      
                        double x=0.,y=0.;
                        H.transform(cit1->pt.x,cit1->pt.y,x,y);
                        
     
                        double score = matching_score<double>(im1, im2, cit1->pt, H, matching_size);
                        if (score < min_score)
                        {                           
                                min_score = score;
                                corresp[c1] = make_pair(c2,M);
                                if(VIZ)
                                {
                                    circle( im3, Point( cit2->pt.x+im1.cols ,cit2->pt.y), 5,  Scalar( 255),3 );
                                    imshow("im3", im3);
                                  //  cv::waitKey(0);
                                }
                        }
                        //cv::waitKey(0);                                  
                      // std::exit(0);                            
                }
 
            }
            return corresp;
        }

      
      
      
        // Match features points of an image pair
        void match_features1(int i1, int i2)
        {
            
            
                            float  epipolar_tol(2.5f);
                float  reproj_tol(2.0);
                float distance_tol=10.;
                
                cout << "Matching features between image " << i1+1 << " and image " << i2+1 << "... "; cout.flush();

                assert(i1>=0 && i1<nimages && i2>=0 && i2< nimages && i1 != i2);

                //Window W = OpenWindow(images[i1].width()+images[i2].width(),max(images[i1].height(),images[i2].height()));
                

                const camera<double> &camera1 = cameras[i1], &camera2 = cameras[i2];  
                Mat3 F = camera2.fundamental(camera1);
                
                // For each feature in image 1
                int f1 = 0, nmatches = 0;
                //for (Array<Feature>::iterator fit1 = features[i1].begin() ; fit1 != features[i1].end() ; ++fit1, f1++)
                for (vector< KeyPoint>::iterator fit1 = features[i1].begin() ; fit1 != features[i1].end() ; ++fit1, f1++)
            
                {
                        //display(images[i1]);
                        //display(images[i2],Point2(images[i1].width(),0));
                        //DrawFeature(*fit1);
                        
                        
                        // Compute the epipolar line in image 2
                        Vec3 e = F * Vec3( fit1->pt.x, fit1->pt.y, 1.f );
                
                        float n = sqrt( e(0)* e(0) + e(1) * e(1) );

                        // Estimate the scale and orientation of the descriptor in image 2
                        Vec3 P3 = Vec3(( cameras[i1].R().row(2)).operator ()(0,0),
                                        ( cameras[i1].R().row(2)).operator ()(0,1),
                                        ( cameras[i1].R().row(2)).operator ()(0,2));
                        
                        float p3 = cameras[i1].t()(2);
                        
                        
                        
                        Vec3 Q3 = Vec3(( cameras[i2].R().row(2)).operator ()(0,0),
                                        ( cameras[i2].R().row(2)).operator ()(0,1),
                                        ( cameras[i2].R().row(2)).operator ()(0,2));
                        
      
                        
                        float q3 = cameras[i2].t()(2);

                        // Look for corresponding features
                        list< pair<double,int> > distances;
                        int f2 = 0;
                        for (vector< KeyPoint>::iterator fit2 = features[i2].begin() ; fit2 != features[i2].end() ; ++fit2, f2++)
                        {
                                // If the distance to the epipolar line exceeds a threshold, skip it
                                if ( fabs(fit2->pt.x * e(0) + fit2->pt.y * e(1) + e(2)) > n * epipolar_tol ) continue;

                                //DrawFeature(*fit2,Point2(images[i1].width(),0));

                                
                                // Compute 3D position
                                double error, rcond;
                                bool infront=true;
                                Triangulation<double> triang;
                                triang.add(camera1,camera2);                                             
                                Vec3 M=  triang.LinearLSTriangulation(Vec3(fit1->pt.x,fit1->pt.y,1.0), Vec3(fit2->pt.x,fit2->pt.y,1.0),error,infront);
                                if (!infront || error > reproj_tol ) continue;
                                
                                
                                float scale2 = fit1->size * ( cameras[i2].alpha() * ( Mat(P3).dot(Mat(M))  + p3 ) ) / ( cameras[i1].beta() * ( Mat(Q3).dot(Mat(M))+ q3 ) );

                                
                                //cout << fit1->scale << " " << fit2->scale << " (" << scale2 << ")" << endl;

                                if (fit2->size> scale2*1.5 || fit2->size < scale2/1.5) continue;

                                //DrawFeature(*fit2,Point2(images[i1].width(),0),Yellow);

                                //double dangle = min( double(fabs(fit1->angle - fit2->angle)), min( fabs(fit1->angle - fit2->angle + 2*M_PI), fabs(fit1->angle - fit2->angle - 2*M_PI) ) );
                                //if (dangle > M_PI/4) continue;

                                // Compute the squared distance between descriptors
                                
                                Mat tmp1=descriptors[i1].row(f1);
                                Mat tmp2=descriptors[i2].row(f2);
                                Mat tmp3=tmp1-tmp2;
                                distances.push_back( make_pair( tmp3.dot(tmp3), f2 ) );

                        }
                        
                        

                         
                        //if (distances.size() < 2) continue;
                        //distances.sort();
                        //if (distances.begin()->first / (++distances.begin())->first < 0.8*0.8) {
                        //      insert_feature_pair(i1,f1,i2,distances.begin()->second);
                        //      nmatches++;
                        //}

                        // Select the best pair and the pairs which are nearly as good as the best one
                        if (distances.empty()) {
                                //Click();
                                continue;
                        }

                        double distance_min = min_element( distances.begin(), distances.end() )->first;
                        double distance_thres = distance_min / distance_tol / distance_tol;
                        for (list< pair<double,int> >::iterator dit = distances.begin(); dit != distances.end() ; ++dit)
                        {
                                if (dit->first < distance_thres)
                                {
                                        insert_feature_pair(i1,f1,i2,dit->second);
                                        nmatches++;
                                        //DrawFeature(features[i2][dit->second],Point2(images[i1].width(),0),Green);
                                }


                                if (dit->first == distance_min)
                                {
                                       // DrawFeature(features[i2][dit->second],Point2(images[i1].width(),0),Blue);
                                }
                        }

                      //  Click();
                }
                cout << nmatches << " matches" << endl;
        }
      
        std::vector< pair<int,Vec3> > compute_match_iccv(int i1, int i2)
        {
            
            
            Mat im1 = images[i1];
            Mat im2=images[i2];
            
            cout << "Matching features between image " << i1+1 << " and image " << i2+1 << "... "; cout.flush();
            const camera<double> &camera1 = cameras[i1], &camera2 = cameras[i2];  
            
            Mat3 F = camera2.fundamental(camera1);
            Vec3 dir = camera1.direction();            
            vector< pair<int,Vec3> > corresp(features[i1].size());
            std::fill (corresp.begin(),corresp.end(),make_pair(-1,Vec3(0,0,0)) );
            
            // For each feature in image 1
            int c1 = 0;
            for (vector< KeyPoint >::iterator cit1 = features[i1].begin()+1 ; cit1 != features[i1].end() ; ++cit1, c1++)
            {
                if (cit1->pt.x < 0 || cit1->pt.x > im1.cols-1 || cit1->pt.y < 0 ||  cit1->pt.y > im1.rows-1) continue;
                
                
                // Compute the epipolar line in image 2
                Vec3 e = F * Vec3( cit1->pt.x, cit1->pt.y, 1.f );                
                float n = sqrt(e[0]   * e[0]+ e[1]* e[1]);

                
                // Look for most similar feature in the epipolar line
                corresp[c1].first = -1;               
                int c2 = 0;
                float  epipolar_tol(2.5f);
                float  reproj_tol(2.0);
                std::vector <KeyPoint> collect1;
                for (vector< KeyPoint >::iterator cit2 = features[i2].begin() ; cit2 != features[i2].end() ; ++cit2, c2++)
                {
                    if (cit2->pt.x < 0|| cit2->pt.x > im2.cols-1 || cit2->pt.y < 0 ||  cit2->pt.y > im2.rows-1) continue;

                        if(distancePointLine(cit2->pt, e) > epipolar_tol) continue;                            
                        collect1.push_back(*cit2);
                        
                }                
            }
        }

        
        
        // Match features points of an image pair
        void match_features(int i1, int i2)
        {
                cout << "Matching features between image " << i1+1 << " and image " << i2+1 << "... "; cout.flush();

                //Window W = OpenWindow(images[i1].width()+images[i2].width(),max(images[i1].height(),images[i2].height()));
                //display(images[i1]);
                //display(images[i2],Point2(images[i1].width(),0));
                
                // Compute the fundamental matrices between the two images
                std::vector< pair<int,Vec3> >
                        corresp1 = compute_match(i1,i2),
                        corresp2 = compute_match(i2,i1);

                int nmatches = 0;
                for (int c1=0; c1<corresp1.size(); c1++)
                {
                        int c2 = corresp1[c1].first;
                        if (c2 != - 1 && corresp2[c2].first == c1)
                        {
                                nmatches++;
                                points.push_back(corresp1[c1].second);
                                //DrawLine(Point2(features[i1][c1].pos),Point2(features[i2][c2].pos)+Point2(images[i1].width(),0),Red);
                        }
                }
                cout << nmatches << " matches" << endl;
               // Click();
        }
        
        void save_cloud(){
        
                cv::viz::writeCloud ("cloud.ply",InputArray(points));   

        }
protected :
    
    
    
    
        vector<Mat>        descriptors;    
	vector<vector<KeyPoint>>   features;
	vector<Point3f>    points; 

	//feature selection parameters
	float tol_epi,tol_dist,tol_reproj,rcond_tol,window ;


        // Insert a pair of feature points
        void insert_feature_pair(int i1, int f1, int i2, int f2)
        {
            assert(i1>=0 && i1<nimages && i2>=0 && i2< nimages && i1 != i2);
            assert(f1>=0 && f1<features[i1].size() && f2>=0 && f2<features[i2].size());
            
            double error, rcond,reproj_tol;
            reproj_tol=2.5;
            bool infront=true;
            Triangulation<double> triang;
            triang.add(cameras[i1],cameras[i2]);                                             
            Vec3 M=  triang.LinearLSTriangulation(Vec3(features[i1][f1].pt.x,features[i1][f1].pt.y,1.0), Vec3(features[i2][f2].pt.x,features[i2][f2].pt.y,1.0),error,infront);
            if (!infront || error > reproj_tol ) return;

            points.push_back(M);
        }
        
};