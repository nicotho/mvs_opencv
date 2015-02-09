#include <iostream>
#include <string>
//opencv core data structures
#include <opencv2/core.hpp>

#include <opencv2/datasets/msm_epfl.hpp>
#include <opencv2/datasets/msm_middlebury.hpp>

#include "camera.hpp"

using namespace std;
using namespace cv;
using namespace cv::cuda;
using namespace cv::datasets;
class mvs{
        typedef Matx<double,3,3> Mat3;
        typedef Vec<double,3>    Vec3;	

public:

	mvs()
	{
	    string path = "/home/nicolas/code/data/fountain/";
	    Ptr<MSM_epfl> dataset = MSM_epfl::create();
	    dataset->load(path);
            nimages = dataset->getTrain().size();
            nimages=4;
            std::cout<<"Nbr imges in dataset "<<nimages<<endl;
            images.resize(nimages);
            cameras.resize(nimages);
            int scale =2;
            for(int i=0;i<nimages;i++)
            {
                
                MSM_epflObj *data = static_cast<MSM_epflObj *>(dataset->getTrain()[i].get());
//                 cv::resize(images[i], images[i], cv::Size(), 0.25, 0.25);      
                cv::resize(cv::imread(path+"png/"+data->imageName.c_str(), IMREAD_GRAYSCALE),images[i],cv::Size(), 1.0/(1<<scale),1.0/(1<<scale));
                cout<<images[i].cols<<" "<<images[i].rows<<endl;
                cameras[i]= camera<>(data->p,scale);
               // cameras[i].write(std::cout);
               // cameras[i].outer_product(Vec<double,3>(1,2,3));
            }



	}
	
// 	 mvs()
//         {
//             string path   ="/home/nicolas/code/data/templeSparseRing/templeSR_par.txt";
//             path="/home/nicolas/code/data/dinoSparseRing/dinoSR_par.txt";
//                             
//             Ptr<MSM_middlebury> dataset = MSM_middlebury::create();
//                      dataset->load(path);
//                      
//                         //cout<<dataset->getValidation();
//             nimages = (int)dataset->getTrain().size();
// //             nimages=2;
//             std::cout<<"Nbr imges in dataset "<<nimages<<endl;
//             images.resize(nimages);
//             cameras.resize(nimages);
//             int scale =0;
//             for(int i=0;i<nimages;i++)
//             {
//                
//               MSM_middleburyObj *data= static_cast<MSM_middleburyObj *>(dataset->getTrain()[i].get());
//              std::cout<<data->imageName;
//               //cv::imread( ("/home/nicolas/code/data/templeSparseRing/" + data->imageName).c_str(), IMREAD_GRAYSCALE);
//               //cv::resize(cv::imread(path+"png/"+data->imageName.c_str(), IMREAD_GRAYSCALE),images[i],cv::Size(), 1.0/(1<<scale),1.0/(1<<scale));
//                               //        cv::imshow("input (scaled)",images[i]);                                        
//                         //cv::waitKey();
//              images[i]=cv::imread( ("/home/nicolas/code/data/dinoSparseRing/" + data->imageName).c_str(), IMREAD_GRAYSCALE);
// 
//              cameras[i]= camera<double>(Mat3(data->k),Mat3(data->r),Vec3(data->t[0],data->t[1],data->t[2]),1.0/(1<<scale));
//             }
//        }



protected:



        int nimages;
        std::vector<cv::Mat> images;
        std::vector<camera<>> cameras;

};