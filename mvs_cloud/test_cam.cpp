#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/datasets/msm_epfl.hpp>
#include <opencv2/datasets/msm_middlebury.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "camera.hpp"




using namespace std;
using namespace cv;
using namespace cv::datasets;





int main()
{
        typedef Matx<double,3,3> Mat3;
        typedef Vec<double,3>    Vec3;
    
        int nimages;
        std::vector<cv::Mat> images;
        std::vector<camera<>> cameras;


//             string path = "/home/nicolas/code/data/templeRing/";
            //string path ="/home/nicolas/code/data/templeSparseRing/templeSR_par.txt";
            string path   ="/home/nicolas/code/data/templeSparseRing/templeSR_par.txt";
                            
            Ptr<MSM_middlebury> dataset = MSM_middlebury::create();
            
            dataset->load(path);

            
            
            //cout<<dataset->getValidation();
            nimages = (int)dataset->getTrain().size();
            nimages=2;
            std::cout<<"Nbr imges in dataset "<<nimages<<endl;
            images.resize(nimages);
            cameras.resize(nimages);
            int scale =1;
            for(int i=0;i<nimages;i++)
            {
               
              MSM_middleburyObj *data= static_cast<MSM_middleburyObj *>(dataset->getTrain()[i].get());
             std::cout<<data->imageName;
              //cv::imread( ("/home/nicolas/code/data/templeSparseRing/" + data->imageName).c_str(), IMREAD_GRAYSCALE);
              //cv::resize(cv::imread(path+"png/"+data->imageName.c_str(), IMREAD_GRAYSCALE),images[i],cv::Size(), 1.0/(1<<scale),1.0/(1<<scale));
                              //        cv::imshow("input (scaled)",images[i]);                                        
                        //cv::waitKey();
             images[i]=cv::imread( ("/home/nicolas/code/data/templeSparseRing/" + data->imageName).c_str(), IMREAD_GRAYSCALE);

             cameras[i]= camera<double>(Mat3(data->k),Mat3(data->r),Vec3(data->t[0],data->t[1],data->t[2]),1.0/(1<<scale));
            }


                camera<>& cam1 = cameras[0];
                camera<>& cam2 = cameras[1];
//             
//               
//                
                 cout<<cam1.P()<<endl;
                cout<<cam1.p()<<endl;
//                 
               cout<<"CAM 2"<<endl;
               cout<<cam2.P()<<endl;
               cout<<cam2.p()<<endl;
               cout<<"Epipole 1 "<<cam1.epipole(cam2)<<endl;
               Vec3 epi=cam2.epipole(cam1);
               cout<<"Epipole 2 "<<cam2.epipole(cam1)<<endl;
                std::cout<<epi(0)/epi(2)<<" "<<epi(1)/epi(2)<<std::endl;
               cout<<"Fundamental\n "<<cam2.fundamental(cam1)<<endl;
               cout<<"Epipole 1 int camera 2\n "<<cam2.epipole(cam1);
               cout<<"Epipole 2 int camera 1\n "<<cam1.epipole(cam2);
               cout<<"Test Fundamental with e1 \n "<<cam2.fundamental(cam1).t()*cam2.epipole(cam1)<<endl;
               cout<<"Test Fundamental with e2 \n "<<cam2.fundamental(cam1)*cam1.epipole(cam2)<<endl;
               
               




}