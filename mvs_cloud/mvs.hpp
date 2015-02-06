#include <iostream>
//opencv core data structures
#include <opencv2/core.hpp>

#include <opencv2/datasets/msm_epfl.hpp>


#include "camera.hpp"

using namespace std;
using namespace cv;
using namespace cv::cuda;
using namespace cv::datasets;
class mvs{
	

public:

	mvs()
	{
	    string path = "/home/nicolas/code/data/fountain/";
	    Ptr<MSM_epfl> dataset = MSM_epfl::create();
	    dataset->load(path);
            nimages = dataset->getTrain().size();
            std::cout<<"Nbr imges in dataset "<<nimages<<endl;
            images.resize(nimages);
            cameras.resize(nimages);
            float scale =0.25;
            for(int i=0;i<nimages;i++)
            {
                
                MSM_epflObj *data = static_cast<MSM_epflObj *>(dataset->getTrain()[i].get());
//                 cv::resize(images[i], images[i], cv::Size(), 0.25, 0.25);      
                cv::resize(cv::imread(path+"png/"+data->imageName.c_str(), IMREAD_GRAYSCALE),images[i],cv::Size(), scale,scale);
                //cout<<data->p<<endl;
                cameras[i]= camera<>(data->p,scale);
                cameras[i].outer_product(Vec<double,3>(1,2,3));
            }



	}


protected:



	int nimages;
	std::vector<cv::Mat> images;
	std::vector<camera<>> cameras;

};