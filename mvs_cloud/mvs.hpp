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
		images.resize(nimages);
		cameras.resize(nimages);
		for(int i=0;i<nimages-1;i++)
		{
		    
		    MSM_epflObj *data = static_cast<MSM_epflObj *>(dataset->getTrain()[i].get());
		    images[i]= cv::imread(path+"png/"+data->imageName.c_str(), IMREAD_GRAYSCALE);
		    cout<<data->p<<endl;
		    cameras[i]= camera(data->p);
	    
		}



	}


protected:



	int nimages;
	std::vector<cv::Mat> images;
	std::vector<camera> cameras;

};