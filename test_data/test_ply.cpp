
#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>



#include <opencv2/viz.hpp>
#include <iostream>


#include <string>
#include <vector>
#include <iostream>


using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{


	std:string path = "/home/nicolas/work/code/git/data/mesh/bunny.ply";
	cv::viz::Mesh m= cv::viz::readMesh(path);	




	cv::viz::writeCloud	("/home/nicolas/work/code/git/data/mesh/bunny_cloud.ply",m.cloud);



}