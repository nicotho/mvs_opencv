#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include "mvs_cloud.hpp"




using namespace std;
using namespace cv;
using namespace cv::cuda;
using namespace cv::datasets;


int main(int argc, char *argv[])
{

	mvs_cloud m;
	m.compute_img_features();
	return 0;


}