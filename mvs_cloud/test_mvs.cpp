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
	m.compute_img_features_sift();
//         m.compute_img_features_harris();
//        m.test_fundamental(0,1);
        //m.compute_match(0,1);
        m.match_features1(0,1);
        std::exit(0);
        for(int i =1; i<m.size_data();i++)
            m.match_features(i-1,i);
        m.save_cloud();
	return 0;


}