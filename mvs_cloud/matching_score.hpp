#ifndef MATCHING_SCORE_H
#define MATCHING_SCORE_H


#include "homography.hpp"
#include <opencv2/core.hpp>


template<typename T> 
T getSubpix(const cv::Mat& img,const T xx,const T yy)
{
    int x = (int)xx;
    int y = (int)yy;
    
    int x0 = cv::borderInterpolate(x,   img.cols, cv::BORDER_REFLECT_101);
    int x1 = cv::borderInterpolate(x+1, img.cols, cv::BORDER_REFLECT_101);
    int y0 = cv::borderInterpolate(y,   img.rows, cv::BORDER_REFLECT_101);
    int y1 = cv::borderInterpolate(y+1, img.rows, cv::BORDER_REFLECT_101);
//std::cout<<"Get subpx 2"<<endl;
    T a = xx - (T)x;
    T c = yy - (T)y;
    
///T tmp=(double)img.at<uchar>(x0-1, y0+3);
     //       std::cout<<"Get subpx 3asdfaaaaaaaaaaaaaaaa "<<tmp <<"  rrrrrrrrrrrrr"<<endl;
      //  std::cout<<"Get subpx  "<< x0<<" "<<y0<<" "<< x1<<" "<< y1<<" "<<endl;

    T pix =((img.at<uchar>(x0, y0) * (1.f - a) + img.at<uchar>(x0, y1) * a) * (1.f - c));
         + ((img.at<uchar>(x1, y0) * (1.f - a) + img.at<uchar>(x1, y1) * a) * c);
         
         

    
    
    
    
   return pix;
}

// template<T> 
// T getColorSubpix(const cv::Mat& img, cv::Point2f pt)
// {
//     int x = (int)pt.x;
//     int y = (int)pt.y;
// 
//     int x0 = cv::borderInterpolate(x,   img.cols, cv::BORDER_REFLECT_101);
//     int x1 = cv::borderInterpolate(x+1, img.cols, cv::BORDER_REFLECT_101);
//     int y0 = cv::borderInterpolate(y,   img.rows, cv::BORDER_REFLECT_101);
//     int y1 = cv::borderInterpolate(y+1, img.rows, cv::BORDER_REFLECT_101);
// 
//     T a = pt.x - (T)x;
//     T c = pt.y - (T)y;
// 
//    u char b = (uchar)cvRound((img.at<CV_8UC1>(y0, x0)[0] * (1.f - a) + img.at<CV_8UC1>(y0, x1)[0] * a) * (1.f - c)
//                            + (img.at<CV_8UC1>(y1, x0)[0] * (1.f - a) + img.at<CV_8UC1>(y1, x1)[0] * a) * c);
//     uchar g = (uchar)cvRound((img.at<CV_8UC1>(y0, x0)[1] * (1.f - a) + img.at<CV_8UC1>(y0, x1)[1] * a) * (1.f - c)
//                            + (img.at<CV_8UC1>(y1, x0)[1] * (1.f - a) + img.at<CV_8UC1>(y1, x1)[1] * a) * c);
//     uchar r = (uchar)cvRound((img.at<CV_8UC1>(y0, x0)[2] * (1.f - a) + img.at<CV_8UC1>(y0, x1)[2] * a) * (1.f - c)
//                            + (img.at<CV_8UC1>(y1, x0)[2] * (1.f - a) + img.at<CV_8UC1>(y1, x1)[2] * a) * c);
// 
//     return CV_8UC1(b, g, r);
// }

// Score entre 0 et 1 (0 = meilleur score)
template < typename T>
T matching_score(const cv::Mat& I1, const cv::Mat& I2,
                                         cv::Point2f pos, const Homography<T>& H,
                                         int size)
{
     
        //cout<<"Starting Matching \n";

        // Round position to nearest pixel
        int X = int(pos.x + (0.5)), Y = int(pos.y + (0.5));
        if (X<size || X+size>=I1.cols || Y<size || Y+size>=I1.rows) return 1;

        

        // Iterates on the window while summing intensities
        double summ = double((2*size+1)*(2*size+1));
        double sum1(0.), sum2(0.), sum11(0.), sum12(0.), sum22(0.);
        for (int YY=Y-size;YY<=Y+size;YY++) 
            for (int XX=X-size;XX<=X+size;XX++)
            {
                    // Apply homography to obtain corresponding point in image 2
                    T x,y;
                    H.transform(T(XX),T(YY),x,y);                   
                    if (x<0 || y<0 || x>I2.cols-1 || y>I2.rows-1) return 1;
   // cout<<"Starting Matching \n";
                    // Get intensities
                ///    std::cout<<"I1 "<<XX<<" "<<YY<<endl;
                 //   std::cout<<"I2 "<<x<<" "<<y<<endl;

                    //const double i1 = getSubpix<T>(I1,XX,YY);
               
                    double i1 = I1.at<uchar>(XX,YY);
                    //std::exit(0);
                    //double i2 = getSubpix<T>(I2,x,y);
//                          std::cout<<"I1 "<<XX<<" "<<YY<<" "<<i1<<endl;
                    double i2 = I2.at<uchar>(int(x+T(0.5)),int(y+T(0.5)));
//                         std::cout<<"I2 "<<x<<" "<<y<<" "<<i2<<endl;
     
                    
                    sum1 += i1;
                    sum2 += i2;
                    sum11 += i1*i1;
                    sum12 += i1*i2;
                    sum22 += i2*i2;
            }

        // Compute matching score
        double score = 0;
        const double eps = 1e-12;

        double s1 = sum1;
        double s2 = sum2;
//         cout<<"difference mean "<<(fabs(s1-s2)>0.6*255.*summ)<<endl;
        // If means are too different, return the worst score
        if (fabs(s1-s2)>0.3*255.*summ) return 1;

//         cout<<"difference mean "<<fabs(s1-s2)<<endl;
        double s11 = sum11;
        double s12 = sum12;
        double s22 = sum22;

        double var1 = summ * s11 - s1 * s1 + eps;
        double var2 = summ * s22 - s2 * s2 + eps;

        // If variances are too different, return the worst score
        if (var1>4*var2 || var2>4*var1) return 1;

        score = ( summ * s12 - s1 * s2 ) / sqrt( var1 * var2 );

        return std::min( 1., 1. - score / 1. );
}

#endif