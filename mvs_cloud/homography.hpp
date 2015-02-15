#ifndef HOMOGRAPHY_H
#define HOMOGRAPHY_H

//opencv core data structures
#include <opencv2/core.hpp>
#include "camera.hpp"


template <typename T>
class Homography 
{
public:
        typedef Matx<T,3,3> Mat3;
        typedef Vec<T,3>    Vec3;
        typedef Vec<T,3>    Vec2;

        Homography() {  }
        Homography(const Mat& A)   {}
        Homography(const camera<T>& cam1, const camera<T>& cam2, const Vec3& point, const Vec3& normal)
        {       
                //dot product
            
                T np = normal(0)*point(0)+ normal(1)*point(1) + normal( 2)*point(2);
                Mat3 p1n, p2n;

                for (int j=0; j<3; j++) 
                    for (int i=0; i<3; i++)
                {
                        p1n(i,j) = cam1.p()(i)*normal(j);
                        p2n(i,j) = cam2.p()(i)*normal(j);
                }
                

              this->_homography   = ( (cam2.P() + p2n * (1.0/np)) * (cam1.P() + p1n * (1.0/np)).inv() );
        }

        void transform(T x, T y, T& X, T& Y) const
        {
                X = this->_homography(0,0) * x + this->_homography(0,1) * y + this->_homography(0,2);
                Y = this->_homography(1,0) * x + this->_homography(1,1) * y + this->_homography(1,2);
                T Z = this->_homography(2,0) * x + this->_homography(2,1) * y + this->_homography(2,2);
                X /= Z;
                Y /= Z;
        }

        Vec2 transform(const Vec2& p) const
        {
                Vec2 P;
                transform(p.x(),p.y(),P.x(),P.y());
                return P;
        }
        
        
        void print ()
        {
            std::cout << this->_homography<<std::endl;
            
        }
        
protected :
    
        Mat3 _homography;
};


#endif