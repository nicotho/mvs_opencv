
#include <opencv2/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>

using namespace cv;


template<class T= double>
class camera
{
	

	typedef Matx<T,3,4> Mat34;
	typedef Matx<T,3,3> Mat3;
	typedef Vec<T,4>    Vec4;
	typedef Vec<T,3>    Vec3;
	typedef Vec<T,2>    Vec2;

public :

	camera(){}
	camera(Mat34 p,T scale)
	{
            _P=p;
            Vec4 t;
            decomposeProjectionMatrix( _P, _K	, _R,  t);			
            _t= Vec3(t[0]/_t[3],t[1]/_t[3],t[2]/_t[3]);
            _c = _R.t() * _t;
            
            
          
            
             _P3=
             _p=
             _K*=scale;
            _u0,_v0;

            
           _alpha*=scale;_beta*=scale;_skew*=scale;_u0*=scale;_v0*=scale;
            
                        
                        

	}

protected :




	// Projection matrix P = K[R|t] = [P3|p]
	Mat34 _P;
        
        //P = [P3|p]
        Mat3 _P3;
        Vec3 _p;  
        
	// Intrinsic parameter (Focal, principal point)
	Mat3 _K;

	// Extrinsic Rotation
	Mat3 _R;

	// Extrinsic translation
	Vec3 _t;

	// Camera center
	Vec3 _c;
        
        //image center
        T _u0,_v0;

        //Focal parameters
        T _alpha,_beta,_skew;
        
        
         
        
public        :

         //Projection matrix
        const Mat34 & P() const { return _P; }

        //Projection matrix 3x3
        const Mat3 & P3() const { return _P3; }
        //Projection matrix 3x3
        const Mat3 & P3inv() const { return _P3.inv(); }

        
        // Returns projection matrix P inverse. 
        const Mat3 Pinv() const { return _P.inverse(); }

    
        //EXTRINSIC ROTATION MATRIX
        const Mat3 & R() const { return _R; }

        //extrinsic translation parameter
        const Vec3 & t() const { return _t; }
        
        //intrinsic parameter matrix
        Mat3 K() const { return _K;}

        

        const Vec3  p() const { return _p; }
        const Vec3  q() const { return -(P3inv() * _p); }
        
        T alpha() const { return _alpha; }
        T beta() const { return _beta; }
        T skew() const { return _skew; }
        T u0() const { return _u0; }
        T v0() const { return _v0; }

        
        
        
        /*! Transforms world coords to camera coords. */
        Vec3 toCamera(const Vec3& v) const { return (_R * v + _t); }

        /*! Transforms camera coords to world coords. */
        Vec3 toWorld(const Vec3& v) const { return (_R.t() * (v - _t));
        }
   /*!
         *  Computes fundamental matrix between this camera and the given one.
         *
         *  \note: m1^T F12 m2 = 0 where
         *         - m1 is in this camera space
         *     and - m2 is in the given camera space
         */
        Mat3 fundamental(const camera& c) const
        {
            const Mat3& P1 = P3();
            const Mat3 Q2 = c.P3inv();
            const Vec3& p1 = p();
            const Vec3 q2 = c.q();

            return outer_product(P1 * q2 + p1) * P1 * Q2;
        }

        
        static Mat3 outer_product(const Vec3& v) {
             return Mat3( .0, -v[2],v[1],
                          v[2], .0, -v[0],
                          -v[1], v[1], .0);

        }
        
        Vec3 direction() const {             
            return Vec3((this->_R.row(2)).operator ()(0,0),
                        (this->_R.row(2)).operator ()(0,1),
                        (this->_R.row(2)).operator ()(0,2));            
        }
            //Mat tmp=this->_R.row(2);
            //return Vec3(tmp(0,1),tmp(0,1),tmp(0,2));}
//             return  this->_R.row(2); }
	/// Projection of a 3D point into the camera plane
	// Vec2 Project(const Mat34 & P, const Vec3 & pt3D)
	// {
	// 	Vec4 HX;
	// 	HX << pt3D, 1.0;
	// 	Vec3 hx = P * HX;
	// 	hx/=hx[2];

	// 	return Vec2(hx[0],hx[1]) ;

	// }

	// /// Projection of a 3D point into the camera plane (member function)
	// Vec2 Project(const Vec3 & pt3D) const
	// {
	// return openMVG::Project(_P, pt3D);
	// }

	// /// Return the residual value to the given 2d point
	// static T Residual(
	// const Mat34 & P,
	// const Vec3 & pt3D,
	// const Vec2 & ref) {
	// return (ref - Project(P, pt3D)).norm();
	// }

	// /// Return the residual value to the given 2d point
	// T Residual(const Vec3 & pt3D, const Vec2 & ref) const  {
	// return (ref - openMVG::Project(_P, pt3D)).norm();
	// }

	// T ResidualSquared(const Vec3 & pt3D, const Vec2 & ref) const  {
	// return (ref - Project(_P, pt3D)).squaredNorm();
	// }

	// // Compute the depth of the X point. R*X[2]+t[2].
	// T Depth(const Vec3 &X) const{
	//   return openMVG::Depth(_R, _t, X);
	// }


	// /// Return the angle (degree) between two pinhole point rays
	// static T AngleBetweenRay(
	//   const PinholeCamera & cam1,
	//   const PinholeCamera & cam2,
	//   const Vec2 & x1, const Vec2 & x2)
	// {
	//   // x = (u, v, 1.0)  // image coordinates
	//   // X = R.t() * K.inv() * x + C // Camera world point
	//   // getting the ray:
	//   // ray = X - C = R.t() * K.inv() * x
	//   Vec3 ray1 = (cam1._R.transpose() *
	//     (cam1._K.inverse() * Vec3(x1(0), x1(1), 1.))).normalized();
	//   Vec3 ray2 = (cam2._R.transpose() *
	//     (cam2._K.inverse() * Vec3(x2(0), x2(1), 1.))).normalized();
	//   T mag = ray1.norm() * ray2.norm();
	//   T dotAngle = ray1.dot(ray2);
	//   return R2D(acos(clamp(dotAngle/mag, -1.0 + 1.e-8, 1.0 - 1.e-8)));
	// }



};