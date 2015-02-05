#include <opencv2/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>

using namespace cv;

class camera
{
	

	typedef Matx<float,3,4> Mat34;
	typedef Matx<float,3,3> Mat3;
	typedef Vec<float,4>    Vec4;
	typedef Vec<float,3>    Vec3;
	typedef Vec<float,2>    Vec2;

public :

	camera(){}
	camera(Mat34 p)
	{
			_P=p;
			Vec4 t;
			decomposeProjectionMatrix( _P, _K	, _R,  t);			
			_t= Vec3(t[0]/_t[3],t[1]/_t[3],t[2]/_t[3]);
			_c = _R.t() * _t;

	}

protected :




	/// Projection matrix P = K[R|t]
	Mat34 _P;

	/// Intrinsic parameter (Focal, principal point)
	Mat3 _K;

	/// Extrinsic Rotation
	Mat3 _R;

	/// Extrinsic translation
	Vec3 _t;

	/// Camera center
	Vec3 _c;

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
	// static double Residual(
	// const Mat34 & P,
	// const Vec3 & pt3D,
	// const Vec2 & ref) {
	// return (ref - Project(P, pt3D)).norm();
	// }

	// /// Return the residual value to the given 2d point
	// double Residual(const Vec3 & pt3D, const Vec2 & ref) const  {
	// return (ref - openMVG::Project(_P, pt3D)).norm();
	// }

	// double ResidualSquared(const Vec3 & pt3D, const Vec2 & ref) const  {
	// return (ref - Project(_P, pt3D)).squaredNorm();
	// }

	// // Compute the depth of the X point. R*X[2]+t[2].
	// double Depth(const Vec3 &X) const{
	//   return openMVG::Depth(_R, _t, X);
	// }


	// /// Return the angle (degree) between two pinhole point rays
	// static double AngleBetweenRay(
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
	//   double mag = ray1.norm() * ray2.norm();
	//   double dotAngle = ray1.dot(ray2);
	//   return R2D(acos(clamp(dotAngle/mag, -1.0 + 1.e-8, 1.0 - 1.e-8)));
	// }



};