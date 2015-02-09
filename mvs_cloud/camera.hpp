#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>

using namespace cv;
using namespace std;

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
	camera(Mat34 P,T scale)
	{
            
            Vec4 t;
            decomposeProjectionMatrix( P, _K, _R,  t);
            

            _t= Vec3(t[0]/t[3],t[1]/t[3],t[2]/t[3]);    
            _p=Vec3(P(0,3),P(1,3),P(2,3));
            
            std::cout<<"P "<<P<<std::endl;
                 std::cout<<"p "<<-_K*_R*_t<<std::endl;
                       
            _u0=_K(0,2);_v0=_K(1,2);        
            _alpha=_K(0,0);_beta=_K(1,1);_skew=_K(0,1);
        
            _update_Pp_from_KRt();
           
           
         this->scaleCamera(scale);
         _update_Pp_from_KRt();

            

	}
        camera(Mat3 K,Mat3 R, Vec3 t_,T scale)
        {
            _t=t_;  
            _K=K*scale;
            _R=R;
            _K(2,2)*=(1.0/scale);
            _u0=_K(0,2);_v0=_K(1,2);        
            _alpha=_K(0,0);_beta=_K(1,1);_skew=0.0;
           // std::cout<<"scale "<<scale<<std::endl;
            //std::cout<<"K "<<_K<<std::endl;
            
            _update_Pp_from_KRt();
            
           
            std::cout<<"P "<<_P<<std::endl;
            std::cout<<"p "<<_p<<std::endl;
            std::cout<<"K "<<_K<<std::endl;
            std::cout<<"R "<<_R<<std::endl;
            std::cout<<"T "<<_t<<std::endl;
            
            this->scaleCamera(scale);
            
            
            _update_Pp_from_KRt();
            
           
//             std::cout<<"P "<<_P<<std::endl;
//             std::cout<<"p "<<_p<<std::endl;
//             std::cout<<"K "<<_K<<std::endl;
//             std::cout<<"R "<<_R<<std::endl;
//             std::cout<<"T "<<_t<<std::endl;
//             
//             std::exit(0);              


        }
protected :


       
    
        //P = [P3|p]        
        Mat3 _P;
        Vec3 _p;  


        // Intrinsic matrix
        Mat3 _K;

	// Extrinsic Rotation
	Mat3 _R;

	// Extrinsic translation
	Vec3 _t;

       
        //image center
        T _u0,_v0;
        //Focal parameters
        T _alpha,_beta,_skew;
        
        
        /*
        * Downscaling formula:
        *
        * 1 time:
        *   x -> 0.5 * (x - 0.5)
        *      = 0.5 * x - 0.25
        *
        * n times:
        *   x -> 0.5^n * x + 0.5 * (0.5^n - 1)
        */
        void scaleCamera(int n)
        {
            if(n<= 0)
                return;
            float scaling1= std::pow(0.5, double(n));
            float scaling2= std::pow(0.5, double(n + 1)) - 1.0;
           cout<<"Scaling 1 "<<scaling1<<endl;
           cout<<"Scaling 2 "<<scaling2<<endl;
            const Mat3      M(scaling1,   0,  scaling2,
                                0, scaling1, scaling2,
                                0,   0,   1);

            _K = M * _K;
            _u0=_K(0,2);_v0=_K(1,2);        
            _alpha=_K(0,0);_beta=_K(1,1);_skew=_K(0,1);
            //_p = M * _p; 
        }
public        :


        //Projection matrix 3x3
        const Mat3 & P() const { return _P; }
        //Projection matrix 3x3
        Mat3  Pinv() const{ return _P.inv(); }

        Vec3 center() const { return -(_P.inv() * _p); }
        // Returns projection matrix P inverse. 
       // Mat34 Pinv() const { return _P.inverse(); }

    
        //EXTRINSIC ROTATION MATRIX
        const Mat3 & R() const { return _R; }

        //extrinsic translation parameter
        const Vec3 & t() const { return _t; }
        
        //intrinsic parameter matrix
        const Mat3 K() const { return _K;}

        

        const Vec3  p() const { return _p; }
         Vec3  q() const { return -(_P.inv() * _p); }
        
        T alpha() const { return _alpha; }
        T beta() const { return _beta; }
        T skew() const { return _skew; }
        T u0() const { return _u0; }
        T v0() const { return _v0; }
        
        
        /*!
         *  Updates _R, _t, _alpha, _beta, _skew, _u0, _v0 from _P, _p.
         *
         *  \note assumes _alpha, _beta > 0
         */
        void update_KRt_from_Pp()
        {
            
//             const T f = _P.row(2).length();
// 
//             Matx <T,1,3> q0 = _P.row(0) / f;
//             Matx <T,1,3> q1 = _P.row(1) / f;
//             Matx <T,1,3>  q2 = _P.row(2) / f;
// //             const Vec3q = _p / f;
//             
                Vec3 q0(_P(0,0),_P(0,1),_P(0,2));
                Vec3 q1(_P(1,0),_P(1,1),_P(1,2));
                Vec3 q2(_P(2,0),_P(2,1),_P(2,2));
                Vec3 q = _p;
                const T f = norm(q0);
                
                q/=f;q0/=f;q1/=f;q2/=f;
            
// 
             const Vec3& r2 = q2;
             _t[2] = q[2];
// 
             _v0 = norm(q1*r2)*norm(q1*r2);
             _beta = norm(q1 - _v0 * r2);
            _t[1] = (q[1] - _v0 * _t[2]) / _beta;
            const Vec3 r1 = (q1 - _v0 * r2) / _beta;
// 
            _u0 = norm(q0*r2)*norm(q0*r2);
            _skew =norm(q1*r1)*norm(q1*r1);
            _alpha = norm(q0 - _skew * r1 - _u0 * r2)*norm(q0 - _skew * r1 - _u0 * r2);
            _t(0) = (q(0) - _skew * _t(1) - _u0 * _t(2)) / _alpha;
            const Vec3 r0 = (q0 - _skew * r1 - _u0 * r2) / _alpha;

// //             _R.set_rows(r0, r1, r2);
//             _P(0,0) =r0(0);_P(0,1) =row0(1);_P(0,2) =row0(2);
//             _P(1,0) =r1(0);_P(1,1) =row1(1);_P(1,2) =row1(2);
//             _P(2,0) =r2(0);_P(2,1) =r2(1);_P(2,2) =r2(2);
//             
//             _K(0,2)=_u0;_K(1,2)=_v0;        
//             _K(0,0)=_alpha;_K(1,1)=_beta;_K(0,1)_skew;
            
        }
        
        void _update_Pp_from_KRt()
        {
           _P=_K*_R;    
           _p= -_K*_R*_t;
        }
        
        
        /*! Transforms world coords to camera coords. */
        Vec3 toCamera(const Vec3& v) const { return (_R * v + _t); }

        /*! Transforms camera coords to world coords. */
        Vec3 toWorld(const Vec3& v) const { return (_R.t() * (v - _t));
        }


        
        Vec3 epipole(const camera& cam) const {
                return (P()*cam.q() + p() );
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
            const Mat3& P1 = P();
            const Mat3 P2inv = c.Pinv();
            const Vec3& p1 = p();
            const Vec3 p2 = c.q();
    
            Mat3 tmp=wedge((P1 * p2) + p());
            return  tmp* (P1 * P2inv);
        }

        
        static Mat3 wedge(const Vec3& v) {
             return Mat3( .0, -v[2],v[1],
                          v[2], .0, -v[0],
                          -v[1], v[0], .0);

        }
        
        Vec3 direction() const {             
            return Vec3((this->_R.row(2)).operator ()(0,0),
                        (this->_R.row(2)).operator ()(0,1),
                        (this->_R.row(2)).operator ()(0,2));            
        }
        void write(std::ostream& stream) const
        {
            (stream << "alpha=" << _alpha << std::endl
                        << "beta=" << _beta << std::endl
                        << "skew=" << _skew << std::endl
                        << "u0=" << _u0 << std::endl
                        << "v0=" << _v0 << std::endl
                        << "R=" << _R << std::endl
                        << "t=" << _t << std::endl);
        }

};


#endif