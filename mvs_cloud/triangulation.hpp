#ifndef TRIANGULATION_H
#define TRIANGULATION_H

#include "camera.hpp"

template <typename T=double>
class Triangulation{

        typedef Matx<T,3,3> Mat3;
        typedef Matx<T,3,4> Mat34;
        typedef Vec<T,3>    Vec3;
        typedef Vec<T,2>    Vec2;
        typedef camera<T>   Camera;
        typedef T           class_type;
        
public:
    
    Triangulation(){}
    ~Triangulation(){}

    void add(const camera<T>& c1,const camera<T>& c2)
    {
       // Mat_<T> t=hconcat(c1.P(),c2.p());
       /* _P1=hconcat(c1.P(),c2.p());
        _P2=hconcat(c1.P(),c2.p());
       */ 
    }
    
    
    Vec3 triangulate(T& residual ){
        
        Vec3 M;
        

//        M =IterativeLinearLSTriangulation();
        M =LinearLSTriangulation();
        return Vec3(0,0,0);
    }



     Vec3 LinearLSTriangulation(Vec3 u,  Vec3 u1)     
    {
        //build matrix A for homogenous equation system Ax = 0
        //assume X = (x,y,z,1), for Linear-LS method
        //which turns it into a AX = B system, where A is 4x3, X is 3x1 and B is 4x1
        Mat34 A(u(0)*_P1(2,0)-_P1(0,0),    u(0)*_P1(2,1)-_P1(0,1),      u(0)*_P1(2,2)-_P1(0,2),
            u(1)*_P1(2,0)-_P1(1,0),    u(1)*_P1(2,1)-_P1(1,1),      u(1)*_P1(2,2)-_P1(1,2),
            u1(0)*_P2(2,0)-_P2(0,0), u1(0)*_P2(2,1)-_P2(0,1),   u1(0)*_P2(2,2)-_P2(0,2),
            u1(1)*_P2(2,0)-_P2(1,0), u1(1)*_P2(2,1)-_P2(1,1),   u1(1)*_P2(2,2)-_P2(1,2)
                );
        Mat_<T> B = (Mat_<T>(4,1) <<    -(u(0)*_P1(2,3)    -_P1(0,3)),
                        -(u(1)*_P1(2,3)  -_P1(1,3)),
                        -(u1(0)*_P2(2,3)    -_P2(0,3)),
                        -(u1(1)*_P2(2,3)    -_P2(1,3)));
    
        Mat_<T> X;
        solve(A,B,X,DECOMP_SVD);
        convertPointsFromHomogeneous(X,X);
        return Vec3(X(0),X(1),X(2));
    }

    
    
    
    
    
    
    
    
    
    
//     /**
//     From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
//     */
//     Vec3 IterativeLinearLSTriangulation(Vec u,    //homogenous image point (u,v,1)
//                                                 Mat34 P,          //camera 1 matrix
//                                                 Vec u1,         //homogenous image point in 2nd camera
//                                                 Mat34 P1          //camera 2 matrix
//                                                 ) {
//         double wi = 1, wi1 = 1;
//         Mat_<double> X(4,1); 
//         for (int i=0; i<10; i++) { //Hartley suggests 10 iterations at most
//             Mat_<double> X_ = LinearLSTriangulation(u,P,u1,P1);
//             X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X_(3) = 1.0;
//             
//             //recalculate weights
//             double p2x = Mat_<double>(Mat_<double>(P).row(2)*X)(0);
//             double p2x1 = Mat_<double>(Mat_<double>(P1).row(2)*X)(0);
//             
//             //breaking point
//             if(fabsf(wi - p2x) <= EPSILON && fabsf(wi1 - p2x1) <= EPSILON) break;
//             
//             wi = p2x;
//             wi1 = p2x1;
//             
//             //reweight equations and solve
//             Mat43 A((u(0)*_P1(2,0)-P(0,0))/wi,       (u(0)*_P1(2,1)-P(0,1))/wi,         (u(0)*_P1(2,2)-P(0,2))/wi,     
//                     (u(1)*_P1(2,0)-P(1,0))/wi,       (u(1)*_P1(2,1)-P(1,1))/wi,         (u(1)*_P1(2,2)-P(1,2))/wi,     
//                     (u1(0)*_P2(2,0)-_P2(0,0))/wi1,   (u1(0)*_P2(2,1)-_P2(0,1))/wi1,     (u1(0)*_P2(2,2)-_P2(0,2))/wi1, 
//                     (u1(1)*_P2(2,0)-_P2(1,0))/wi1,   (u1(1)*_P2(2,1)-_P2(1,1))/wi1,     (u1(1)*_P2(2,2)-_P2(1,2))/wi1
//                     );
//             Mat_<double> B = (Mat_<double>(4,1) <<    -(u(0)*_P1(2,3)    -P(0,3))/wi,
//                             -(u(1)*_P1(2,3)  -P(1,3))/wi,
//                             -(u1(0)*_P2(2,3)    -_P2(0,3))/wi1,
//                             -(u1(1)*_P2(2,3)    -_P2(1,3))/wi1
//                             );
//             
//             solve(A,B,X_,DECOMP_SVD);
//             X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X_(3) = 1.0;
//         }
//         return X;
//     }


protected:
    
    
    int num_views;
    Mat34 _P1,_P2;
    
    T* data;





};

#endif