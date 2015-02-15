 #ifndef TRIANGULATION_H
#define TRIANGULATION_H

#include "camera.hpp"

template <typename T=double>
class Triangulation{

        typedef Matx<T,3,3> Mat3;
        typedef Matx<T,3,4> Mat34;
        typedef Matx<T,4,3> Mat43;
        typedef Vec<T,3>    Vec3;
        typedef Vec<T,2>    Vec2;
        typedef camera<T>   Camera;
        typedef T           value_type;
        
public:
    
    Triangulation(){}
    ~Triangulation(){}

    void add(const camera<T>& c1,const camera<T>& c2)
    {
       _P1= c1.P();
       _P2= c2.P();
       _p1=c1.p();
       _p2=c2.p();
    }
    
    



    Vec3 LinearLSTriangulation(Vec3 u,  Vec3 u1,T& residual ,bool& infront)
    {
        //build matrix A for homogenous equation system Ax = 0
        //assume X = (x,y,z,1), for Linear-LS method
        //which turns it into a AX = B system, where A is 4x3, X is 3x1 and B is 4x1
        Mat43 A(u(0)*_P1(2,0) -_P1(0,0) ,    u(0)*_P1(2,1) -_P1(0,1) ,  u(0)*_P1(2,2)-_P1(0,2)  ,
                u(1)*_P1(2,0) -_P1(1,0) ,    u(1)*_P1(2,1) -_P1(1,1) ,  u(1)*_P1(2,2)-_P1(1,2)  ,
                u1(0)*_P2(2,0)-_P2(0,0) ,    u1(0)*_P2(2,1)-_P2(0,1) ,  u1(0)*_P2(2,2)-_P2(0,2) ,
                u1(1)*_P2(2,0)-_P2(1,0) ,    u1(1)*_P2(2,1)-_P2(1,1) ,  u1(1)*_P2(2,2)-_P2(1,2));
        
        Mat_<T> B = (Mat_<T>(4,1) <<    -(u(0) *_p1(2)  -_p1(0))  ,
                                        -(u(1) *_p1(2)  -_p1(1))  ,
                                        -(u1(0)*_p2(2)  -_p2(0))  ,
                                        -(u1(1)*_p2(2)  -_p2(1)));
        
        //std::cout<<endl<< A<<endl;
        //std::cout <<B<<endl;
    
        Mat_<T> X;
        //std::cout<<"solve \n";
        solve(A,B,X,DECOMP_SVD);
       // std::cout<<"Finished solve \n"<<X<<endl;
        
        
        residual=compute_error(X,u, u1,infront);
        
        return Vec3(X(0),X(1),X(2));
    }
    
    
    
    T compute_error(const Vec3& M,Vec3 u,  Vec3 u1,bool & infront)
    {
        value_type residual = 0;

    
        Vec3 m = _P1 * M + _p1;
        value_type x = m(0) / m(2);
        value_type y = m(1) / m(2);
        if (m(2)<=0) infront = false;
        
//         std::cout<< "U "<<u << " reprojected "<<x<<" "<<y<<endl;
        residual += norm(Vec2(x, y) - Vec2(u(0),u(1)))*norm(Vec2(x, y) - Vec2(u(0),u(1)));
    

        m = _P2 * M + _p2;
        x = m(0) / m(2);
        y = m(1) / m(2);
        if (m(2)<=0) infront = false;
        residual += norm(Vec2(x, y) - Vec2(u1(0),u1(1)))*norm(Vec2(x, y) - Vec2(u1(0),u1(1)));
        
//         std::cout<< "U1 "<<u1 << " reprojected "<<x<<" "<<y<<endl;
//         std::cout<<residual<<endl;
        //std::exit(0);
        return residual;
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
    Mat3 _P1,_P2;
    Vec3 _p1,_p2;
    T* data;





};

#endif