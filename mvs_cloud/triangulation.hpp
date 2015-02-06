




class Triangulation
{





// HZ 12.2 pag.312
void TriangulateDLT(const Mat34 &P1, const Vec2 &x1,
                    const Mat34 &P2, const Vec2 &x2,
                    Vec4 *X_homogeneous) {
  Mat4 design;
  for (int i = 0; i < 4; ++i) {
    design(0,i) = x1[0] * P1(2,i) - P1(0,i);
    design(1,i) = x1[1] * P1(2,i) - P1(1,i);
    design(2,i) = x2[0] * P2(2,i) - P2(0,i);
    design(3,i) = x2[1] * P2(2,i) - P2(1,i);
  }
  Nullspace(&design, X_homogeneous);
}

void TriangulateDLT(const Mat34 &P1, const Vec2 &x1,
                    const Mat34 &P2, const Vec2 &x2,
                    Vec3 *X_euclidean) {
    
  Vec4 X_homogeneous;
  TriangulateDLT(P1, x1, P2, x2, &X_homogeneous);
 // HomogeneousToEuclidean(X_homogeneous, X_euclidean);
}

/**
 From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
 */
// Mat_ LinearLSTriangulation(Point3d u,       //homogenous image point (u,v,1)
//                    Matx34d P,       //camera 1 matrix
//                    Point3d u1,      //homogenous image point in 2nd camera
//                    Matx34d P1       //camera 2 matrix
//                                    )
// {
//     //build matrix A for homogenous equation system Ax = 0
//     //assume X = (x,y,z,1), for Linear-LS method
//     //which turns it into a AX = B system, where A is 4x3, X is 3x1 and B is 4x1
//     Matx43d A(u.x*P(2,0)-P(0,0),    u.x*P(2,1)-P(0,1),      u.x*P(2,2)-P(0,2),
//           u.y*P(2,0)-P(1,0),    u.y*P(2,1)-P(1,1),      u.y*P(2,2)-P(1,2),
//           u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1),   u1.x*P1(2,2)-P1(0,2),
//           u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1),   u1.y*P1(2,2)-P1(1,2)
//               );
//     Mat_ B = (Mat_(4,1) <<    -(u.x*P(2,3)    -P(0,3)),
//                       -(u.y*P(2,3)  -P(1,3)),
//                       -(u1.x*P1(2,3)    -P1(0,3)),
//                       -(u1.y*P1(2,3)    -P1(1,3)));
//  
//     Mat_ X;
//     solve(A,B,X,DECOMP_SVD);
//  
//     return X;
// }


};