


class fundamental{



public :


    fundamental(const Mat34 & P1){}
    ~fundamental(){}
    
        
        
        
    void EssentialFromFundamental(const Mat3 &F,
                                const Mat3 &K1,
                                const Mat3 &K2,
                                Mat3 *E) {
    *E = K2.transpose() * F * K1;
    }

    // HZ 9.6 pag 257 (formula 9.12)
    // Or http://ai.stanford.edu/~birch/projective/node20.html
    void FundamentalFromEssential(const Mat3 &E,
                                const Mat3 &K1,
                                const Mat3 &K2,
                                Mat3 *F)  {
    *F = K2.inverse().transpose() * E * K1.inverse();
    }


    // HZ 9.6 pag 257
    void EssentialFromRt(const Mat3 &R1,
                        const Vec3 &t1,
                        const Mat3 &R2,
                        const Vec3 &t2,
                        Mat3 *E) {
    Mat3 R;
    Vec3 t;
    RelativeCameraMotion(R1, t1, R2, t2, &R, &t);
    Mat3 Tx = CrossProductMatrix(t);
    *E = Tx * R;
    }



};