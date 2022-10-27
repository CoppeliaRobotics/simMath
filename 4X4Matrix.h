#pragma once

#include "mathDefines.h"
#include "3Vector.h"
#include "4Vector.h"
#include "7Vector.h"
#include "3X3Matrix.h"

class CMatrix;

class C4X4Matrix  
{
public:

    C4X4Matrix();
    C4X4Matrix(const C4X4Matrix& m);
    C4X4Matrix(const CMatrix& m);
    C4X4Matrix(const C3X3Matrix& m,const C3Vector& x);
    C4X4Matrix(const C3Vector& x,const C3Vector& y,const C3Vector& z,const C3Vector& pos);
    C4X4Matrix(const C7Vector& transf);
    ~C4X4Matrix();

    void setIdentity();
    void buildInterpolation(const C4X4Matrix& fromThis,const C4X4Matrix& toThat,simMathReal t);
    C7Vector getTransformation() const;
    void buildXRotation(simMathReal angle);
    void buildYRotation(simMathReal angle);
    void buildZRotation(simMathReal angle);
    void buildTranslation(simMathReal x,simMathReal y,simMathReal z);
    void rotateAroundX(simMathReal angle);
    void rotateAroundY(simMathReal angle);
    void rotateAroundZ(simMathReal angle);
    void translate(simMathReal x,simMathReal y,simMathReal z);
    void inverse();
    void getData(simMathReal* m) const;
    void setData(const simMathReal* m);
    // Avoid using these:
    /*
    void get(simMathReal m[4][4]) const;
    void set(const simMathReal m[4][4]);
    void get(simMathReal* m) const;
    void set(const simMathReal* m);
    void copyTo(simMathReal m[4][4]) const;
    void copyToInterface(simMathReal* m) const;
    void copyFromInterface(const simMathReal* m);
//*/
    //--------------------------
    bool isValid() const;
    C4X4Matrix getInverse() const;
    void setMultResult(const C4X4Matrix& m1,const C4X4Matrix& m2);

    C4X4Matrix& operator= (const CMatrix& m);
    C4X4Matrix& operator= (const C4X4Matrix& m);
    void operator*= (const C4X4Matrix& m);
    C4X4Matrix operator* (const CMatrix& m) const;
    C4X4Matrix operator* (const C4X4Matrix& m) const;
    C3Vector operator* (const C3Vector& v) const;

    C3X3Matrix M;
    C3Vector X;
};
