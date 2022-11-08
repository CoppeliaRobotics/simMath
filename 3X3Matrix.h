#pragma once

#include "mathDefines.h"
#include "3Vector.h"

class C4Vector;

class C3X3Matrix  
{
public:
    C3X3Matrix();
    C3X3Matrix(const C4Vector& q);
    C3X3Matrix(const C3X3Matrix& m);
    C3X3Matrix(const C3Vector& xAxis,const C3Vector& yAxis,const C3Vector& zAxis);
    ~C3X3Matrix();

    void buildInterpolation(const C3X3Matrix& fromThis,const C3X3Matrix& toThat,simMathReal t);
    C4Vector getQuaternion() const;
    void setEulerAngles(simMathReal a,simMathReal b,simMathReal g);
    void setEulerAngles(const C3Vector& v);
    C3Vector getEulerAngles() const;
    void buildXRotation(simMathReal angle);
    void buildYRotation(simMathReal angle);
    void buildZRotation(simMathReal angle);
    C3Vector getNormalVector() const;

    void clear();
    void setIdentity();
    void transpose();
    void setAxes(const C3Vector& xAxis,const C3Vector& yAxis,const C3Vector& zAxis);
    void getData(simMathReal d[9]) const;
    void setData(const simMathReal d[9]);
    bool isValid() const;
    C3X3Matrix getTranspose() const;

    simMathReal& operator() (size_t i,size_t j);
    const simMathReal& operator() (size_t i,size_t j) const;

    void operator*= (const C3X3Matrix& m);
    void operator*= (simMathReal f);
    void operator/= (simMathReal f);
    void operator+= (const C3X3Matrix& m);
    void operator-= (const C3X3Matrix& m);

    C3X3Matrix& operator= (const C3X3Matrix& m);
    C3X3Matrix operator* (const C3X3Matrix& m) const;
    C3Vector operator* (const C3Vector& v) const;
    C3X3Matrix operator* (simMathReal f) const;
    C3X3Matrix operator/ (simMathReal f) const;
    C3X3Matrix operator+ (const C3X3Matrix& m) const;
    C3X3Matrix operator- (const C3X3Matrix& m) const;

    C3Vector axis[3];
};
