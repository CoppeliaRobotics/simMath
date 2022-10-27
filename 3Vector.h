#pragma once

#include "mathDefines.h"

class C3X3Matrix;
class C4X4Matrix;
class C7Vector;

class C3Vector  
{
public:

    C3Vector();
    C3Vector(simMathReal v0,simMathReal v1,simMathReal v2);
    C3Vector(const simMathReal v[3]);
    C3Vector(const C3Vector& v);
    ~C3Vector();

    void buildInterpolation(const C3Vector& fromThis,const C3Vector& toThat,simMathReal t);
    simMathReal getAngle(const C3Vector& v) const;
    C3X3Matrix getProductWithStar() const;
    simMathReal* ptr();
    bool isColinear(const C3Vector& v,simMathReal precision) const;
    simMathReal getLength() const;
    void setData(const simMathReal v[3]);
    void getData(simMathReal v[3]) const;
    void setData(simMathReal v0,simMathReal v1,simMathReal v2);
    // avoid those:
    /*
    void set(const simMathReal v[3]);
    void get(simMathReal v[3]) const;
    void getInternalData(simMathReal d[3]) const;
    void setInternalData(const simMathReal d[3]);
    void copyTo(simMathReal v[3]) const;
    */
    // -----------
    C3Vector getNormalized() const;
    void keepMax(const C3Vector& v);
    void keepMin(const C3Vector& v);
    bool isValid() const;
    void normalize();
    void clear();

    simMathReal& operator() (size_t i);
    const simMathReal& operator() (size_t i) const;

    void operator*= (const C4X4Matrix& m);
    void operator*= (const C3X3Matrix& m);
    void operator*= (const C7Vector& transf);
    void operator*= (simMathReal d);
    void operator/= (simMathReal d);
    void operator+= (const C3Vector& v);
    void operator-= (const C3Vector& v);
    void operator^= (const C3Vector& v);

    C3Vector& operator= (const C3Vector& v);
    bool operator!= (const C3Vector& v);
    C3Vector operator* (simMathReal d) const;
    simMathReal operator* (const C3Vector& v) const;
    C3Vector operator/ (simMathReal d) const;
    C3Vector operator+ (const C3Vector& v) const;
    C3Vector operator- (const C3Vector& v) const;
    C3Vector operator^ (const C3Vector& v) const;

    static const C3Vector unitXVector;
    static const C3Vector unitYVector;
    static const C3Vector unitZVector;
    static const C3Vector zeroVector;

    simMathReal data[3];
};




