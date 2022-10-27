#pragma once

#include "mathDefines.h"
#include "3Vector.h"
#include "4Vector.h"

class C4X4Matrix;

class C7Vector  
{
public:
    C7Vector();
    C7Vector(const C7Vector& v);
    C7Vector(const C4Vector& q);
    C7Vector(const C3Vector& x);
    C7Vector(const C4Vector& q,const C3Vector& x);
    C7Vector(const C4X4Matrix& m);
    C7Vector(simMathReal angle,const C3Vector& pos,const C3Vector& dir);
    ~C7Vector();

    void setIdentity();
    C4X4Matrix getMatrix() const;
    C7Vector getInverse() const;
    void setMultResult(const C7Vector& v1,const C7Vector& v2);
    void buildInterpolation(const C7Vector& fromThis,const C7Vector& toThat,simMathReal t);
    void inverse();
    C3Vector getAxis(size_t index) const;
    void getData(simMathReal d[7],bool xyzwLayout=false) const;
    void setData(const simMathReal d[7],bool xyzwLayout=false);
    void setFromMatrix(const C4X4Matrix& m);
    // Avoid using those:
    /*
    void set(simMathReal m[4][4]);
    void copyTo(simMathReal m[4][4]) const;
    void get(simMathReal d[7],bool xyzwLayout=false) const;
    void set(const simMathReal d[7],bool xyzwLayout=false);
    void getInternalData(simMathReal d[7],bool xyzwLayout=false) const;
    void setInternalData(const simMathReal d[7],bool xyzwLayout=false);
    */
    //----------------------

    simMathReal& operator() (size_t i);
    const simMathReal& operator() (size_t i) const;

    void operator*= (const C7Vector& v);

    C7Vector& operator= (const C7Vector& v);
    bool operator!= (const C7Vector& v);
    C7Vector operator* (const C7Vector& v) const;
    C3Vector operator* (const C3Vector& v) const;

    static const C7Vector identityTransformation;
    
    C4Vector Q;
    C3Vector X;
};
