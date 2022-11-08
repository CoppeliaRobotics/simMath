#pragma once

#include "mathDefines.h"
#include "3Vector.h"
#include "3X3Matrix.h"

class C4Vector  
{
public:
    C4Vector();
    C4Vector(simMathReal w,simMathReal x,simMathReal y,simMathReal z);
    C4Vector(const simMathReal wxyz[4],bool xyzwLayout=false);
    C4Vector(const C3Vector& v);
    C4Vector(const C4Vector& q);
    C4Vector(simMathReal a,simMathReal b,simMathReal g);
    C4Vector(simMathReal angle,const C3Vector& axis);
    C4Vector(const C3Vector& startV,const C3Vector& endV);
    ~C4Vector();

    void setEulerAngles(const C3Vector& v);
    void setEulerAngles(simMathReal a,simMathReal b,simMathReal g);
    void setAngleAndAxis(simMathReal angle,const C3Vector& axis);
    void setVectorMapping(const C3Vector& startV,const C3Vector& endV);
    void buildInterpolation(const C4Vector& fromThis,const C4Vector& toThat,simMathReal t);
    void buildInterpolation_otherWayRound(const C4Vector& fromThis,const C4Vector& toThat,simMathReal t);
    void buildRandomOrientation();

    simMathReal getAngleBetweenQuaternions(const C4Vector& q) const;
    C4Vector getAngleAndAxis() const;
    C3Vector getAngleAndAxis(simMathReal& angle) const;
    C3Vector getAxis(size_t index) const;
    C3X3Matrix getMatrix() const;
    C3Vector getEulerAngles() const;
    void getData(simMathReal wxyz[4],bool xyzwLayout=false) const;
    void setData(const simMathReal wxyz[4],bool xyzwLayout=false);
    void normalize();
    void clear();
    void setIdentity();
    C4Vector getInverse() const;
    void inverse();

    simMathReal& operator() (size_t i);
    const simMathReal& operator() (size_t i) const;

    void operator*= (simMathReal d);
    void operator*= (const C4Vector& v);
    void operator/= (simMathReal d);
    void operator+= (const C4Vector& v);

    C4Vector& operator= (const C4Vector& v);
    bool operator!= (const C4Vector& v);
    C4Vector operator* (const C4Vector& v) const;
    C3Vector operator* (const C3Vector& v) const;
    C4Vector operator* (simMathReal d) const;
    C4Vector operator/ (simMathReal d) const;
    C4Vector operator+ (const C4Vector& v) const;

    static const C4Vector identityRotation;

    simMathReal data[4]; // quaternion storage: wxyz
};
