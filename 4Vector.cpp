#include "4Vector.h"
#include "MyMath.h"

const C4Vector C4Vector::identityRotation(simOne,simZero,simZero,simZero);

C4Vector::C4Vector()
{
}

C4Vector::C4Vector(simMathReal w,simMathReal x,simMathReal y,simMathReal z)
{
    data[0]=w;
    data[1]=x;
    data[2]=y;
    data[3]=z;
    // We don't normalize here
}

C4Vector::C4Vector(const simMathReal wxyz[4],bool xyzwLayout/*=false*/)
{
    if (xyzwLayout)
    {
        data[0]=wxyz[3];
        data[1]=wxyz[0];
        data[2]=wxyz[1];
        data[3]=wxyz[2];
    }
    else
    {
        data[0]=wxyz[0];
        data[1]=wxyz[1];
        data[2]=wxyz[2];
        data[3]=wxyz[3];
    }
    // We don't normalize here
}

C4Vector::C4Vector(const C4Vector& q)
{
    data[0]=q(0);
    data[1]=q(1);
    data[2]=q(2);
    data[3]=q(3);
    // We don't normalize here
}

C4Vector::C4Vector(const C3Vector& v)
{ // Alpha, beta and gamma are in radians!
    setEulerAngles(v);
}

C4Vector::C4Vector(simMathReal a,simMathReal b,simMathReal g)
{ // Alpha, beta and gamma are in radians!
    setEulerAngles(a,b,g);
}

C4Vector::C4Vector(simMathReal angle,const C3Vector& axis)
{ // Builds a rotation quaternion around axis (angle in radian!)
    setAngleAndAxis(angle,axis);
}

C4Vector::C4Vector(const C3Vector& startV,const C3Vector& endV)
{
    setVectorMapping(startV,endV);
}

C4Vector::~C4Vector()
{

}

void C4Vector::setEulerAngles(simMathReal a,simMathReal b,simMathReal g)
{ // a,b anf g are in radian!
    C4Vector vx(a,C3Vector(simOne,simZero,simZero));
    C4Vector vy(b,C3Vector(simZero,simOne,simZero));
    C4Vector vz(g,C3Vector(simZero,simZero,simOne));
    (*this)=vx*vy*vz;
}

void C4Vector::setEulerAngles(const C3Vector& v)
{ // v(0), v(1) and v(2) are in radian!
    setEulerAngles(v(0),v(1),v(2));
}

void C4Vector::setAngleAndAxis(simMathReal angle,const C3Vector& axis)
{ // angle in radian!
    C3Vector axisTmp=axis;
    axisTmp.normalize();
    simMathReal sinA=sin(angle/simTwo);
    data[1]=axisTmp(0)*sinA;
    data[2]=axisTmp(1)*sinA;
    data[3]=axisTmp(2)*sinA;
    data[0]=cos(angle/simTwo);
}

void C4Vector::setVectorMapping(const C3Vector& startV,const C3Vector& endV)
{
    C3Vector v0(startV.getNormalized());
    C3Vector v1(endV.getNormalized());
    C3Vector cross(v0^v1);
    simMathReal cosAngle=v0*v1;
    if (cosAngle>simOne)
        setIdentity();
    else
        setAngleAndAxis(CMath::robustAcos(cosAngle),cross);
}

C3Vector C4Vector::getAngleAndAxis(simMathReal& angle) const
{
    C3Vector retV;
    C4Vector d(*this);
    if (d(0)<simZero)
        d=d*-simOne;
    simMathReal l=sqrt(d(0)*d(0)+d(1)*d(1)+d(2)*d(2)+d(3)*d(3));
    simMathReal cosA=d(0)/l; // Quaternion needs to be normalized
    if (cosA>simOne) // Just make sure..
        cosA=simOne;
    angle=CMath::robustAcos(cosA)*simTwo;
    simMathReal sinA=sqrt(simOne-cosA*cosA);
    if (fabs(sinA)<simMathReal(0.00005))
        sinA=simOne;
    else
        sinA*=l; // Quaternion needs to be normalized
    retV(0)=d(1)/sinA;
    retV(1)=d(2)/sinA;
    retV(2)=d(3)/sinA;
    return(retV);
}

C4Vector C4Vector::getAngleAndAxis() const
{ // Returned vector is (angle,x,y,z) (angle is in radians)
    simMathReal angle;
    C3Vector a(getAngleAndAxis(angle));
    C4Vector retVal(angle,a(0),a(1),a(2));
    return(retVal);
}

C3Vector C4Vector::getEulerAngles() const
{
    return(getMatrix().getEulerAngles());
}

simMathReal C4Vector::getAngleBetweenQuaternions(const C4Vector& q) const
{
    simMathReal angle=fabs(data[0]*q(0)+data[1]*q(1)+data[2]*q(2)+data[3]*q(3));
    return(CMath::robustAcos(angle)*simTwo);
}

void C4Vector::buildInterpolation(const C4Vector& fromThis,const C4Vector& toThat,simMathReal t)
{
    C4Vector AA(fromThis);
    C4Vector BB(toThat);
    if (AA(0)*BB(0)+AA(1)*BB(1)+AA(2)*BB(2)+AA(3)*BB(3)<simZero)
        AA=AA*-simOne;
    C4Vector r((AA.getInverse()*BB).getAngleAndAxis());
    (*this)=(AA*C4Vector(r(0)*t,C3Vector(r(1),r(2),r(3))));
    // Already normalized through * operator
}

void C4Vector::buildInterpolation_otherWayRound(const C4Vector& fromThis,const C4Vector& toThat,simMathReal t)
{
    C4Vector AA(fromThis);
    C4Vector BB(toThat);
    if (AA(0)*BB(0)+AA(1)*BB(1)+AA(2)*BB(2)+AA(3)*BB(3)<simZero)
        AA=AA*-simOne;
    C4Vector r((AA.getInverse()*BB).getAngleAndAxis());

    // r(0) is the rotation angle
    // r(1),r(2),r(3) is the rotation axis
    // Here, since we want to rotate the other way round, we inverse the axis and rotate by 2*pi-r(0) instead:
    (*this)=(AA*C4Vector((piValTimes2-r(0))*t,C3Vector(r(1)*-simOne,r(2)*-simOne,r(3)*-simOne)));
    // Already normalized through * operator
}

void C4Vector::buildRandomOrientation()
{
    C3Vector u(SIM_RAND_FLOAT,SIM_RAND_FLOAT,SIM_RAND_FLOAT);
    data[0]=sqrt(simOne-u(0))*sin(piValTimes2*u(1));
    data[1]=sqrt(simOne-u(0))*cos(piValTimes2*u(1));
    data[2]=sqrt(u(0))*sin(piValTimes2*u(2));
    data[3]=sqrt(u(0))*cos(piValTimes2*u(2));
}

void C4Vector::getData(simMathReal wxyz[4],bool xyzwLayout/*=false*/) const
{
    if (xyzwLayout)
    {
        wxyz[3]=data[0];
        wxyz[0]=data[1];
        wxyz[1]=data[2];
        wxyz[2]=data[3];
    }
    else
    {
        wxyz[0]=data[0];
        wxyz[1]=data[1];
        wxyz[2]=data[2];
        wxyz[3]=data[3];
    }
}

void C4Vector::setData(const simMathReal wxyz[4],bool xyzwLayout/*=false*/)
{
    if (xyzwLayout)
    {
        data[0]=wxyz[3];
        data[1]=wxyz[0];
        data[2]=wxyz[1];
        data[3]=wxyz[2];
    }
    else
    {
        data[0]=wxyz[0];
        data[1]=wxyz[1];
        data[2]=wxyz[2];
        data[3]=wxyz[3];
    }
}

simMathReal& C4Vector::operator() (size_t i)
{
    return(data[i]);
}

const simMathReal& C4Vector::operator() (size_t i) const
{
    return(data[i]);
}

void C4Vector::normalize()
{
    simMathReal l=sqrt(data[0]*data[0]+data[1]*data[1]+data[2]*data[2]+data[3]*data[3]);
    data[0]/=l;
    data[1]/=l;
    data[2]/=l;
    data[3]/=l;
}

void C4Vector::clear()
{
    data[0]=simZero;
    data[1]=simZero;
    data[2]=simZero;
    data[3]=simZero;
}

void C4Vector::setIdentity()
{
    data[0]=simOne;
    data[1]=simZero;
    data[2]=simZero;
    data[3]=simZero;
}

C4Vector C4Vector::getInverse() const
{
    return(C4Vector(data[0],-data[1],-data[2],-data[3]));
}

void C4Vector::inverse()
{
    data[1]=-data[1];
    data[2]=-data[2];
    data[3]=-data[3];
}

C4Vector C4Vector::operator/ (simMathReal d) const
{
    C4Vector retV;
    retV(0)=data[0]/d;
    retV(1)=data[1]/d;
    retV(2)=data[2]/d;
    retV(3)=data[3]/d;
    return(retV);
}

C4Vector C4Vector::operator* (simMathReal d) const
{
    C4Vector retV;
    retV(0)=data[0]*d;
    retV(1)=data[1]*d;
    retV(2)=data[2]*d;
    retV(3)=data[3]*d;
    return(retV);
}

C4Vector& C4Vector::operator= (const C4Vector& v)
{
    data[0]=v(0);
    data[1]=v(1);
    data[2]=v(2);
    data[3]=v(3);
    return(*this);
}

bool C4Vector::operator!= (const C4Vector& v)
{
    return( (data[0]!=v(0))||(data[1]!=v(1))||(data[2]!=v(2))||(data[3]!=v(3)) );
}

C4Vector C4Vector::operator* (const C4Vector& v) const
{ // Quaternion multiplication.
    C4Vector retV;
    retV(0)=data[0]*v(0)-data[1]*v(1)-data[2]*v(2)-data[3]*v(3);
    retV(1)=data[0]*v(1)+v(0)*data[1]+data[2]*v(3)-data[3]*v(2);
    retV(2)=data[0]*v(2)+v(0)*data[2]+data[3]*v(1)-data[1]*v(3);
    retV(3)=data[0]*v(3)+v(0)*data[3]+data[1]*v(2)-data[2]*v(1);
    //  retV.normalize(); // NOOOOOOO!!!!!! We might compute the rotation of a vector which should be (q*v*qI).normalize and not q*((v*qi).normalize).normalize !!
    return(retV);
}

C3Vector C4Vector::getAxis(size_t index) const
{
    C3X3Matrix m=getMatrix();
    return(m.axis[index]);
}

C3Vector C4Vector::operator* (const C3Vector& v) const
{ // Rotation of a vector.
    C4Vector tmpV(simOne,v(0),v(1),v(2));
    tmpV=(*this)*(tmpV*getInverse());
    return(C3Vector(tmpV(1),tmpV(2),tmpV(3)));
}

C4Vector C4Vector::operator+ (const C4Vector& v) const
{
    C4Vector retV;
    retV(0)=data[0]+v(0);
    retV(1)=data[1]+v(1);
    retV(2)=data[2]+v(2);
    retV(3)=data[3]+v(3);
    return(retV);
}

C3X3Matrix C4Vector::getMatrix() const
{
    C3X3Matrix retM;
    simMathReal xx=data[1]*data[1];
    simMathReal xy=data[1]*data[2];
    simMathReal xz=data[1]*data[3];
    simMathReal xw=data[1]*data[0];
    simMathReal yy=data[2]*data[2];
    simMathReal yz=data[2]*data[3];
    simMathReal yw=data[2]*data[0];
    simMathReal zz=data[3]*data[3];
    simMathReal zw=data[3]*data[0];

    retM(0,0)=simOne-simTwo*(yy+zz);
    retM(0,1)=simTwo*(xy-zw);
    retM(0,2)=simTwo*(xz+yw);
    retM(1,0)=simTwo*(xy+zw);
    retM(1,1)=simOne-simTwo*(xx+zz);
    retM(1,2)=simTwo*(yz-xw);
    retM(2,0)=simTwo*(xz-yw);
    retM(2,1)=simTwo*(yz+xw);
    retM(2,2)=simOne-simTwo*(xx+yy);
    return(retM);
}

void C4Vector::operator/= (simMathReal d)
{
    data[0]/=d;
    data[1]/=d;
    data[2]/=d;
    data[3]/=d;
}

void C4Vector::operator*= (simMathReal d)
{
    data[0]*=d;
    data[1]*=d;
    data[2]*=d;
    data[3]*=d;
}

void C4Vector::operator*= (const C4Vector& v)
{
    (*this)=(*this)*v;
    // Already normalized through * operator
}

void C4Vector::operator+= (const C4Vector& v)
{
    data[0]+=v(0);
    data[1]+=v(1);
    data[2]+=v(2);
    data[3]+=v(3);
}

/*
 * void C4Vector::get(simMathReal wxyz[4],bool xyzwLayout) const
{ // Avoid using this. Use get/setData instead
    getData(wxyz,xyzwLayout);
}

void C4Vector::set(const simMathReal wxyz[4],bool xyzwLayout)
{ // Avoid using this. Use get/setData instead
    setData(wxyz,xyzwLayout);
}

void C4Vector::getInternalData(simMathReal wxyz[4],bool xyzwLayout) const
{ // Avoid using this. Use get/setData instead
    getData(wxyz,xyzwLayout);
}

void C4Vector::setInternalData(const simMathReal wxyz[4],bool xyzwLayout)
{ // Avoid using this. Use get/setData instead
    setData(wxyz,xyzwLayout);
}
*/
