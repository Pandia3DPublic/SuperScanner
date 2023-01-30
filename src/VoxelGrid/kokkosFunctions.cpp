#include "kokkosFunctions.h"


//#################### integer ####################


std::ostream & operator << (std::ostream &out, const Kvec3i &v)
{
    out << "Kvec3i " << v.x << " " << v.y <<" " << v.z;
    return out;
}

//#################### float ####################



std::ostream & operator << (std::ostream &out, const Kvec3f &v)
{
    out << "x :" << v.x << " y :" << v.y <<" z :" << v.z << std::endl; ;
    return out;
}
