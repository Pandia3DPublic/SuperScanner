#pragma once
//todo template instantiate this out of one class
class Kvec3f;
class Kvec3i;
class Kvec3b;
class Kvec4f;
class Kmat3f;
class Kmat4f;


class Kvec3b{
public:
    unsigned char x, y, z;

    KOKKOS_INLINE_FUNCTION
    Kvec3b(){};

    KOKKOS_INLINE_FUNCTION
    ~Kvec3b(){};

    KOKKOS_INLINE_FUNCTION
    Kvec3b(unsigned char x_, unsigned char y_, unsigned char z_)
    {
        x = x_;
        y = y_;
        z = z_;
    }

    KOKKOS_INLINE_FUNCTION
    void set(unsigned char x_, unsigned char y_, unsigned char z_)
    {
        x = x_;
        y = y_;
        z = z_;
    }

    KOKKOS_INLINE_FUNCTION
    void operator=(Kvec3b b)
    {
        set(b.x, b.y, b.z);
    }

    KOKKOS_INLINE_FUNCTION
    bool operator==(Kvec3b b)
    {
        return (x == b.x && y == b.y && z == b.z);
    }

    KOKKOS_INLINE_FUNCTION
    bool operator!=(Kvec3b b)
    {
        return (x != b.x || y != b.y || z != b.z);
    }

    KOKKOS_INLINE_FUNCTION
    Kvec3b operator*(unsigned char a)
    {
        return Kvec3b(a * x, a * y, a * z);
    }

    KOKKOS_INLINE_FUNCTION
    Kvec3b operator-(Kvec3b a)
    {
        return Kvec3b(x - a.x, y - a.y, z - a.z);
    }

    KOKKOS_INLINE_FUNCTION
    Kvec3b operator+(Kvec3b a)
    {
        return Kvec3b(x + a.x, y + a.y, z + a.z);
    }
    KOKKOS_INLINE_FUNCTION
    Kvec3b operator/(unsigned char a)
    {
        return Kvec3b(x / a, y / a, z / a);
    }

    KOKKOS_INLINE_FUNCTION
    void print(const char* name = ""){
        printf("%s(%i, %i, %i)\n", name, x, y, z);
    }
};

KOKKOS_INLINE_FUNCTION
Kvec3b operator*(float a, Kvec3b b)
{
    return Kvec3b(a * b.x, a * b.y, a * b.z);
}

class Kvec3i{
public:

    int x,y,z;

    KOKKOS_INLINE_FUNCTION 
    Kvec3i(){};

    KOKKOS_INLINE_FUNCTION 
    ~Kvec3i(){};


    KOKKOS_INLINE_FUNCTION
    Kvec3i(int x_,int y_,int z_){
        x = x_;
        y = y_;
        z = z_;
    }

    KOKKOS_INLINE_FUNCTION
    void set(int x_,int y_,int z_){
        x = x_;
        y = y_;
        z = z_;
    }

    KOKKOS_INLINE_FUNCTION
    void operator=(Kvec3i b){
        set(b.x,b.y,b.z);
    }

    //todo test
    KOKKOS_INLINE_FUNCTION
    bool operator==(Kvec3i b){
        return(x==b.x && y==b.y && z == b.z);
    }

    KOKKOS_INLINE_FUNCTION
    bool operator!=(Kvec3i b)
    {
        return (x != b.x || y != b.y || z != b.z);
    }

    friend std::ostream & operator << (std::ostream &out, const Kvec3i &v);

    KOKKOS_INLINE_FUNCTION
    void print(const char* name = ""){
        printf("%s(%i, %i, %i)\n", name, x, y, z);
    }

};



class Kvec3f{
public:
    float x,y,z;

    KOKKOS_INLINE_FUNCTION 
    Kvec3f(){};

    KOKKOS_INLINE_FUNCTION 
    ~Kvec3f(){};

    KOKKOS_INLINE_FUNCTION
    Kvec3f(float x_,float y_,float z_){
        x = x_;
        y = y_;
        z = z_;
    }

    KOKKOS_INLINE_FUNCTION
    void set(float x_,float y_,float z_){
        x = x_;
        y = y_;
        z = z_;
    }

    KOKKOS_INLINE_FUNCTION
    float length(){
        return sqrt(x*x+y*y+z*z);
    }


    KOKKOS_INLINE_FUNCTION
    void normalize(){
        float tmp = length();
        x /=tmp;
        y /=tmp;
        z /=tmp;
    }

    KOKKOS_INLINE_FUNCTION
    void operator=(Kvec3f b){
        set(b.x,b.y,b.z);
    }

    KOKKOS_INLINE_FUNCTION
    Kvec3f operator/(float a){
        return Kvec3f(x/a,y/a,z/a);
    }

    KOKKOS_INLINE_FUNCTION
    Kvec3f operator*(float a){ 
        return Kvec3f(a*x, a*y, a*z);
    }

    KOKKOS_INLINE_FUNCTION
    Kvec3f operator-(Kvec3f a){
        return Kvec3f(x-a.x,y-a.y,z-a.z);
    }


    KOKKOS_INLINE_FUNCTION
    Kvec3f operator+(Kvec3f a){
        return Kvec3f(x+a.x,y+a.y, z+a.z);
        
    }



    KOKKOS_INLINE_FUNCTION
    float operator*(Kvec3f a){ //scalar product
        return x*a.x + y*a.y + z*a.z;
    }


//todo maybe make these return *this
    KOKKOS_INLINE_FUNCTION
    void operator+=(Kvec3f a){
        x+=a.x;
        y+=a.y;
        z+=a.z;
    }

    KOKKOS_INLINE_FUNCTION
    void operator-=(Kvec3f a){
        x-=a.x;
        y-=a.y;
        z-=a.z;
    }

    KOKKOS_INLINE_FUNCTION
    void operator*=(float a){
        x*=a;
        y*=a;
        z*=a;
    }

    KOKKOS_INLINE_FUNCTION
    Kvec3f cross(Kvec3f b)
    {
        Kvec3f tmp;
        tmp.x = y * b.z - z * b.y;
        tmp.y = z * b.x - x * b.z;
        tmp.z = x * b.y - y * b.x;
        return tmp;
    }

    Eigen::Vector3d toEigen(){
        return Eigen::Vector3d(x,y,z);
    }

    Eigen::Vector4d toEigen4()
    {
        return Eigen::Vector4d(x, y, z,1.0);
    }
    friend std::ostream & operator << (std::ostream &out, const Kvec3f &v);

    KOKKOS_INLINE_FUNCTION
    void print(const char* name = ""){
        printf("%s(%.9g, %.9g, %.9g)\n", name, x, y, z);
    }
};

KOKKOS_INLINE_FUNCTION
Kvec3f operator*(float a, Kvec3f b){ 
    return Kvec3f(a*b.x, a*b.y, a*b.z);
}




class Kvec4f{
public:
    float x,y,z,w;

    KOKKOS_INLINE_FUNCTION 
    Kvec4f(){};

    KOKKOS_INLINE_FUNCTION 
    ~Kvec4f(){};

    KOKKOS_INLINE_FUNCTION
    Kvec4f(float x_,float y_,float z_, float w_){
        x = x_;
        y = y_;
        z = z_;
        w = w_;
    }

    KOKKOS_INLINE_FUNCTION
    void set(float x_,float y_,float z_, float w_){
        x = x_;
        y = y_;
        z = z_;
        w = w_;
    }

    KOKKOS_INLINE_FUNCTION
    float length(){
        return sqrt(x*x+y*y+z*z+w*w);
    }


    KOKKOS_INLINE_FUNCTION
    void normalize(){
        float tmp = length();
        x /=tmp;
        y /=tmp;
        z /=tmp;
        w /=tmp;
    }

    KOKKOS_INLINE_FUNCTION
    void operator=(Kvec4f b){
        set(b.x,b.y,b.z,b.w);
    }

    KOKKOS_INLINE_FUNCTION
    Kvec4f operator/(float a){
        return Kvec4f(x/a,y/a,z/a,w/a);
    }

    KOKKOS_INLINE_FUNCTION
    Kvec4f operator*(float a){ 
        return Kvec4f(a*x, a*y, a*z, a*w);
    }

    KOKKOS_INLINE_FUNCTION
    Kvec4f operator-(Kvec4f a){
        return Kvec4f(x-a.x,y-a.y,z-a.z,w-a.w);
    }


    KOKKOS_INLINE_FUNCTION
    Kvec4f operator+(Kvec4f a){
        return Kvec4f(x+a.x,y+a.y, z+a.z, w+a.w);
        
    }



    KOKKOS_INLINE_FUNCTION
    float operator*(Kvec4f a)const{ //scalar product
        return x*a.x + y*a.y + z*a.z +w*a.w;
    }


//todo maybe make these return *this
    KOKKOS_INLINE_FUNCTION
    void operator+=(Kvec4f a){
        x+=a.x;
        y+=a.y;
        z+=a.z;
        w+=a.w;
    }

    KOKKOS_INLINE_FUNCTION
    void operator-=(Kvec4f a){
        x-=a.x;
        y-=a.y;
        z-=a.z;
        w-=a.w;
    }

    KOKKOS_INLINE_FUNCTION
    void operator*=(float a){
        x*=a;
        y*=a;
        z*=a;
        w*=a;
    }

    // KOKKOS_INLINE_FUNCTION
    // Kvec4f cross(Kvec4f b)
    // {
    //     Kvec4f tmp;
    //     tmp.x = y * b.z - z * b.y;
    //     tmp.y = z * b.x - x * b.z;
    //     tmp.z = x * b.y - y * b.x;
    //     tmp.w = 1.0f;
    //     return tmp;
    // }

    KOKKOS_INLINE_FUNCTION
    void print(const char* name = ""){
        printf("%s(%.9g, %.9g, %.9g, %.9g)\n", name, x, y, z, w);
    }
};




class Kvec2f{
public:
    float x,y;

    KOKKOS_INLINE_FUNCTION 
    Kvec2f(){};

    KOKKOS_INLINE_FUNCTION 
    ~Kvec2f(){};

    KOKKOS_INLINE_FUNCTION
    Kvec2f(float x_,float y_){
        x = x_;
        y = y_;
    }

    KOKKOS_INLINE_FUNCTION
    void set(float x_,float y_){
        x = x_;
        y = y_;
    }

    KOKKOS_INLINE_FUNCTION
    float length(){
        return sqrt(x*x+y*y);
    }


    KOKKOS_INLINE_FUNCTION
    void normalize(){
        float tmp = length();
        x /=tmp;
        y /=tmp;
    }

    KOKKOS_INLINE_FUNCTION
    void operator=(Kvec2f b){
        set(b.x,b.y);
    }

    KOKKOS_INLINE_FUNCTION
    Kvec2f operator/(float a){
        return Kvec2f(x/a,y/a);
    }

    KOKKOS_INLINE_FUNCTION
    Kvec2f operator*(float a){ 
        return Kvec2f(a*x, a*y);
    }

    KOKKOS_INLINE_FUNCTION
    Kvec2f operator-(Kvec2f a){
        return Kvec2f(x-a.x,y-a.y);
    }


    KOKKOS_INLINE_FUNCTION
    Kvec2f operator+(Kvec2f a){
        return Kvec2f(x+a.x,y+a.y);
        
    }



    KOKKOS_INLINE_FUNCTION
    float operator*(Kvec2f a){ //scalar product
        return x*a.x + y*a.y;
    }


//todo maybe make these return *this
    KOKKOS_INLINE_FUNCTION
    void operator+=(Kvec2f a){
        x+=a.x;
        y+=a.y;
    }

    KOKKOS_INLINE_FUNCTION
    void operator-=(Kvec2f a){
        x-=a.x;
        y-=a.y;
    }

    friend std::ostream & operator << (std::ostream &out, const Kvec2f &v);

    KOKKOS_INLINE_FUNCTION
    void print(const char* name = ""){
        printf("%s(%.9g, %.9g)\n", name, x, y);
    }
};


class Kvec3d{
public:

    double x,y,z;
};

template <typename T>
void printlayout(T view_){
    if (view_.is_layout_left) {
      std::cout << "is layout left" << std::endl;
    }
    else {
      std::cout << "is layout right or stride" << std::endl;
    }
}

class Kmat3f {
    public:
    Kvec3f r1, r2, r3; //rows
    // float val[9];
    // float val[3][3];

    KOKKOS_INLINE_FUNCTION
    Kmat3f(){};

    KOKKOS_INLINE_FUNCTION
    ~Kmat3f(){};

    KOKKOS_INLINE_FUNCTION
    Kmat3f(Kvec3f r1_, Kvec3f r2_, Kvec3f r3_) {
        r1 = r1_;
        r2 = r2_;
        r3 = r3_;
    }

    KOKKOS_INLINE_FUNCTION
    void set(Kvec3f r1_, Kvec3f r2_, Kvec3f r3_){
        r1 = r1_;
        r2 = r2_;
        r3 = r3_;
    }

    KOKKOS_INLINE_FUNCTION
    void set(Kmat3f m){
        r1 = m.r1;
        r2 = m.r2;
        r3 = m.r3;
    }

    KOKKOS_INLINE_FUNCTION
    void setIdentity(){
        r1 = Kvec3f(1.0, 0.0, 0.0);
        r2 = Kvec3f(0.0, 1.0, 0.0);
        r3 = Kvec3f(0.0, 0.0, 1.0);
    }

    KOKKOS_INLINE_FUNCTION
    void operator=(Kmat3f m){
        set(m);
    }

    KOKKOS_INLINE_FUNCTION
    Kmat3f operator+(Kmat3f m){
        Kmat3f tmp;
        tmp.r1 = r1 + m.r1;
        tmp.r2 = r2 + m.r2;
        tmp.r3 = r3 + m.r3;
        return tmp;
    }

    KOKKOS_INLINE_FUNCTION
    Kmat3f operator-(Kmat3f m){
        Kmat3f tmp;
        tmp.r1 = r1 - m.r1;
        tmp.r2 = r2 - m.r2;
        tmp.r3 = r3 - m.r3;
        return tmp;
    }

    KOKKOS_INLINE_FUNCTION
    void operator*=(float f){
        r1 *= f;
        r2 *= f;
        r3 *= f;
    }

    KOKKOS_INLINE_FUNCTION
    Kmat3f operator*(Kmat3f m){
        Kmat3f tmp;
        Kvec3f c1 = Kvec3f(m.r1.x, m.r2.x, m.r3.x);
        Kvec3f c2 = Kvec3f(m.r1.y, m.r2.y, m.r3.y);
        Kvec3f c3 = Kvec3f(m.r1.z, m.r2.z, m.r3.z);
        tmp.r1 = Kvec3f(r1*c1, r1*c2, r1*c3);
        tmp.r2 = Kvec3f(r2*c1, r2*c2, r2*c3);
        tmp.r3 = Kvec3f(r3*c1, r3*c2, r3*c3);
        return tmp;
    }

    KOKKOS_INLINE_FUNCTION
    Kvec3f operator*(Kvec3f v){
        Kvec3f tmp;
        tmp.x = r1 * v;
        tmp.y = r2 * v;
        tmp.z = r3 * v;
        return tmp;
    }

    KOKKOS_INLINE_FUNCTION
    void print(const char* name = ""){
        r1.print(name);
        r2.print();
        r3.print();
    }
};


class Kmat4f {
    public:
    Kvec4f r1, r2, r3, r4; //rows

    KOKKOS_INLINE_FUNCTION
    Kmat4f(){};

    KOKKOS_INLINE_FUNCTION
    ~Kmat4f(){};

    KOKKOS_INLINE_FUNCTION
    Kmat4f(Kvec4f r1_, Kvec4f r2_, Kvec4f r3_, Kvec4f r4_) {
        r1 = r1_;
        r2 = r2_;
        r3 = r3_;
        r4 = r4_;
    }

    KOKKOS_INLINE_FUNCTION
    void set(Kvec4f r1_, Kvec4f r2_, Kvec4f r3_, Kvec4f r4_){
        r1 = r1_;
        r2 = r2_;
        r3 = r3_;
        r4 = r4_;
    }

    KOKKOS_INLINE_FUNCTION
    void set(Kmat4f m){
        r1 = m.r1;
        r2 = m.r2;
        r3 = m.r3;
        r4 = m.r4;
    }

    KOKKOS_INLINE_FUNCTION
    void setIdentity(){
        r1 = Kvec4f(1.0, 0.0, 0.0, 0.0);
        r2 = Kvec4f(0.0, 1.0, 0.0, 0.0);
        r3 = Kvec4f(0.0, 0.0, 1.0, 0.0);
        r4 = Kvec4f(0.0, 0.0, 0.0, 1.0);
    }

    KOKKOS_INLINE_FUNCTION
    void operator=(Kmat4f m){
        set(m);
    }

    KOKKOS_INLINE_FUNCTION
    Kmat4f operator+(Kmat4f m){
        Kmat4f tmp;
        tmp.r1 = r1 + m.r1;
        tmp.r2 = r2 + m.r2;
        tmp.r3 = r3 + m.r3;
        tmp.r4 = r4 + m.r4;
        return tmp;
    }

    KOKKOS_INLINE_FUNCTION
    Kmat4f operator-(Kmat4f m){
        Kmat4f tmp;
        tmp.r1 = r1 - m.r1;
        tmp.r2 = r2 - m.r2;
        tmp.r3 = r3 - m.r3;
        tmp.r4 = r4 - m.r4;
        return tmp;
    }

    KOKKOS_INLINE_FUNCTION
    void operator*=(float f){
        r1 *= f;
        r2 *= f;
        r3 *= f;
        r4 *= f;
    }

    KOKKOS_INLINE_FUNCTION
    Kmat4f operator*(Kmat4f m){
        Kmat4f tmp;
        Kvec4f c1 = Kvec4f(m.r1.x, m.r2.x, m.r3.x, m.r4.x);
        Kvec4f c2 = Kvec4f(m.r1.y, m.r2.y, m.r3.y, m.r4.y);
        Kvec4f c3 = Kvec4f(m.r1.z, m.r2.z, m.r3.z, m.r4.z);
        Kvec4f c4 = Kvec4f(m.r1.w, m.r2.w, m.r3.w, m.r4.w);
        tmp.r1 = Kvec4f(r1*c1, r1*c2, r1*c3, r1*c4);
        tmp.r2 = Kvec4f(r2*c1, r2*c2, r2*c3, r2*c4);
        tmp.r3 = Kvec4f(r3*c1, r3*c2, r3*c3, r3*c4);
        tmp.r4 = Kvec4f(r4*c1, r4*c2, r4*c3, r4*c4);
        return tmp;
    }
    

    KOKKOS_INLINE_FUNCTION
    Kvec4f operator*(Kvec4f v) const{
        Kvec4f tmp;
        tmp.x = r1 * v;
        tmp.y = r2 * v;
        tmp.z = r3 * v;
        tmp.w = r4 * v;
        return tmp;
    }

    KOKKOS_INLINE_FUNCTION
    Kvec3f operator*(Kvec3f v) const
    {
        Kvec3f out;
        Kvec4f tmp(v.x,v.y,v.z,1.0);
        out.x = r1 * tmp;
        out.y = r2 * tmp;
        out.z = r3 * tmp;
        return out;
    }

    KOKKOS_INLINE_FUNCTION
    Kvec3f rotate(Kvec3f v)const
    {
        Kvec3f out;
        Kvec4f tmp(v.x, v.y, v.z, 0.0);
        out.x = r1 * tmp;
        out.y = r2 * tmp;
        out.z = r3 * tmp;
        return out;
    }

    KOKKOS_FUNCTION
    void print(const char* name = ""){
        r1.print(name);
        r2.print();
        r3.print();
        r4.print();
    }

};

//all inter vector operators go here

KOKKOS_INLINE_FUNCTION
Kvec3f operator*(float a, Kvec3i b)
{
    return Kvec3f(a * b.x, a * b.y, a * b.z);
}