// ----------------------------------------------------
// ------------OVERLOAD CONSTRUCTOR--------------------
// testEkf::testEfk(int x):y{x}
// {

// }
// ----------------------------------------------------


#include <iostream>
#include <cmath>
#include <array>
#include <Eigen/Dense>

using namespace std;

Eigen::Matrix3d testMatrix;
Eigen::VectorXd testArray(3);

class myClass
{  
    private:
        int pri = 5;
    public:
        int pub = 15;

    // Public is passed to everyone
    public:

        // Constructor runs everytime an object is assigned
        myClass()
        {
            cout << "Constructor" << endl;
        }
        void myFunc()
        {
            cout << "Test class Func" << endl;
            cout << sqrt(++pri) << endl;
        }

    // Private is not passed to anyone
    private: 
    
        void privateFunc()
        {
            cout << "Test private func" << endl;
        }
    
    // Protected is passed only to child classes    
    protected:

        void protectedFunc()
        {
            cout << "Test protected func" << endl;
        }

};

// Test inheritance
class inheritClass : public myClass
{
    public:
        // For inherited constructor, constructor of both parent and 
        // child class are called
        inheritClass()
        {
            cout << "Inherit Const" << endl;
            cout << "---------------------------------" << endl;
            myClass::protectedFunc();
        }
};

void myFunc(int x)
{
    cout << "Test Func" << endl;
    cout << x++ << endl;
    cout << x << endl;
}

int main()
{
    cout << "---------------------------------" << endl;
    myClass myObj;
    myObj.myFunc();
    cout << "---------------------------------" << endl;
    inheritClass myObj2;
    myObj2.myFunc();
    int var;
    cout << "---------------------------------" << endl;
    array<int, 5> array1 {0,1,0,2,0};
    // int array1 [] {0,1,0,2,0};
    // for(int i = 0; i < array1.size(); i++)
    // for(int i = 0; i < sizeof(array1); i++)
    // {
    //     cout << array1[i] << endl;
    // }
    // myFunc(array);

    double x = 10;
    testMatrix << x,0,0,
    0,x,0,
    0,0,x;

    testArray << 1,2,3;

    cout << testMatrix*testArray << endl;


    cout << "---------------------------------" << endl;
    return 0;
}