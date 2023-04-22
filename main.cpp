#include <iostream>
#include "RRTstar.h"


int main()
{
    /**

    RRTstar3D ali;
    // get_safety_dist(x) : x = the distance around the obstracts
    ali.get_safety_dist(5);
    // get_obstract_point(x,y,z)
    ali.get_obstract_point(50,50,50);
    // void set_the_cons ( int _max_x , int _max_y , int _max_z , int _min_x , int _min_y , int _min_z, int _max_step , int _min_step)
    ali.set_the_cons(300,300,300,1,1,1,10,1);
    // point3D source = { x , { y , z } }
    point3D source = { 100 , {100,100} };
    point3D fina = { 200, {200, 200} };
    ali.set_start_and_goal(source,fina);

    // go : start the algorithm
    ali.go();
    // print_the_path() : ( x , y , z )
    ali.print_the_path();
    // when you call reboot you can reuse the algorithm
    // false : the obstracts didn't change
    // true : the obstracts changed
    ali.reboot(false);

    // you can reuse the algorithm like that
    /*
    point3D source = { 100 , {100,100} };
    point3D fina = { 200, {200, 200} };
    ali.set_start_and_goal(source,fina);

    // go : start the algorithm
    ali.go();
    // print_the_path() : ( x , y , z )
    ali.print_the_path();
    */

    /////////////////////////////////////////////

    cout << endl <<endl;

    /////////////////////////////////////////////

    RRTstar2D ali2;

    ali2.get_safety_dist(5);
    ali2.get_obstract_point(50,50);
    ali2.get_obstract_point(150,150);

    ali2.set_the_cons(700,700,1,1,10,1);

    point2D sour = { 100 , 100 };
    point2D fi = { 500, 500 };
    ali2.set_start_and_goal(sour,fi);

    ali2.go();
    ali2.print_the_path();
    ali2.reboot(false);
    return 0;
}
