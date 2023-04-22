#ifndef RRTSTAR_H_INCLUDED
#define RRTSTAR_H_INCLUDED


#include <iostream>
#include<vector>
#include<algorithm>
#include<time.h>
#include<cstdlib>
#include<vector>
#include<math.h>
#include<set>

using std::vector;
using std::set;
using std::cout;
using std::cin;
using std::endl;
using std::pair;

#define point2D pair< int,int >
#define point3D pair< int, pair< int,int > >

#define group3D set< pair< int, pair< int,int > > >
#define group2D set< pair< int, int > >

#define Tree vector< Node * >

#define x3 first
#define y3 second.first
#define z3 second.second

#define x2 first
#define y2 second

class RRTstar3D
{
private:
    class Node
    {
    public:
        point3D p;
        Node * parent;
    };

    int max_x, max_y, max_z, min_x, min_y, min_z;
    int max_step, min_step;

    point3D start, goal;
    group3D obs, visited;
    int safety_dist;
    Node * path;
    Tree tree;
public:
    Tree Path;
    void set_the_cons ( int _max_x , int _max_y , int _max_z , int _min_x , int _min_y , int _min_z, int _max_step , int _min_step);
    void set_start_and_goal ( point3D _start , point3D _goal );
    double Dist (point3D a, point3D p);
    int find_near(point3D a);
    point3D random_point();
    point3D check_point(point3D a, point3D b);
    void go();
    void print_the_path();
    void get_safety_dist(int SD);
    void get_obstract_point(int _x , int _y , int _z);
    void reboot(bool f);
};


void RRTstar3D::reboot(bool f)
{
    visited.erase( visited.begin(),visited.end());
    Path.erase(Path.begin(),Path.end());
    tree.erase(tree.begin(),tree.end());
    if (f) obs.erase(obs.begin(),obs.end());
}

point3D RRTstar3D::check_point(point3D New, point3D old)
{
    double dd = Dist( New , old );
    if (dd <= max_step && dd>=min_step ) return New;

    dd = dd / max_step;

    int _x = abs(New.x3 - old.x3) - abs((New.x3 - old.x3)/dd ) ;
    int _y = abs(New.y3 - old.y3) - abs((New.y3 - old.y3)/dd  );
    int _z = abs(New.z3 - old.y3) - abs((New.z3 - old.y3)/dd  );

    if ( New.x3 > old.x3 ) New.x3 -= _x ;
    else New.x3+= _x ;

    if ( New.y3 > old.y3 ) New.y3 -= _y ;
    else New.y3 += _y ;

    if ( New.z3 > old.z3 ) New.z3 -= _z ;
    else New.z3 += _z ;

    return New;
}

void RRTstar3D::set_the_cons ( int _max_x , int _max_y , int _max_z , int _min_x , int _min_y , int _min_z, int _max_step , int _min_step )
{
    max_x = _max_x;
    max_y = _max_y;
    max_z = _max_z;
    min_x = _min_x;
    min_y = _min_y;
    min_z = _min_z;
    if (min_x < 1) min_x = 1;
    if (min_y < 1) min_y = 1;
    if (min_z < 1) min_z = 1;
    max_step = _max_step;
    min_step = _min_step;
}

void RRTstar3D::set_start_and_goal ( point3D _start , point3D _goal )
{
    srand(time(0));
    start = _start;
    goal = _goal ;
    visited.insert(start);
    Node * n = new Node();
    n->p = start;
    n->parent = nullptr;
    tree.push_back(n);
}

double RRTstar3D::Dist (point3D a, point3D b)
{
    double dx = a.x3 - b.x3;
    double dy = a.y3 - b.y3;
    double dz = a.z3 - b.z3;
    return ( sqrt ( dx*dx + dy*dy + dz*dz  )  );
}


int RRTstar3D::find_near(point3D a)
{
    double dd = Dist(tree[0]->p , a);
    int i = 0;
    for (int k = 1; k < tree.size() ;k++)
    {
        double nn = Dist(a,tree[k]->p);
        if (  nn  <  dd  &&  nn >= min_step  ) { i = k; dd = nn; }
    }
    return i;
}

point3D RRTstar3D::random_point()
{
    int X = rand()%(max_x-min_x ) + (min_x) ;
    int Y = rand()%(max_y - min_y ) + (min_y) ;
    int Z = rand()%(max_z - min_z) + (min_z) ;
    point3D f = { X , { Y , Z } };
    return f;
}

void RRTstar3D::go()
{
//    bool finish = false;
//    srand(time());
    while (true)
    {
        point3D f = random_point();
        int ind = find_near(f);
        f = check_point(f , tree[ind]->p);
        if (binary_search(obs.begin() , obs.end() , f ) || binary_search(visited.begin() , visited.end() , f )) continue;
        visited.insert(f);
        Node * n = new Node();
        n->p = f;
        n->parent = tree[ind];
        tree.push_back(n);
        if ( Dist(f,goal)<=max_step &&  Dist(f,goal)>=min_step ) { break;}
    }

//    cout << 1 <<endl;
//    rr+=num_of_iteration;
//    if (!finish) go();
    Node * n = new Node();
    n->p = goal;
    n->parent = tree[ tree.size()-1];
    tree.push_back(n);

    n = tree[ tree.size() -1  ];
    while  (n != nullptr)
    {
        Path.push_back(n);
        n = n->parent;
    }
    reverse(Path.begin(),Path.end());
}

void RRTstar3D::print_the_path()
{
    for (int i=0; i<Path.size(); i++)
    {
        cout <<"( " << Path[i]->p.x3 << " , " << Path[i]->p.y3 << " , " << Path[i]->p.z3 << " )" <<endl;
    }
}

void RRTstar3D::get_safety_dist(int SD)
{
    safety_dist = SD;
}

void RRTstar3D::get_obstract_point(int _x , int _y , int _z)
{
    point3D ob = { _x, {_y ,_z}  };
    obs.insert(ob);

    for(int i = _x-safety_dist ; i<_x+safety_dist  ;i++)
    {
        for(int j = _y-safety_dist ; j<_y+safety_dist  ;j++)
        {
            for(int k = _z-safety_dist ; k<_z+safety_dist  ;k++)
            {
                if ( ((i -_x)*(i -_x) +  (j -_y)*(j -_z) + (k -_z)*(k -_z) )< safety_dist*safety_dist  )
                obs.insert ( { i ,{ j,k } }  );
            }
        }
    }

}






////////////////////////////////////
///////////////////////////////////
///////////////////////////////////


class RRTstar2D
{
private:
    class Node
    {
    public:
        point2D p;
        Node * parent;
    };

    int max_x, max_y, min_x, min_y;
    int max_step, min_step;

    point2D start, goal;
    group2D obs, visited;
    int safety_dist;
    Node * path;
    Tree tree;

public:
    Tree Path;
    void set_the_cons ( int _max_x , int _max_y , int _min_x , int _min_y , int _max_step , int _min_step );
    void set_start_and_goal ( point2D _start , point2D _goal );
    double Dist (point2D a, point2D p);
    int find_near(point2D a);
    point2D random_point();
    point2D check_point(point2D a, point2D b);
    void go();
    void print_the_path();
    void get_safety_dist(int SD);
    void get_obstract_point(int _x , int _y );
    void reboot(bool f);
};

void RRTstar2D::reboot(bool f)
{
    visited.erase( visited.begin(),visited.end());
    Path.erase(Path.begin(),Path.end());
    tree.erase(tree.begin(),tree.end());
    if (f) obs.erase(obs.begin(),obs.end());
}



point2D RRTstar2D::check_point(point2D New, point2D old)
{
    double dd = Dist( New , old );
    if (dd <= max_step && dd>=min_step ) return New;

    dd = dd / max_step;

    int _x = abs(New.x2 - old.x2) - abs((New.x2 - old.x2)/dd ) ;
    int _y = abs(New.y2- old.y2) - abs((New.y2 - old.y2)/dd  );

    if ( New.x2 > old.x2 ) New.x2-= _x ;
    else New.x2 += _x ;

    if ( New.y2 > old.y2 ) New.y2 -= _y ;
    else New.y2 += _y ;

    return New;
}

void RRTstar2D::set_the_cons ( int _max_x , int _max_y  , int _min_x , int _min_y ,  int _max_step , int _min_step )
{
    max_x = _max_x;
    max_y = _max_y;

    min_x = _min_x;
    min_y = _min_y;

    if (min_x < 1) min_x = 1;
    if (min_y < 1) min_y = 1;

    max_step = _max_step;
    min_step = _min_step;
}

void RRTstar2D::set_start_and_goal ( point2D _start , point2D _goal )
{
    srand(time(0));
    start = _start;
    goal = _goal ;
    visited.insert(start);
    Node * n = new Node();
    n->p = start;
    n->parent = nullptr;
    tree.push_back(n);
}

double RRTstar2D::Dist (point2D a, point2D b)
{
    double dx = a.x2 - b.x2;
    double dy = a.y2 - b.y2;

    return ( sqrt ( dx*dx + dy*dy   )  );
}


int RRTstar2D::find_near(point2D a)
{
    double dd = Dist(tree[0]->p , a);
    int i = 0;
    for (int k = 1; k < tree.size() ;k++)
    {
        double nn = Dist(a,tree[k]->p);
        if (  nn  <  dd  &&  nn >= min_step  ) { i = k; dd = nn; }
    }
    return i;
}

point2D RRTstar2D::random_point()
{
    int X = rand()%(max_x-min_x ) + (min_x) ;
    int Y = rand()%(max_y - min_y ) + (min_y) ;

    point2D f = { X , Y   };
    return f;
}

void RRTstar2D::go()
{
//    bool finish = false;
//    srand(time());
    while (true)
    {
        point2D f = random_point();
        int ind = find_near(f);
        f = check_point(f , tree[ind]->p);
        if (binary_search(obs.begin() , obs.end() , f ) || binary_search(visited.begin() , visited.end() , f )) continue;
        visited.insert(f);
        Node * n = new Node();
        n->p = f;
        n->parent = tree[ind];
        tree.push_back(n);
        if ( Dist(f,goal)<=max_step &&  Dist(f,goal)>=min_step ) { break;}
    }

//    cout << 1 <<endl;
//    rr+=num_of_iteration;
//    if (!finish) go();
    Node * n = new Node();
    n->p = goal;
    n->parent = tree[ tree.size()-1];
    tree.push_back(n);

    n = tree[ tree.size() -1  ];
    while  (n != nullptr)
    {
        Path.push_back(n);
        n = n->parent;
    }
    reverse(Path.begin(),Path.end());
}

void RRTstar2D::print_the_path()
{
    
    for (int i=0; i<Path.size(); i++)
    {
        cout <<"( " << Path[i]->p.x2 << " , " << Path[i]->p.y2 << " )" <<endl;
    }
    
/*
    cout <<"[ ";
    for (auto & a: obs)
    {
//        cout << Path[i]->p.x2 << " , " ;
            cout << a.x2 << " , " ;
    }

    cout <<"] ";
    cout <<endl;
    cout <<"[ ";
    for (auto & a: obs)
    {
//        cout<< Path[i]->p.y2 << " , ";
cout <<a.y2 << " , " ;
    }
    cout <<"] ";*/
}

void RRTstar2D::get_safety_dist(int SD)
{
    safety_dist = SD;
}

void RRTstar2D::get_obstract_point(int _x , int _y )
{
    point2D ob = { _x, _y   };
    obs.insert(ob);

    for(int i = _x-safety_dist ; i<_x+safety_dist  ;i++)
    {
        for(int j = _y-safety_dist ; j<_y+safety_dist  ;j++)
        {
                if ( ((i -_x)*(i -_x) +  (j -_y)*(j -_y)  )< safety_dist*safety_dist  )
                obs.insert ( { i , j }  );
        }
    }

}

#endif // RRTSTAR_H_INCLUDED
