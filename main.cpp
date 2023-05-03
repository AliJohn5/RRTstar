#include <iostream>
#include<vector>
#include<algorithm>
#include<time.h>
#include<cstdlib>
#include<vector>
#include<math.h>
#include<set>
#include <random>

#define endl '\n'
#define reboot_obsract true
#define dont_reboot_obsract false


using std::vector;
using std::set;
using std::cout;

std::random_device rd;
std::mt19937 gen(rd());

struct Point3D
{
	int x, y, z;
	Point3D()
	{

	}
	Point3D(int a, int b, int c)
	{
		x = a;
		y = b;
		z = c;
	}
	double cost;
	Point3D* parent;
	vector<Point3D*>childrens;
};

struct Point3DForAmin
{
    int x,y,z;
    double vx,vy,vz;
    Point3DForAmin(int a,int b,int c)
    {
        x=a;
        y=b;
        z=c;
    }
};


bool operator == (Point3D a, Point3D b)
{
	if (a.x == b.x && a.y == b.y && a.z == b.z) return true;
	return false;
}

bool operator != (Point3D a, Point3D b)
{
	if (a.x != b.x) return true;
	if (a.y != b.y) return true;
	if (a.z != b.z) return true;
	return false;
}

bool operator > (Point3D a, Point3D b)
{
	if (a.x > b.x) return true;
	else if (a.x == b.x)
	{
		if (a.y > b.y) return true;
		else if (a.y == b.y)
		{
			if (a.z > b.z) return true;
		}
	}
	return false;

}


bool operator >= (Point3D a, Point3D b)
{
	if (a == b) return true;
	else
	{
		if (a.x > b.x) return true;
		if (a.y > b.y) return true;
		if (a.z > b.z) return true;
	}
	return false;
}


bool operator <= (Point3D a, Point3D b)
{
	if (a == b) return true;
	else
	{
		if (a.x < b.x) return true;
		if (a.y < b.y) return true;
		if (a.z < b.z) return true;
	}
	return false;
}


bool operator < (Point3D a, Point3D b)
{
	if (a.x < b.x) return true;
	else if (a.x == b.x)
	{
		if (a.y < b.y) return true;
		else if (a.y == b.y)
		{
			if (a.z < b.z) return true;
		}
	}
	return false;
}

bool operator !(Point3D& a)
{
	if (a.x == 0 && a.y == 0 && a.z == 0) return true;
	return false;
}



class RRTstar3D
{
public:
	RRTstar3D(int _max_x, int _max_y, int _max_z,
		      int _min_x, int _min_y, int _min_z,
		      int _max_step, int _min_step);
    int max_x, max_y, max_z, min_x, min_y, min_z;
	int max_step, min_step;
	int safetyDist = 1;
	int timeForAll;
    double fullDist;
	Point3D* not_found_point1 = new Point3D(100000,100000,100000);
	Point3D* head;
	Point3D start, goal;
	set<Point3D>visited;
	set<Point3D>obstract;
	vector<Point3D>path;
	vector<Point3DForAmin*>pathForAmin;
	void set_start_and_goal(Point3D _start, Point3D _goal, int time_for_all_points);
	double Dist(Point3D a, Point3D p);
	Point3D* random_point();
	Point3D check_point(Point3D New, Point3D* old);
	void print_the_path();
	void get_safety_dist(int SD);
	void get_obstract_point(int _x, int _y, int _z);
	void reboot();
	void rebootAll (bool f);
	double is_goal(Point3D old, Point3D b);
	void go();
	void BFS1(Point3D* head, Point3D* fin, Point3D* &temp, double &dist);
    void print_path_for_amin();
};



RRTstar3D::RRTstar3D(int _max_x, int _max_y, int _max_z, int _min_x, int _min_y, int _min_z, int _max_step, int _min_step)
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
	srand(time(0));
}
void RRTstar3D::rebootAll(bool f)
{
	visited.erase(visited.begin(), visited.end());
	path.erase(path.begin(), path.end());
	if (f) obstract.erase(obstract.begin(), obstract.end());
}

void RRTstar3D::reboot()
{
	path.erase(path.begin(), path.end());
}


void RRTstar3D::get_safety_dist(int SD)
{
	safetyDist = SD;
}


void RRTstar3D::get_obstract_point(int _x, int _y, int _z)
{
	Point3D ob( _x, _y ,_z);
	obstract.insert(ob);

	for (int i = _x - safetyDist; i < _x + safetyDist; i++)
	{
		for (int j = _y - safetyDist; j < _y + safetyDist; j++)
		{
			for (int k = _z - safetyDist; k < _z + safetyDist; k++)
			{
				if (((i - _x) * (i - _x) + (j - _y) * (j - _z) + (k - _z) * (k - _z)) < safetyDist * safetyDist)
				{
					Point3D p(i, j, k);
					obstract.insert(p);
				}
			}
		}
	}

}


void RRTstar3D::print_the_path()
{
	cout << "path from ( " << start.x << ", " << start.y << " ," << start.z << " ) to ( ";
	cout << goal.x << ", " << goal.y << ", " << goal.z << " ) with "<<path.size()<<" point is:"<<endl;
	for (int i = 0; i < path.size(); i++)
	{
		cout << "( " << path[i].x << " , " << path[i].y << " , " << path[i].z << " )" << "  cost: "<< path[i].cost<< endl;
	}
}

void RRTstar3D::set_start_and_goal(Point3D _start, Point3D _goal, int time_for_all_points)
{
	srand(time(0));
	start = _start;
	goal = _goal;
	visited.insert(start);
	Point3D* n = new Point3D(start.x, start.y, start.z);
	n->parent = nullptr;
	n->cost = 0;
	head = n;
    timeForAll = time_for_all_points;
}


double RRTstar3D::Dist(Point3D old, Point3D b)
{
	int dx = old.x - b.x;
	int dy = old.y - b.y;
	int dz = old.z - b.z;
	return (sqrt(dx * dx + dy * dy + dz * dz));
}



double RRTstar3D::is_goal(Point3D old, Point3D b)
{
	double dx = old.x - b.x;
	double dy = old.y - b.y;
	double dz = old.z - b.z;
	return (sqrt(dx * dx + dy * dy + dz * dz));
}

Point3D RRTstar3D::check_point(Point3D New, Point3D* old)
{
	double dd = is_goal(*old, New);
	if (dd <= max_step && dd >= min_step) return New;

	dd = dd / max_step;

	int _x = abs(New.x - old->x) - abs((New.x - old->x) / dd);
	int _y = abs(New.y - old->y) - abs((New.y - old->y) / dd);
	int _z = abs(New.z - old->z) - abs((New.z - old->z) / dd);

	if (New.x > old->x) New.x -= _x;
	else New.x += _x;

	if (New.y > old->y) New.y -= _y;
	else New.y += _y;

	if (New.z > old->z) New.z -= _z;
	else New.z += _z;

	return New;
}


Point3D* RRTstar3D::random_point()
{
	/*0*int X = rand() % (max_x - min_x) + (min_x);
	int Y = rand() % (max_y - min_y) + (min_y);
	int Z = rand() % (max_z - min_z) + (min_z);*/
 // Standard mersenne_twister_engine seeded with rd()

	std::uniform_real_distribution<> dis_X(0, max_x);
	std::uniform_real_distribution<> dis_Y(0, max_y);
	std::uniform_real_distribution<> dis_Z(0, max_z);
	Point3D *f =new Point3D(dis_X(gen), dis_Y(gen), dis_Z(gen));
	return f;
}

void RRTstar3D::print_path_for_amin()
{
    cout << "path from ( " << start.x << ", " << start.y << " ," << start.z << " ) to ( ";
	cout << goal.x << ", " << goal.y << ", " << goal.z << " ) with "<<path.size()<<" point is:"<<endl;
	for (int i = 0; i < pathForAmin.size(); i++)
	{
		cout << "( " << pathForAmin[i]->x << " , " << pathForAmin[i]->y << " , " << pathForAmin[i]->z << " )" << endl;
		cout << "( " << pathForAmin[i]->vx << " , " << pathForAmin[i]->vy << " , " << pathForAmin[i]->vz << " )" << endl;
        cout << "______________" <<endl;
	}
}


void RRTstar3D::go()
{
	Point3D* final_near = new Point3D();
	while (true)
	{
		Point3D* f = random_point();
		/*   ***   **/
		double dist = 1000;
		Point3D* temp = new Point3D();
		temp = not_found_point1;
		BFS1(head, f, temp, dist);
		/*****************/
		Point3D f1 = check_point(*f, temp);
		if (obstract.find(f1)!=obstract.end() && visited.find(f1)!=visited.end()) continue;
		/****************/
		/******************/
		visited.insert(f1);
		Point3D* n = new Point3D(f1.x,f1.y,f1.z);
		n->parent = temp ;
		n->cost = temp->cost + is_goal(*temp, *n);
		temp->childrens.push_back(n);
		final_near = n;
		if (is_goal(f1, goal) <= max_step && is_goal(f1, goal) >= min_step) { break; }
	}

	Point3D* n = new Point3D(goal.x,goal.y,goal.z);
	n->parent = final_near;
	n->cost = n->parent->cost + is_goal(*n, *n->parent);
	final_near->childrens.push_back(n);

	while (*n != start)
	{
		path.push_back(*n);
		n = n->parent;
	}
	path.push_back(*n);
	//reverse(path.begin(), path.end());

    int number_of_point = path.size();

    for(int i=1;i<number_of_point;i++)
    {
        double dist = Dist(path[i],path[i-1]);
        double theta = atan( (double) (path[i].y-path[i-1].y) / (path[i].x-path[i-1].x) );
        double t1 = (double) (timeForAll * dist) / path[0].cost;
        double v = (double) dist / t1;
        Point3DForAmin* m = new Point3DForAmin( path[i].x, path[i].y, path[i].z );
        m->vx = v*cos(theta);
        m->vy = v*sin(theta);
        m->vz = tan( (double) m->vy/m->vx);
        pathForAmin.push_back(m);
    }
    Point3DForAmin* m = new Point3DForAmin( path[0].x,path[0].y,path[0].z  );
    m->vx = 0;
    m->vy = 0;
    m->vz = 0;
    reverse(pathForAmin.begin(),pathForAmin.end());
    pathForAmin.push_back(m);
}

/// function for find nearest Point using BFS

void RRTstar3D::BFS1(Point3D* head, Point3D* fin, Point3D* &temp, double &dist)
{
	double dd = Dist(*head, *fin);
	if ( dist > dd && dd > min_step && *head!=*fin)
	{
		dist = dd;
		temp = head;
	}
	for (auto a : head->childrens) BFS1(a, fin, temp,dist);
}

int main()
{
	RRTstar3D ali2(640, 480, 480, 1, 1, 1, 25, 2);
	ali2.get_safety_dist(2);
	ali2.get_obstract_point(20, 20,20);
	Point3D sour(50, 50, 50);
	Point3D fi(300, 300, 300);
    int time_for_all = 50;
	ali2.set_start_and_goal(sour, fi,time_for_all);
	ali2.go();
	ali2.print_path_for_amin();
	ali2.rebootAll(reboot_obsract);

	return 0;
}