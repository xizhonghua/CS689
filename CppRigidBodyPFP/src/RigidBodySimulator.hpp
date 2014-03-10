#ifndef RIGID_BODY_SIMULATOR_HPP_
#define RIGID_BODY_SIMULATOR_HPP_

#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <cassert>


struct Point
{
    double m_x;
    double m_y;
};

struct Vector2
{
	double x;
	double y;

	Vector2() : x(0), y(0) {}
	Vector2(double x, double y): x(x), y(y) {}
	Vector2(const Vector2& v) : x(v.x), y(v.y) {}

	double Normal()
	{
		return sqrt(x*x + y*y);
	}

	Vector2 Normalize()
	{
		double n = this->Normal();
		if(n == 0) return Vector2();

		return Vector2(x/n, y/n);
	}

	Vector2 GetPerpendicular()
	{
		return Vector2(-y, x);
	}

	double& operator[] (const int index) {
		if(index == 0) return x;
		if(index == 1) return y;
		assert(false);
	}
};

struct Vector3
{
	double x,y,z;

	Vector3() : x(0), y(0), z(0) {}
	Vector3(double x, double y, double z): x(x), y(y), z(z) {}
	Vector3(const Vector3& v) : x(v.x), y(v.y), z(v.z) {}

	double Normal()
	{
		return sqrt(x*x + y*y + z*z);
	}

	Vector3 Normalize()
	{
		double n = this->Normal();
		if(n == 0) return Vector3();

		return Vector3(x/n, y/n, z/n);
	}

	double& operator[] (const int index) {
		if(index == 0) return x;
		if(index == 1) return y;
		if(index == 2) return z;
		assert(false);
	}

	Vector2 toVector2() const { return Vector2(x,y); }
};



    
class RigidBodySimulator
{
public:    
    RigidBodySimulator(void);
    
    ~RigidBodySimulator(void);

    double GetGoalCenterX(void) const
    {
	return m_circles[0];	
    }

    double GetGoalCenterY(void) const
    {
	return m_circles[1];	
    }

    double GetGoalRadius(void) const
    {
	return m_circles[2];
    }
    

    int GetNrObstacles(void) const
    {
	return m_circles.size() / 3 - 1;
    }

    /**
     *@brief Returns closest point on the i-th circle obstacle to point [x, y]
     */
    Vector2 ClosestPointOnObstacle(const int i, const double x, const double y);


    double GetRobotX(void) const
    {
	return m_robot.m_x;
    }
    
    double GetRobotY(void) const
    {
	return m_robot.m_y;
    }
    
    double GetRobotTheta(void) const
    {
	return m_robot.m_theta;
    }

    int GetNrRobotVertices(void) const
    {
	return m_robot.m_currVertices.size() / 2;
    }
    
    /*
     * Vertices are stored consecutively in the vector
     * So the x-coord of the i-th vertex is at index 2 * i
     * and the y-coord of the i-th vertex is at index 2 * i + 1
     */
    const double* GetRobotVertices(void) const
    {
	return &(m_robot.m_currVertices[0]);
    }

    /**
     *@brief Returns true iff the robot center is inside the goal circle
     */
    bool HasRobotReachedGoal(void) const
    {
	const double gx = GetGoalCenterX();
	const double gy = GetGoalCenterY();
	const double rx = GetRobotX();
	const double ry = GetRobotY();
	
	return 
	    sqrt((gx - rx) * (gx - rx) + (gy - ry) * (gy - ry)) <= GetGoalRadius();
    }
    
    
protected:
    void AddToRobotConfiguration(const double dx, const double dy, const double dtheta);

    void ReadRobot(const char fname[]);

    std::vector<double> m_circles;

    struct Robot
    {
	std::vector<double> m_initVertices;
	std::vector<double> m_currVertices;
	std::vector<int>    m_triangles;
	double              m_x;
	double              m_y;
	double              m_theta;
    };
	
    Robot m_robot;

    friend class Graphics;
};

#endif
