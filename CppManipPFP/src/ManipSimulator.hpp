#ifndef MANIP_SIMULATOR_HPP_
#define MANIP_SIMULATOR_HPP_

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
    

class ManipSimulator
{
public:    
    ManipSimulator(void);
    
    ~ManipSimulator(void);
    
    double GetGoalCenterX(void) const
    {
	return m_circles[0];	
    }

    double GetGoalCenterY(void) const
    {
	return m_circles[1];	
    }

    int GetNrObstacles(void) const
    {
	return m_circles.size() / 3 - 1;
    }

    /**
     *@brief Returns closest point on the i-th circle obstacle to point [x, y]
     */
    Vector2 ClosestPointOnObstacle(const int i, const double x, const double y);

    int GetNrLinks(void) const
    {
	return m_joints.size();
    }

    double GetLinkStartX(const int i) const
    {
	return m_positions[2 * i];
    }

    double GetLinkStartY(const int i) const
    {
	return m_positions[2 * i + 1];
    }

    double GetLinkEndX(const int i) const
    {
	return GetLinkStartX(i + 1);
    }

    double GetLinkEndY(const int i) const
    {
	return GetLinkStartY(i + 1);
    }

    bool HasRobotReachedGoal(void) const;
    
    // reset links to initial status;
    void RestLinks();
protected:

    double GetObstacleCenterX(const int i) const
    {
	return m_circles[3 * i + 3];
    }
    
    double GetObstacleCenterY(const int i) const
    {
	return m_circles[3 * i + 4];
    }
    
    double GetObstacleRadius(const int i) const
    {
	return m_circles[3 * i + 5];
    }

    double GetLinkTheta(const int i) const
    {
	return m_joints[i];
    }

    double GetLinkLength(const int i) const
    {
	return m_lengths[i];
    }

    double GetGoalRadius(void) const
    {
	return m_circles[2];
    }
    
    void FK(void);


protected:

    void AddLink(const double length);
    
    void AddToLinkTheta(const int i, const double dtheta)
    {
	m_joints[i] += dtheta;
    }

    std::vector<double> m_joints;
    std::vector<double> m_lengths;
    std::vector<double> m_positions;
    std::vector<double> m_circles;
    
    friend class Graphics;
};

#endif
