#include "ManipPlanner.hpp"

#include <iostream>
using namespace std;

// ======================================================================
Vector2 operator*(const Vector2& lhs, double scale)
{
	return Vector2(lhs.x * scale, lhs.y * scale);
}

Vector2 operator*(double scale, const Vector2& rhs)
{
	return rhs*scale;
}

double operator*(const Vector2& lhs, const Vector2& rhs)
{
	return lhs.x*rhs.x + lhs.y*rhs.y;
}

Vector2 operator-(const Vector2& lhs, const Vector2& rhs)
{
	return Vector2(lhs.x - rhs.x, lhs.y - rhs.y);
}

Vector2 operator+(const Vector2& lhs, const Vector2& rhs)
{
	return Vector2(lhs.x + rhs.x, lhs.y + rhs.y);
}

Vector3 operator*(const Vector3& lhs, double scale)
{
	return Vector3(lhs.x * scale, lhs.y * scale, lhs.z * scale);
}

Vector3 operator*(double scale, const Vector3& rhs)
{
	return rhs*scale;
}

double operator*(const Vector3& lhs, const Vector3& rhs)
{
	return lhs.x*rhs.x + lhs.y*rhs.y + lhs.z*rhs.z;
}

Vector3 operator-(const Vector3& lhs, const Vector3& rhs)
{
	return Vector3(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
}

Vector3 operator+(const Vector3& lhs, const Vector3& rhs)
{
	return Vector3(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
}
// ======================================================================

ManipPlanner::ManipPlanner(ManipSimulator * const manipSimulator)
{
    m_manipSimulator = manipSimulator;   
}

ManipPlanner::~ManipPlanner(void)
{
    //do not delete m_simulator  
}

vector<Vector2> ManipPlanner::GetLinkEndPoints(void)
{
	int nrLinks = this->m_manipSimulator->GetNrLinks();

	vector<Vector2> end_points;

	for(int i=0;i<nrLinks;i++)
	{
	   Vector2 p = Vector2(m_manipSimulator->GetLinkEndX(i), m_manipSimulator->GetLinkEndY(i));
	   end_points.push_back(p);
	}

	return end_points;
}

void ManipPlanner::ConfigurationMove(double allLinksDeltaTheta[])
{
   vector<Vector2> end_points = this->GetLinkEndPoints();

   const Vector2 end_point = end_points.back();
}

