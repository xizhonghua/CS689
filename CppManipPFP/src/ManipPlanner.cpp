#include "ManipPlanner.hpp"

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

void ManipPlanner::ConfigurationMove(double allLinksDeltaTheta[])
{
   
}

