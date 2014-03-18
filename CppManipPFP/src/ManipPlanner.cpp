#include "ManipPlanner.hpp"

#include <iostream>
using namespace std;

const double FarAway = 0.8;
const double RepScaleFactor = 0.005;
const double AttrScaleFactor = 0.0001;

// ======================================================================
Vector2 operator*(const Vector2& lhs, double scale)
{
	return Vector2(lhs.x * scale, lhs.y * scale);
}

Vector2 operator*(double scale, const Vector2& rhs)
{
	return rhs*scale;
}

Vector2 operator/(const Vector2& lhs, double scale)
{
	if(scale == 0) return Vector2();
	return 1.0/scale*lhs;
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

vector<double> ManipPlanner::Add(const vector<double>& lhs, const vector<double>& rhs)
{
	assert(lhs.size() == rhs.size());
	vector<double> out = lhs;
	for(size_t i=0;i<rhs.size();++i)
		out[i]+=rhs[i];
	return out;
}

vector<vector<double> > ManipPlanner::ComputeJacobianT(const int j)
{
	const int n = this->m_manipSimulator->GetNrLinks();
	vector<vector<double> > J;
	J.resize(n);

	for(int i=0;i<=j;i++)
	{
		double x = - this->m_manipSimulator->GetLinkEndY(j) + this->m_manipSimulator->GetLinkStartY(i);
		double y = this->m_manipSimulator->GetLinkEndX(j) - this->m_manipSimulator->GetLinkStartX(i);

		J[i].push_back(x);
		J[i].push_back(y);
	}

	for(int i=j+1;i<n;i++)
	{
		J[i].push_back(0);
		J[i].push_back(0);
	}

	return J;
}

vector<double> ManipPlanner::ApplyJacobianTranspose(const vector<vector<double> >& jacobian_t, const Vector2& U)
{
	vector<double> out;

	for(size_t i=0;i<jacobian_t.size();++i)
	{
		out.push_back(jacobian_t[i][0] * U.x + jacobian_t[i][1] * U.y);
	}

	return out;
}

void ManipPlanner::ConfigurationMove(double allLinksDeltaTheta[])
{
	const int nrLinks = this->m_manipSimulator->GetNrLinks();
	const int nrObsts = this->m_manipSimulator->GetNrObstacles();

	vector<Vector2> end_points = this->GetLinkEndPoints();

	// last point on the link
	const Vector2 last_point = end_points.back();

	const Vector2 goal = Vector2(this->m_manipSimulator->GetGoalCenterX(), this->m_manipSimulator->GetGoalCenterY());

	vector<double> csg_att(nrLinks);
	vector<double> csg_rep(nrLinks);

	// loop each control point
	for(size_t j=0;j<nrLinks;j++)
	{
		// control point p
		const Vector2& p = end_points[j];

		// compute transpose of the Jacobian
		vector<vector<double> > J_T = this->ComputeJacobianT(j);

		// loop each obstacles
		for(int i=0;i<nrObsts;i++)
		{
			// get the closest point from p to obst_o
			Vector2 closest_point = this->m_manipSimulator->ClosestPointOnObstacle(i, p.x, p.y);

			// distance from control point to closest point on obst_o
			double dis = (closest_point - p).Normal();

			// obstacle is far away from current control point, just ignore it
			if(dis > FarAway) continue;

			// W-space repulsive gradient
			Vector2 u_rep_j_i = RepScaleFactor * ((closest_point - p) / (dis*dis));

			// compute C-space repulsive gradient;
			vector<double> csg_rep_j_i = this->ApplyJacobianTranspose(J_T, u_rep_j_i);

			// add it up
			csg_rep = this->Add(csg_rep, csg_rep_j_i);
		}
	}

	// compute transpose of the Jacobian for the last point on the link
	vector<vector<double> > J_T = this->ComputeJacobianT(nrLinks-1);

	// compute the C-space attractive gradient
	csg_att = this->ApplyJacobianTranspose(J_T,  AttrScaleFactor * (last_point - goal));

	// add attr & rep
	vector<double> sum = this->Add(csg_att, csg_rep);

	// output
	for(size_t i=0;i<sum.size();++i)
		allLinksDeltaTheta[i] = -sum[i];
}

