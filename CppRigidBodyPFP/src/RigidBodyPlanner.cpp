#include <cassert>
#include <iostream>
using namespace std;
#include "RigidBodyPlanner.hpp"

const double FarAway = 2.0;
const double RepScaleFactor = 0.2;
const double RepThreshold = 0.1;
const double AttrScaleFactor = 0.002;
const double PosScale = 0.8;
const double StuckMoved = 0.3;
const double VirtualGoalFactor = 20;


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

// ====================================================

vector<Vector2> RigidBodyPlanner::GetRobotVertices(void)
{
	const double* coords = this->m_simulator->GetRobotVertices();
	int count = this->m_simulator->GetNrRobotVertices();

	vector<Vector2> vs;
	for(int i=0;i<count;i++)
		vs.push_back(Vector2(coords[i*2],coords[i*2+1]));

	return vs;
}

vector<vector<double> > RigidBodyPlanner::TransposeJacobian(const vector<vector<double> >& jacobian)
{
	vector<vector<double> > j_t;

	for(int j=0;j<jacobian[0].size();j++)
	{
		vector<double> row;
		for(int i=0;i<jacobian.size();i++)
		{
			row.push_back(jacobian[i][j]);
		}
		j_t.push_back(row);
	}

	return j_t;
}

vector<vector<double> > RigidBodyPlanner::ComputeJacobian(const Vector3& q, const Vector2& control_point)
{
	// x' = x_j * cos(theta) - y_j * sin(theta) + x
	// y' = x_j * sin(theta) + y_j * cos(theta) + y
	vector<double> J1,J2;
	J1.push_back(1);J1.push_back(0);J1.push_back(control_point.x * -sin(q.z) - control_point.y * cos(q.z));
	J2.push_back(0);J2.push_back(1);J2.push_back(control_point.x * cos(q.z) - control_point.y * sin(q.z));

	vector<vector<double> > J;
	J.push_back(J1);
	J.push_back(J2);

	return J;
}

Vector3 RigidBodyPlanner::ApplyJacobianTranspose(const vector<vector<double> >& jacobian_t, Vector2 p)
{
	Vector3 result;

	// assert jacobian_t is a 3*2 matrix
	assert(jacobian_t.size() == 3);
	assert(jacobian_t[0].size() == 2);

	for(int i=0;i<3;i++)
		for(int j=0;j<2;j++)
			result[i] += jacobian_t[i][j] * p[j];

	return result;
}

RigidBodyPlanner::RigidBodyPlanner(RigidBodySimulator * const simulator)
{
    m_simulator = simulator;
    m_first_move = true;
}


RigidBodyPlanner::~RigidBodyPlanner(void)
{
    //do not delete m_simulator  
}

void RigidBodyPlanner::GetInitalCoordinates(void)
{
	this->m_coords = this->GetRobotVertices();
	this->m_com = Vector2(0,0);
	for(size_t i=0;i<m_coords.size();++i)
		m_com = m_com + m_coords[i];
	m_com = m_com * (1.0/m_coords.size());
	for(size_t i=0;i<m_coords.size();++i)
	{
		m_coords[i] = m_coords[i] - m_com;
		cout<<"vertex i = "<<m_coords[i].x<<" "<<m_coords[i].y<<endl;
	}
}

RigidBodyMove RigidBodyPlanner::ConfigurationMove(void)
{
	if(this->m_first_move)
	{
		this->GetInitalCoordinates();
		this->m_pos = this->m_com;
		this->m_first_move = false;
	}


    // robot's current position
    Vector3 q(this->m_simulator->GetRobotX(), this->m_simulator->GetRobotY(), this->m_simulator->GetRobotTheta());
    // goal's position
    Vector2 g(this->m_simulator->GetGoalCenterX(), this->m_simulator->GetGoalCenterY());
    // get all vertices of the robot
    vector<Vector2> vs = this->GetRobotVertices();
    // get the total number of the obstacles
    int obst_count = this->m_simulator->GetNrObstacles();


   // update recent pos;
   this->m_pos = this->m_pos * PosScale + (1-PosScale) * q.toVector2();
   // get recent moved distance
   Vector2 moved = q.toVector2() - this->m_pos;

   cout<<"recent movement = "<<moved.Normal()<<" distance to goal = "<<(g - q.toVector2()).Normal()<<endl;

    // Configuration space gradient
    Vector3 csg_attr;
    Vector3 csg_rep;

    // loop all control points
    for(size_t j=0;j<vs.size();++j)
    {
    	// control point p
    	const Vector2& p = vs[j];

    	// compute Jacobian & its transpose as J' for control point j at q
    	vector<vector<double> > jacobian_t = this->TransposeJacobian(this->ComputeJacobian(q, this->m_coords[j]));

    	// compute the gradient of the attraction
    	// U_Att = 1/2 * factor * |p - g| ^ 2
    	// U_Att' = factor*(p - g)
    	Vector2 u_att = (p - g) * AttrScaleFactor;

    	// add attract to the configuration space gradient
    	csg_attr = csg_attr + this->ApplyJacobianTranspose(jacobian_t, u_att);

    	// loop each obstacle
    	for(int i=0;i<obst_count;++i)
    	{
    		// get the closest point form p to obst_i
    		Vector2 cp = this->m_simulator->ClosestPointOnObstacle(i, p.x, p.y);

    		double dis = (p - cp).Normal();

    		// obstacle is far away from current control point, just ignore it
    		if ( dis > FarAway ) continue;

    		// compute the workspace gradient
    		Vector2 u_rep = RepScaleFactor* ((cp - p)  / (dis * dis));

    		//u_rep = u_rep * (drand48() * 10 + 0.5);

    		csg_rep = csg_rep + this->ApplyJacobianTranspose(jacobian_t, u_rep);
    	}
    }

    // combine csg_attr and csg_rep
    Vector3 csg = csg_attr + csg_rep;




	// get stuck due to local minimal
	if(moved.Normal() < StuckMoved && csg_rep.toVector2().Normal() > RepThreshold) {

		for(size_t j=0;j<vs.size();++j)
		{
			// control point p
			const Vector2& p = vs[j];

			// add a virtual goal that is VirtualGoalFactor times away from robot in the opposite direction to the goal
			Vector2 u_att2 = (g - p).GetPerpendicular()*VirtualGoalFactor*AttrScaleFactor;
			vector<vector<double> > jacobian_t = this->TransposeJacobian(this->ComputeJacobian(q, p));
			// get the attract csg with some random factor
			Vector3 csg_attr2 = this->ApplyJacobianTranspose(jacobian_t, u_att2) * drand48();
			// add to the csg
			csg = csg + csg_attr2;
		}
	}

    RigidBodyMove move;
    move.m_dx = -csg.x;
    move.m_dy = -csg.y;
    move.m_dtheta = -csg.z;

    return move;
}
