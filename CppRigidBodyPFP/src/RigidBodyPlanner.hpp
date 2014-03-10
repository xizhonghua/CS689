#ifndef RIGID_BODY_PLANNER_HPP_
#define RIGID_BODY_PLANNER_HPP_

#include <vector>
using namespace std;

#include "RigidBodySimulator.hpp"



struct RigidBodyMove
{
    double m_dx;
    double m_dy;
    double m_dtheta;
};

class RigidBodyPlanner
{
public:
    RigidBodyPlanner(RigidBodySimulator * const simulator);
            
    ~RigidBodyPlanner(void);

    /*
     * This is the function that you should implement.
     * This function needs to compute by how much the position (dx, dy) 
     * and orientation (dtheta) should change so that the robot makes a small 
     * move toward the goal while avoiding obstacles, 
     * as guided by the potential field.
     *
     * You have access to the simulator.
     * You can use the methods available in simulator to get all the information
     * you need to correctly implement this function
     *
     */
    RigidBodyMove ConfigurationMove(void);
    
protected:

    vector<Vector2> GetRobotVertices(void);

	vector<vector<double> > ComputeJacobian(const Vector3& q, const Vector2& p);
	vector<vector<double> > TransposeJacobian(const vector<vector<double> >& jacobian);

	// return v (3*1) = j_t (3*2) * p (2*1)
	Vector3 ApplyJacobianTranspose(const vector<vector<double> >& jacobian_t, Vector2 p);

	void GetInitalCoordinates(void);


    RigidBodySimulator *m_simulator;


    bool m_first_move;
    // coordinates of the control points in the initial configuration
    vector<Vector2> m_coords;
    // center of mass
    Vector2 m_com;

    // pos
    Vector2 m_pos;
};


#endif
