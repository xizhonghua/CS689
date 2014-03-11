#ifndef MANIP_PLANNER_HPP_
#define MANIP_PLANNER_HPP_

#include "ManipSimulator.hpp"

#include <vector>
using namespace std;

class ManipPlanner
{
public:
    ManipPlanner(ManipSimulator * const manipSimulator);
            
    ~ManipPlanner(void);

/*
 * This is the function that you should implement.
 * This function needs to compute by how much the link angles should change
 * so that the robot makes a small move toward the goal while avoiding
 * obstacles, as guided by the potential field.
 *
 * allLinksDeltaTheta(j) should contain the small delta change for the angle
 * associated with the j-th link.
 *
 * Note that the attractive potential should be defined only between the end
 * effector point on the manipulator and the goal center. 
 *
 * The repulsive potential on the other hand should be defined between each
 * obstacle and each link end.
 *
 * This will ensure that, when possible, the end effector moves toward the
 * goal while every link avoids collisions with obstacles.
 *
 * You have access to the simulator.
 * You can use the methods available in simulator to get all the information
 * you need to correctly implement this function
 *
 */
    void ConfigurationMove(double allLinksDeltaTheta[]);
    
        
protected:

    // compute the Jacobian for a control_point at cfg q
    vector<vector<double> > ComputeJacobian(const Vector3& q, const Vector2& control_point);

    // get transpose of Jocabian
    vector<vector<double> > TransposeJacobian(const vector<vector<double> >& jacobian);

    // apply the JacobianTranspose to a control point
    Vector3 ApplyJacobianTranspose(const vector<vector<double> >& jacobian_t, Vector2 p);
    
    // get end points of the link
    vector<Vector2> GetLinkEndPoints(void);

    ManipSimulator  *m_manipSimulator;
};

#endif
