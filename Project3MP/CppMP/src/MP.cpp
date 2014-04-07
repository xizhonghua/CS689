#include "MP.hpp"
#include "PseudoRandom.hpp"
#include "MyTimer.hpp"
#include <cstring>
#include <iostream>
using namespace std;

const double P = 0.1;

MotionPlanner::MotionPlanner(Simulator * const simulator)
{
    m_simulator = simulator;   

    Vertex *vinit = new Vertex();

    vinit->m_parent   = -1;   
    vinit->m_nchildren= 0;    
    vinit->m_state[0] = m_simulator->GetRobotCenterX();
    vinit->m_state[1] = m_simulator->GetRobotCenterY();

    AddVertex(vinit);
    m_vidAtGoal = -1;
    m_totalSolveTime = 0;
}

MotionPlanner::~MotionPlanner(void)
{
    //do not delete m_simulator  

    const int n = m_vertices.size();
    for(int i = 0; i < n; ++i)
	delete m_vertices[i];
}

double MotionPlanner::Dist(const double* st1, const double* st2)
{
	return sqrt((st1[0]-st2[0])*(st1[0]-st2[0]) + (st1[1]-st2[1])*(st1[1]-st2[1]));
}

void MotionPlanner::ExtendTree(const int    vid, 
			       const double sto[])
{
	// get the vertex to expend from
	Vertex* v = m_vertices[vid];
	// get resolution
	double res = m_simulator->GetDistOneStep();
	// compute diff
	double diff[2] = { sto[0] - v->m_state[0], sto[1] - v->m_state[1] };
	// compute dist
	double dist = this->Dist(v->m_state, sto);
	// compute max steps
	int steps = dist / res;

	if(steps == 0) {
		cout<<"too near to expend... dist = "<<dist<<endl;
		return;
	}

	// forwarding vector
	double ss[2] = { diff[0]/steps, diff[1]/steps };
	// goal
	double gs[2] = { this->m_simulator->GetGoalCenterX(), this->m_simulator->GetGoalCenterY() };

	for(int i=1;i<=steps;i++)
	{
		// temp state
		double ts[2] = { v->m_state[0] + i*ss[0], v->m_state[1] + i*ss[1] };

		// config robot
		this->m_simulator->SetRobotCenter(ts[0], ts[1]);

		// validity check
		if(this->m_simulator->IsValidState())
		{
			Vertex *nv = new Vertex();

			nv->m_parent = (i == 1 ? vid : this->m_vertices.size()-1);
			nv->m_state[0] = ts[0];
			nv->m_state[1] = ts[1];
			nv->m_nchildren= 0;

			double dist = this->Dist(ts, gs);

			// close to goal
			if(dist < res + this->m_simulator->GetGoalRadius())
			{
				nv->m_type = nv->TYPE_GOAL;
			}

			// add to tree
			this->AddVertex(nv);
		}
		else
		{
			// hit obstacle return
			break;
		}
	}
}

void MotionPlanner::RandomConfig(double cfg[])
{

	if(PseudoRandomUniformReal() < P)
	{
		// select random state from goal region with probability P
		cfg[0] =  this->m_simulator->GetGoalCenterX();
		cfg[1] = this->m_simulator->GetGoalCenterY();
	}
	else
	{
	    // select random state uniformly with probability = (1 - P)
		this->m_simulator->SampleState(cfg);
	}
}

void MotionPlanner::ExtendRandom(void)
{
    Clock clk;
    StartTime(&clk);

	int vid = PseudoRandomUniformReal() * this->m_vertices.size();

	double sto[2];

	this->RandomConfig(sto);

    this->ExtendTree(vid, sto);
    
    m_totalSolveTime += ElapsedTime(&clk);
}

void MotionPlanner::ExtendRRT(void)
{
    Clock clk;
    StartTime(&clk);
 
    // generate a random config
    double sto[2];
    this->RandomConfig(sto);

    double min_dist = 1e10;
    Vertex* closest = NULL;
    int vid = -1;

    // find the closest vertex in the tree
    for(int i=0, size = this->m_vertices.size();i<size;++i)
    {
    	Vertex* v = this->m_vertices[i];
    	double dist = this->Dist(sto, v->m_state);
    	if(dist < min_dist)
    	{
    		min_dist = dist;
    		closest = v;
    		vid = i;
    	}
    }
    
    if(closest)
    {
    	this->ExtendTree(vid, sto);
    }

    m_totalSolveTime += ElapsedTime(&clk);
}


void MotionPlanner::ExtendEST(void)
{
    Clock clk;
    StartTime(&clk);

    // generate a random config
    double sto[2];
	this->RandomConfig(sto);

	Vertex* selected = NULL;
	int vid = -1;
	double total_weight = 0;
	int size = this->m_vertices.size();

	for(int i=0;i<size;++i)
	{
		Vertex* v = this->m_vertices[i];
		total_weight += v->weight();
	}

	//cout<<"tw = "<<total_weight<<" w = "<<w<<endl;
	for(int i=0;i<size;++i)
	{
		Vertex* v = this->m_vertices[i];
		if( v->weight() >= PseudoRandomUniformReal() * total_weight)
		{
			selected = v;
			vid = i;
			break;
		}
	}

	if(selected)
	{
		this->ExtendTree(vid, sto);
	}

    m_totalSolveTime += ElapsedTime(&clk);
}


void MotionPlanner::ExtendMyApproach(void)
{
    Clock clk;
    StartTime(&clk);
 
//your code
    
    m_totalSolveTime += ElapsedTime(&clk);
}


void MotionPlanner::AddVertex(Vertex * const v)
{
    if(v->m_type == Vertex::TYPE_GOAL)
    	m_vidAtGoal = m_vertices.size();

    m_vertices.push_back(v); 
    if(v->m_parent >= 0)
	(++m_vertices[v->m_parent]->m_nchildren);
}

void MotionPlanner::GetPathFromInitToGoal(std::vector<int> *path) const
{
    std::vector<int> rpath;
    
    rpath.clear();
    
    int i = m_vidAtGoal;
    do
    {
	rpath.push_back(i);
	i = m_vertices[i]->m_parent;	
    } 
    while(i >= 0);
    
    path->clear();
    for(int i = rpath.size() - 1; i >= 0; --i)
	path->push_back(rpath[i]);
}
