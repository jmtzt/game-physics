#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change
#define ZERO Vec3(0.0, 0.0, 0.0)

class Point {
public:
	Vec3 Position = ZERO;
	Vec3 Velocity = ZERO;
	Vec3 Force = ZERO;
	Vec3 Acceleration = ZERO;
	bool isFixed = false;

	Point(Vec3 pos, Vec3 vel, Vec3 force, bool fixed) {
		Position = pos;
		Velocity = vel;
		Force = force;
		isFixed = fixed;
	}

	void clearForce() {
		Force = ZERO;
		Acceleration = ZERO;
	}
	
};

class Spring {
public:
	int p1;
	int p2;
	float initialLength;
	Spring(int point1, int point2, float initialLen) {
		p1 = point1;
		p2 = point2;
		initialLength = initialLen;
	}
};

class MassSpringSystemSimulator:public Simulator{
public:
	// Construtors
	MassSpringSystemSimulator();
	
	// UI Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Specific Functions
	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force);
	void applyEulerStep(float timeStep);
	void applyMidpointStep(float timeStep);
	void printResults(int springIndex);
	void drawPoints();
	void checkCollison();
	
	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

private:
	// Data Attributes
	std::vector<Point> pointArr;
	std::vector<Spring> springArr;
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	int m_iIntegrator;
	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	int   m_iNumPoints;
	float m_fPointSize;
	int   m_iNumSprings;
	bool m_bGravity = false;
	// functions
	void setupTwoPoints();
	void demo1();
	void demo2();
	void demo3();
	void setupTenPoints();
	void demo4();
};
#endif