#include "MassSpringSystemSimulator.h"
#include <sstream>
#include <iostream>
#include <cmath>
#define FIXED_FLOAT(x) std::fixed <<std::setprecision(4)<<(x)
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
#define ZERO Vec3(0.0, 0.0, 0.0)

int old_demo = -1;
int demo = 1;
Vec3 gravity = ZERO;

void print3dVec(Vec3 v, string s) {
	std::cout << std::fixed;
	std::cout << std::setprecision(4);
	std::cout << "------------------" << std::endl;
	std::cout << s << std::endl;
	std::cout << v.x << " "
		<< v.y << " "
		<< v.z << " "
		<< std::endl;
}

MassSpringSystemSimulator::MassSpringSystemSimulator() {
	m_fPointSize = 0.05f;
	m_fDamping = 0.0f;
	applyExternalForce(Vec3(0, 0, 0));
	setIntegrator(EULER);
}

void MassSpringSystemSimulator::setupTwoPoints(){
	setMass(10.0f);
	setDampingFactor(0.0f);
	setStiffness(40.0f);
	applyExternalForce(Vec3(0, 0, 0));
	int p0 = addMassPoint(Vec3(0.0, 0.0f, 0), Vec3(-1.0, 0.0f, 0), false);
	int p1 = addMassPoint(Vec3(0.0, 2.0f, 0), Vec3(1.0, 0.0f, 0), false);
	addSpring(p0, p1, 1.0);
}

void MassSpringSystemSimulator::demo1() {
	old_demo = 1;
	setupTwoPoints();
	m_bGravity = false;
	if (m_iIntegrator == EULER) {
		applyEulerStep(0.1);
		checkCollison();
	}
	else if (m_iIntegrator == MIDPOINT) {
		applyMidpointStep(0.1);
	}
}

void MassSpringSystemSimulator::demo2() {
	setupTwoPoints();
	setIntegrator(EULER);
	applyEulerStep(0.005);
	checkCollison();
	old_demo = 2;
}

void MassSpringSystemSimulator::demo3() {
	setupTwoPoints();
	setIntegrator(MIDPOINT);
	//applyMidpointStep(0.005);
	applyMidpointStep(0.1);
	old_demo = 3;
}

void MassSpringSystemSimulator::setupTenPoints(){
	setMass(10.0f);
	setDampingFactor(0.0f);
	setStiffness(40.0f);
	m_bGravity = true;
	setIntegrator(EULER);
	applyExternalForce(Vec3(0, 0, 0));
	int p0 = addMassPoint(Vec3(0, 0, 0), Vec3(0, 0.1, 0), false);
	int p1 = addMassPoint(Vec3(0.2, 0.8f, 0), Vec3(0, -.1, 0), false);
	addSpring(p0, p1, 1.0f);
	int p2 = addMassPoint(Vec3(0.1, 0.3, 0.5), Vec3(0, .1, 0), false);
	addSpring(p0, p2, 0.5f);
	int p3 = addMassPoint(Vec3(0.3, 0.5f, 0), Vec3(0, 0.1f, 0), false);
	addSpring(p2, p3, 0.4);
	int p4 = addMassPoint(Vec3(0.5, 0.2f, 0), Vec3(0, 0.0f, 0), false);
	addSpring(p3, p4, 0.6);
	int p5 = addMassPoint(Vec3(0.5, 0.4f, 0), Vec3(0, 0.1f, 0), false);
	addSpring(p4, p5, 0.8);
	int p6 = addMassPoint(Vec3(0.4, 0.0f, 0), Vec3(0, 0.1f, 0), false);
	addSpring(p5, p6, 0.1);
	int p7 = addMassPoint(Vec3(0.6, 0.3f, 0), Vec3(0, 0.1f, 0), false);
	addSpring(p6, p7, 0.2);
	int p8 = addMassPoint(Vec3(0.1, 0.4f, 0), Vec3(0, 0.0f, 0), false);
	addSpring(p7, p8, 0.6);
	int p9 = addMassPoint(Vec3(0.7, 0.2f, 0), Vec3(0, 0.0f, 0), false);
	addSpring(p8, p9, 0.8);
	addSpring(p9, p0, 0.5);
}

void MassSpringSystemSimulator::demo4() {
	old_demo = 4;
	setupTenPoints();
}

const char* MassSpringSystemSimulator::getTestCasesStr() {
	return "EULER,LEAPFROG,MIDPOINT";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	TwAddVarRW(DUC->g_pTweakBar, "Demo #", TW_TYPE_INT32, &demo, "min=1 step=1 max=4");
	TwAddVarRW(DUC->g_pTweakBar, "Point Size", TW_TYPE_FLOAT, &m_fPointSize, "min=0.01 step=0.01");
	TwAddVarRW(DUC->g_pTweakBar, "Apply Gravity", TW_TYPE_BOOLCPP, &m_bGravity, "");
}

void MassSpringSystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
	m_externalForce = ZERO;
	pointArr.erase(pointArr.begin(), pointArr.end());
	//std::cout << "POINT ARR SIZE " << pointArr.size() << std::endl;
	springArr.erase(springArr.begin(), springArr.end());
	//std::cout << "SPRING ARR SIZE " << springArr.size() << std::endl;
}

void MassSpringSystemSimulator::drawPoints(){
	std::mt19937 eng;
	std::uniform_real_distribution<float> randCol(0.0f, 1.0f);
	std::uniform_real_distribution<float> randPos(-0.5f, 0.5f);
    // m_iNumPoints
	for (int i = 0; i < pointArr.size(); i++)
	{
		if (i != 0) {
			if (i == pointArr.size() - 1) {
				DUC->beginLine();
				DUC->drawLine(pointArr[i].Position, Vec3(255.0, 255.0, 255.0), pointArr[0].Position, Vec3(125.0, 125.0, 125.0));
				DUC->endLine();
			}
			else {
				DUC->beginLine();
				DUC->drawLine(pointArr[i].Position, Vec3(255.0, 255.0, 255.0), pointArr[i-1].Position, Vec3(125.0, 125.0, 125.0));
				DUC->endLine();
			}
		}
		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(randCol(eng), randCol(eng), randCol(eng)));
		DUC->drawSphere(pointArr[i].Position, Vec3(m_fPointSize, m_fPointSize, m_fPointSize));
		
	}
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	if (old_demo != demo) {
		reset();
		switch (demo) {
			case 1:
				demo1();
				break;
			case 2:
				demo2();
				break;
			case 3:
				demo3();
				break;
			case 4:
				demo4();
				break;
		}
	}
	drawPoints();
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iIntegrator = testCase;
	setIntegrator(m_iIntegrator);
	switch (m_iIntegrator)
	{
	case 0:
		cout << "EULER !\n";	
		break;
	case 1:
		cout << "LEAPFROG !\n";
		break;
	case 2:
		cout << "MIDPOINT !\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// find a proper scale!
		float inputScale = 0.001f;
		inputWorld = inputWorld * inputScale;
		//m_vfMovableObjectPos = m_vfMovableObjectFinalPos + inputWorld;
		// TODO: FIX User interaction
		for (int i = 0; i < getNumberOfMassPoints(); i++)
		{
			pointArr[i].Position +=  inputScale * (float)m_trackmouse.x * inputWorld;
			pointArr[i].Position += -inputScale * (float)m_trackmouse.y * inputWorld;
		}
	}
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	// update current setup for each frame
	switch (m_iIntegrator)
	{// handling different cases
	case EULER:
		applyEulerStep(timeStep);
		checkCollison();
		break;
	case LEAPFROG:
		/*applyLeapfrogStep(timeStep);*/
		break;
	case MIDPOINT:
		applyMidpointStep(timeStep);
		break;
	default:
		break;
	}
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::setMass(float mass) {
	m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
	Point p = Point(position, Velocity, ZERO, isFixed);

	pointArr.push_back(p);

	return getNumberOfMassPoints() - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	Spring s = Spring(masspoint1, masspoint2, initialLength);
	springArr.push_back(s);
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return pointArr.size();
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	return springArr.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return pointArr[index].Position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return pointArr[index].Velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
	m_externalForce += force;
}


void MassSpringSystemSimulator::checkCollison(){
	for (int i = 0; i < getNumberOfMassPoints(); i++) {
		if (pointArr[i].Position.y < -1) {
			pointArr[i].Position.y = -1.0f;
			pointArr[i].Velocity.y = std::abs(pointArr[i].Velocity.y);
		}
	}
}

void MassSpringSystemSimulator::applyEulerStep(float timeStep){
	Point p1 = Point(ZERO, ZERO, ZERO, false);
	Point p2 = Point(ZERO, ZERO, ZERO, false);
	Vec3 dist, dist_norm = ZERO;
	float length = 0;
	for (int i = 0; i < getNumberOfMassPoints(); i++) {
		pointArr[i].clearForce();
		if (m_bGravity) {
			gravity = Vec3(0.0, -10.0f, 0.0);
		}
	}

	for (int i = 0; i < getNumberOfSprings(); i++) {
		p1 = pointArr[springArr[i].p1];
		p2 = pointArr[springArr[i].p2];

		dist = p1.Position - p2.Position;
		length = std::sqrt(p1.Position.squaredDistanceTo(p2.Position));
		dist_norm = dist / length;
	
		pointArr[springArr[i].p1].Force += -m_fStiffness * (length - springArr[i].initialLength) * dist_norm;
		pointArr[springArr[i].p2].Force += -1 * pointArr[springArr[i].p1].Force;
	}

	for (int i = 0; i < getNumberOfMassPoints(); i++) {
		pointArr[i].Acceleration = (pointArr[i].Force / m_fMass) + gravity + m_externalForce - m_fDamping * pointArr[i].Velocity;

		pointArr[i].Position = pointArr[i].Position + (timeStep * pointArr[i].Velocity);

		pointArr[i].Velocity = pointArr[i].Velocity + (timeStep * pointArr[i].Acceleration);
	}
	
}

void MassSpringSystemSimulator::applyMidpointStep(float timeStep)
{
	Point p1 = Point(ZERO, ZERO, ZERO, false);
	Point p2 = Point(ZERO, ZERO, ZERO, false);
	Vec3 dist, dist_norm, dist_m, dist_m_norm,
		p1vm, p2vm, p1fm, p2fm,
		p1am, p2am, p1xm, p2xm = ZERO;
	float length, length_mid = 0;

	std::vector<Vec3> oldPosArr;
	std::vector<Vec3> oldVelArr;

	for (int i = 0; i < getNumberOfMassPoints(); i++) {
		oldPosArr.push_back(pointArr[i].Position);
		oldVelArr.push_back(pointArr[i].Velocity);
	}

	applyEulerStep(timeStep / 2);

	for (int i = 0; i < getNumberOfMassPoints(); i++) {
		pointArr[i].clearForce();
		if (m_bGravity) {
			gravity = Vec3(0.0, -10.0f, 0.0);
		}
	}

	for (int i = 0; i < getNumberOfSprings(); i++) {
		p1 = pointArr[springArr[i].p1];
		p2 = pointArr[springArr[i].p2];

		dist = p1.Position - p2.Position;
		length = std::sqrt(p1.Position.squaredDistanceTo(p2.Position));
		dist_norm = dist / length;

		pointArr[springArr[i].p1].Force += -m_fStiffness * (length - springArr[i].initialLength) * dist_norm;
		pointArr[springArr[i].p2].Force += -1 * pointArr[springArr[i].p1].Force;
	}


	for (int i = 0; i < getNumberOfMassPoints(); i++) {
		pointArr[i].Acceleration = (pointArr[i].Force / m_fMass) + gravity + m_externalForce - m_fDamping * pointArr[i].Velocity;;

		pointArr[i].Position = oldPosArr[i] + (timeStep * pointArr[i].Velocity);

		pointArr[i].Velocity = oldVelArr[i] + (timeStep * pointArr[i].Acceleration);
	}

	checkCollison();
}


void MassSpringSystemSimulator::printResults(int springIndex)
{
	std::cout << "\nSpring #" << springIndex << std::endl;
	std::cout << std::fixed;
	std::cout << std::setprecision(4);
	print3dVec(pointArr[springArr[springIndex].p1].Position, "Position p1");
	print3dVec(pointArr[springArr[springIndex].p2].Position, "Position p2");
	print3dVec(pointArr[springArr[springIndex].p1].Velocity, "Velocity p1");
	print3dVec(pointArr[springArr[springIndex].p2].Velocity, "Velocity p2");
}



