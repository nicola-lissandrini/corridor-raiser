#include "corridor_raiser.h"
#include <eigen3/unsupported/Eigen/Splines>


#include <random>

using namespace Eigen;
using namespace std;

CorridorRaiser::CorridorRaiser(const CorridorRaiserParams &_params):
	params(_params),
	time(0),
	keyPoints(_params.keypointsCount,N_DIM),
	corridor(_params.splineDensity + 1, N_DIM),
	angles(_params.keypointsCount - 1),
	phases(_params.keypointsCount - 1),
	widthShift(0),
	gen(rd())
{
	flags.addFlag ("first_odometry",true);
	initAngles ();
}

void CorridorRaiser::setPosition (const Vector2d &_robotPos) {
	robotPos = _robotPos;

	flags.set ("first_odometry");
}

int CorridorRaiser::getClosestKeypoint ()
{
	MatrixXd dist = keyPoints;

	dist.rowwise() -= robotPos.transpose ();

	Eigen::Index argmin;
	dist.rowwise().norm ().minCoeff (&argmin);

	return (int) argmin;
}

void CorridorRaiser::updateCorridor()
{
	Spline2d spline = SplineFitting<Spline2d>::Interpolate(keyPoints.transpose (), N_DIM);

	double time = 0;
	for(int i = 0; i < params.splineDensity + 1; i++) {
		corridor.row(i) << spline (time).transpose ();
		time += 1.0 / double (params.splineDensity);
	}
}

MatrixXd CorridorRaiser::getRotatedDiff ()
{
	MatrixXd rotatedDiff(params.splineDensity, N_DIM);

	for (int i = 0; i < rotatedDiff.rows (); i++) {
		Vector2d currDiff = (corridor.row (i+1) - corridor.row(i)).normalized ();

		// Perform a CCW rotation by pi/2
		// [0, -1]
		// [1,  0]
		rotatedDiff.row (i) << -currDiff[1], currDiff[0];
	}

	return rotatedDiff;
}

double CorridorRaiser::getNormal (double variance)
{
	if (variance == 0)
		return 0;
	else {
		normal_distribution<double> nd(0.0, params.keypointsVariance);
		return nd(gen);
	}
}

double CorridorRaiser::getUniform () {
	uniform_real_distribution<double> ud(0.0,  1.0);

	return ud(gen);
}

void CorridorRaiser::initAngles ()
{
	angles[0] = getNormal( params.keypointsVariance);
	phases[0] = getUniform () * 2 * M_PI;
	for (int i = 1; i < params.keypointsCount - 1; i++) {
		angles[i] = angles[i-1] + getNormal (params.keypointsVariance);
		phases[i] = getUniform () * 2 * M_PI;
	}
}

double CorridorRaiser::getWidth (double tau, double t)
{
	return params.averageWidth + params.widthAmplitude * cos (2 * M_PI * params.widthSpatialFrequency * tau -
									  2 * M_PI * params.widthTemporalFrequency * t * (params.staticWalls ? 0:1));
}

void CorridorRaiser::updateAngles ()
{
	for (int i = 1; i < angles.size (); i++)
		angles[i] += params.angleAmplitude * sin (2 * M_PI *  params.angleFrequency * time + phases[i]);
}

void CorridorRaiser::addKeypoints ()
{
	keyPoints.conservativeResize (keyPoints.rows () + 1, keyPoints.cols ());

	angles.conservativeResize (angles.size () + 1);
	phases.conservativeResize (phases.size () + 1);

	angles[angles.size ()-1] = angles[angles.size()-2] + getNormal (params.keypointsVariance);
	phases[phases.size ()-1] = getUniform () * 2 * M_PI;

}

void CorridorRaiser::updateKeypoints ()
{
	updateAngles ();

	keyPoints.row (0) << 0, 0;

	int closest = getClosestKeypoint ();

	if (closest + params.keypointsCount > keyPoints.rows ()) {
		addKeypoints ();
	}

	for (int i = closest + 1; i < keyPoints.rows (); i++) {
		double currAngle = angles[i-1];
		Vector2d currPoint (params.keypointsRadius * cos (currAngle), params.keypointsRadius * sin(currAngle));
		keyPoints.row (i) << (currPoint.transpose () + keyPoints.row(i-1));
	}
}

MatrixXd CorridorRaiser::getWallPolyline (double coeff)
{
	MatrixXd directions = coeff * getRotatedDiff ();
	MatrixXd outer(params.splineDensity, N_DIM);
	MatrixXd inner(params.splineDensity, N_DIM);
	MatrixXd stacked(inner.rows () + outer.rows (), inner.cols());

	for (int i = 0; i < directions.rows (); i++) {
		inner.row(i) = corridor.row(i) + getWidth (i, time) * directions.row(i);
		outer.row(i) = corridor.row(i) + (getWidth (i, time) + params.wallThickness) * directions.row(i);
	}

	stacked << inner, outer.colwise ().reverse();

	return stacked;
}

MatrixXd CorridorRaiser::getCorridor () {
	return corridor;
}

std::vector<MatrixXd> CorridorRaiser::newWalls()
{
	vector<MatrixXd> wallsPolyline;

	if (!params.staticWalls || time == 0) {
		QUA;
		updateKeypoints ();
		updateCorridor ();
	}

	wallsPolyline.resize (2);

	wallsPolyline[0] = getWallPolyline (LEFT_WALL_COEFF);
	wallsPolyline[1] = getWallPolyline (RIGHT_WALL_COEFF);

	time += 1 / params.rate;

	return wallsPolyline;
}
