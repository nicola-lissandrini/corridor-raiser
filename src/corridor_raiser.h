#ifndef CORRIDOR_RAISER_H
#define CORRIDOR_RAISER_H

#include <eigen3/Eigen/Core>
#include <vector>
#include <random>

#include <iostream>
#include "common.h"

#define N_DIM 2
#define LEFT_WALL_COEFF (+1)
#define RIGHT_WALL_COEFF (-1)

struct CorridorRaiserParams {
	double averageWidth;
	double wallThickness;
	double keypointsVariance;
	double keypointsRadius;
	double widthTemporalFrequency;
	double widthSpatialFrequency;
	double widthAmplitude;
	double angleAmplitude;
	double angleFrequency;
	double rate;
	int keypointsCount;
	int splineDensity;
	bool staticWalls;
};

class CorridorRaiser
{
	CorridorRaiserParams params;
	double time;
	Eigen::MatrixXd keyPoints;
	Eigen::MatrixXd corridor;
	Eigen::VectorXd angles;
	Eigen::VectorXd phases;
	Eigen::Vector2d robotPos;
	ReadyFlags<std::string> flags;
	double widthShift;
    std::random_device rd;
    std::mt19937 gen;

	double getNormal (double variance);
	double getUniform ();
	double getWidth (double tau, double t);

	int getClosestKeypoint ();

	void addKeypoints ();
	void initAngles ();
	void updateAngles ();
	void updateKeypoints ();
	void updateCorridor ();
	Eigen::MatrixXd getRotatedDiff ();
	Eigen::MatrixXd getWallPolyline (double coeff);


public:
	CorridorRaiser (const CorridorRaiserParams &_params);

	std::vector<Eigen::MatrixXd>  newWalls ();
	void setPosition (const Eigen::Vector2d &_robotPos);
	Eigen::MatrixXd getCorridor();
};

#endif // CORRIDOR_RAISER_H
