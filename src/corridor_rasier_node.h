#ifndef CORRIDOR_RASIER_NODE_H
#define CORRIDOR_RASIER_NODE_H

#include <eigen3/Eigen/Core>
#include <nav_msgs/Odometry.h>

#define NODE_NAME "corridor_raiser"
#include "sparcsnode.h"

#include "corridor_raiser.h"
#include "multi_array_manager.h"

class CorridorRaiserNode : public SparcsNode
{
	CorridorRaiser *corridorRaiser;

	void initParams ();
	void initROS ();
	int actions ();

	void wallsTensorToMsg(const std::vector<Eigen::MatrixXd> &wallsEigen, std_msgs::Float32MultiArray &wallsMsg);
	void corridorMatrixToMsg(const Eigen::MatrixXd &corridorEigen, std_msgs::Float32MultiArray &corridorMsg);

public:
	CorridorRaiserNode ();
	void odomCallback(const nav_msgs::Odometry &odomMsg);
};

#endif // CORRIDOR_RASIER_NODE_H
