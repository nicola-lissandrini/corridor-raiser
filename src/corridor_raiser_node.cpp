#include "corridor_rasier_node.h"
#include <std_msgs/Float32MultiArray.h>

using namespace std;
using namespace ros;
using namespace Eigen;

CorridorRaiserNode::CorridorRaiserNode ():
	SparcsNode(NODE_NAME)
{
	initParams ();
	initROS ();
}

void CorridorRaiserNode::initParams ()
{
	CorridorRaiserParams corridorParams;

	corridorParams.averageWidth = paramDouble (params,"average_corridor_width");
	corridorParams.splineDensity = paramDouble (params,"spline_densitiy");
	corridorParams.keypointsCount = paramDouble (params, "keypoints_count");
	corridorParams.wallThickness = paramDouble (params,"wall_thickness");
	corridorParams.keypointsVariance = paramDouble (params, "keypoints_variance");
	corridorParams.keypointsRadius = paramDouble (params, "keypoints_radius");
	corridorParams.widthSpatialFrequency = paramDouble (params, "width_spatial_frequency");
	corridorParams.widthTemporalFrequency = paramDouble (params, "width_temporal_frequency");
	corridorParams.angleAmplitude = paramDouble (params, "angle_amplitude");
	corridorParams.widthAmplitude = paramDouble (params,"width_amplitude");
	corridorParams.angleFrequency = paramDouble (params, "angle_frequency");
	corridorParams.staticWalls = paramBool (params, "static_walls");
	corridorParams.rate = paramDouble (params, "rate");
	corridorRaiser = new CorridorRaiser (corridorParams);
}

void CorridorRaiserNode::initROS ()
{
	addSub ("odom_sub", paramString (params, "odom_topic_sub"), 1, &CorridorRaiserNode::odomCallback);

	addPub<std_msgs::Float32MultiArray> ("walls_pub", paramString (params, "wall_topic_pub"), 1);
	addPub<std_msgs::Float32MultiArray> ("corridor_pub",paramString (params,"corridor_topic_pub"), 1);
}

void CorridorRaiserNode::odomCallback (const nav_msgs::Odometry &odomMsg) {
	corridorRaiser->setPosition (Eigen::Vector2d (odomMsg.pose.pose.position.x,
												  odomMsg.pose.pose.position.y));
}

void CorridorRaiserNode::wallsTensorToMsg(const vector<MatrixXd> &wallsEigen, std_msgs::Float32MultiArray &wallsMsg)
{
	// Todo: input sizes check

	MultiArray32Manager array({wallsEigen.size (),
							  wallsEigen[0].rows (),
							  wallsEigen[0].cols ()});

	for (int i = 0; i < array.size (0); i++)
		for (int j = 0; j < array.size (1); j++)
			for (int k = 0; k < array.size (2); k++){
				array.set ({i,j,k}, wallsEigen[i].coeff (j, k));
			}

	wallsMsg = array.getMsg ();
}

void CorridorRaiserNode::corridorMatrixToMsg (const MatrixXd &corridorEigen, std_msgs::Float32MultiArray &corridorMsg)
{
	MultiArray32Manager array ({corridorEigen.rows (), corridorEigen.cols()});

	for (int i = 0; i < array.size (0); i++)
		for (int j = 0; j < array.size (1); j++)
			array.set ({i,j}, corridorEigen.coeff (i,j));

	corridorMsg = array.getMsg ();
}

int CorridorRaiserNode::actions ()
{
	vector<MatrixXd> walls = corridorRaiser->newWalls ();
	MatrixXd corridor = corridorRaiser->getCorridor ();
	std_msgs::Float32MultiArray wallsMsg, corridorMsg;

	wallsTensorToMsg (walls, wallsMsg);
	corridorMatrixToMsg (corridor, corridorMsg);

	publish ("walls_pub", wallsMsg);
	publish ("corridor_pub", corridorMsg);

	return 0;
}

int main (int argc, char *argv[])
{
	init (argc, argv, NODE_NAME);

	CorridorRaiserNode crn;

	return crn.spin ();

}
