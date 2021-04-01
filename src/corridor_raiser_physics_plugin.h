#ifndef CORRIDOR_RAISER_PLUGIN_H
#define CORRIDOR_RAISER_PLUGIN_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/ode/ODECollision.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/rendering/Scene.hh>

#define NODE_NAME "corridor_raiser"
#include "sparcsnode.h"
#include "multi_array_manager.h"
#include <semaphore.h>

#define GZQUA {gzerr << "\e[33mReached " << __PRETTY_FUNCTION__ << "\e[0m:" << __LINE__ << endl;}

#define N_WALLS 2

namespace gazebo {

class CorridorRaiserPlugin : public ModelPlugin
{
	ros::NodeHandle *rosNode;
	ros::Subscriber wallsSub;
	std::thread rosQueueThread;
	ros::CallbackQueue rosQueue;

	sem_t *semaphore;

	physics::LinkPtr link;
	physics::ModelPtr model;
	std::vector<std::string> oldName;
	physics::PhysicsEnginePtr engine;
	event::ConnectionPtr updateConnection;

	std::vector<std::vector<ignition::math::Vector2d>> walls;

	bool first;
	bool update;
	int time;

	void initROS();
	void queueThread();

	std::string getCollisionName(int i);
	void polylineFromWalls(msgs::Polyline *poly, int wallId);
	msgs::Geometry buildGeometryMsg(int i);
	void updateCollision(int i);
	void OnUpdate (const common::UpdateInfo& _info);

public:
	CorridorRaiserPlugin ();

	void Load (physics::ModelPtr _model, sdf::ElementPtr _sdf);
	void wallsCallback (const std_msgs::Float32MultiArrayConstPtr &wallsMsg);
	~CorridorRaiserPlugin();
};

}

#endif // CORRIDOR_RAISER_PLUGIN_H
