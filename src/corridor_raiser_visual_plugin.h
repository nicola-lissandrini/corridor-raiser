#ifndef CORRIDOR_RAISER_VISUAL_PLUGIN_H
#define CORRIDOR_RAISER_VISUAL_PLUGIN_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/rendering/Scene.hh>

#include <std_msgs/Float32MultiArray.h>
#include <semaphore.h>

#define N_WALLS 2
#define GZQUA {gzerr << "\e[33mReached " << __PRETTY_FUNCTION__ << "\e[0m:" << __LINE__ << endl;}

namespace gazebo {

class CorridorRaiserVisualPlugin : public VisualPlugin
{
	ros::NodeHandle *rosNode;
	ros::Subscriber wallsSub;
	std::thread rosQueueThread;
	ros::CallbackQueue rosQueue;

	rendering::ScenePtr scene;
	event::ConnectionPtr updateConnection;
	sem_t *semaphore;

	std::vector<std::vector<ignition::math::Vector2d>> walls;
	std::vector<int> visualOldId;
	std::vector<bool> first;
	bool update;
	int time;
	int unique;

	void initROS();

	void queueThread();
	void polylineFromWalls(msgs::Polyline *poly, int wallId);
	std::string getCurrentVisualName (int i);
	msgs::Visual buildVisualMsg (int i);
	void replaceVisual (const msgs::Visual &visualMsg, int i);
	void updateVisual (int i);
	void UpdateChild ();

public:
	CorridorRaiserVisualPlugin();
	~CorridorRaiserVisualPlugin();
	void Load (rendering::VisualPtr _visual, sdf::ElementPtr _sdf);

	void wallsCallback (const std_msgs::Float32MultiArrayConstPtr &wallsMsg);
};

}

#endif // CORRIDOR_RAISER_VISUAL_PLUGIN_H
