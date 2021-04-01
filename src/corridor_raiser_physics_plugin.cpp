#include "corridor_raiser_physics_plugin.h"
#include <semaphore.h>
#include <errno.h>

using namespace gazebo;
using namespace physics;
using namespace gazebo::rendering;
using namespace ignition::math;
using namespace ros;
using namespace std;

CorridorRaiserPlugin::CorridorRaiserPlugin():
	ModelPlugin ()
{}

CorridorRaiserPlugin::~CorridorRaiserPlugin ()
{
	sem_unlink ("/corridor_semaphore");
}

void CorridorRaiserPlugin::Load (ModelPtr _model, sdf::ElementPtr _sdf)
{
	semaphore = sem_open ("/corridor_semaphore", O_CREAT, S_IWUSR|S_IRUSR, 1);

	if (semaphore == SEM_FAILED) {
		sem_unlink ("/corridor_semaphore");
		switch (errno) {
		case EACCES:
			cout << "The semaphore exists, but the caller does not have permission to open it." << endl;
			sem_unlink ("/corridor_semaphore");
			return;
			break;

		case ENOENT:
			cout << "O_CREAT was specified, but name wasn't well formed." << endl;
			return;
			break;
		default:
			cout << "num " << endl;
		}
	}
	GZQUA;
	update = false;
	time = 0;
	walls.resize (N_WALLS);

	engine = _model->GetWorld ()->Physics ();
	link = _model->GetChildLink ("corridor_1_link_physics");
	model = _model;

	oldName.resize (N_WALLS);
	oldName[0] = "null";
	oldName[1] = "null";

	initROS ();

	updateConnection = event::Events::ConnectWorldUpdateBegin (
				boost::bind(&CorridorRaiserPlugin::OnUpdate, this, _1));
}

void CorridorRaiserPlugin::queueThread () {
	static const double timeout = 0.01;
	while (rosNode->ok ()) {
		rosQueue.callAvailable (WallDuration(timeout));
	}
}

void CorridorRaiserPlugin::wallsCallback (const std_msgs::Float32MultiArrayConstPtr &wallsMsg)
{
	MultiArray32Manager wallsArray(*wallsMsg);

	int wallSize = wallsArray.getMsg ().layout.dim[1].size;

	walls[0].clear ();
	walls[1].clear ();

	for (int i = 0; i < wallSize; i++) {
		walls[0].push_back (Vector2d (wallsArray.get ({0, i, 0}),
									  wallsArray.get ({0, i, 1})));

		walls[1].push_back (Vector2d (wallsArray.get ({1, i, 0}),
									  wallsArray.get ({1, i, 1})));

		//gzerr << "Muro 1\n" << walls[0][i] << endl << "Muro 2\n" << walls[1][i] << endl << endl;
	}

	update = true;
}

void CorridorRaiserPlugin::initROS ()
{
	// Init ROS
	if (!ros::isInitialized ()) {
		int argc = 0;
		char **argv = NULL;

		ros::init (argc, argv, NODE_NAME);
	}

	rosNode = new NodeHandle (NODE_NAME);
	rosNode->setCallbackQueue (&rosQueue);

	SubscribeOptions so =
			SubscribeOptions::create<std_msgs::Float32MultiArray> ("/corridor_raiser/walls", 1,
																   boost::bind(&CorridorRaiserPlugin::wallsCallback, this, _1),
																   ros::VoidPtr (), &rosQueue);

	rosQueueThread = thread (bind (&CorridorRaiserPlugin::queueThread, this));

	wallsSub = rosNode->subscribe (so);
}

void CorridorRaiserPlugin::polylineFromWalls (msgs::Polyline *poly, int wallId)
{
	vector<Vector2d> wall = walls[wallId];

	for (int i = 0; i < wall.size (); i++) {
		msgs::Set (poly->add_point (), wall[i]);
	}

	poly->set_height (1);
}


msgs::Geometry CorridorRaiserPlugin::buildGeometryMsg (int i)
{
	msgs::Geometry geometryMsg;

	geometryMsg.set_type (msgs::Geometry::POLYLINE);
	polylineFromWalls (geometryMsg.add_polyline (), i);

	return geometryMsg;
}

string CorridorRaiserPlugin::getCollisionName (int i) {
	return "corridor_1_collision_" + to_string (i);
}


void CorridorRaiserPlugin::updateCollision (int i)
{

	if (oldName[i] != "null"){
		LinkPtr old = model->GetLink (oldName[i]);
		gzmsg << old->GetName () << endl;
		model->RemoveChild (boost::reinterpret_pointer_cast<Entity> (old));
		old->Fini ();
	}
	LinkPtr newLink = model->CreateLink ("corridor_1_link_dyn_" + to_string(i) + "_" + to_string(time));
	CollisionPtr collision = engine->CreateCollision ("polyline",newLink);
	sdf::ElementPtr linkSdf(new sdf::Element);
	sdf::ElementPtr collisionSdf(new sdf::Element);
	PolylineShapePtr polyline = boost::reinterpret_pointer_cast<PolylineShape> (engine->CreateShape ("polyline",collision)); // = (collision->GetShape ());

	msgs::Geometry geometryMsg = buildGeometryMsg (i);

	collisionSdf->Copy (collision->GetSDF ()->Clone ());
	polyline->ProcessMsg (geometryMsg);
	polyline->Init ();

	collisionSdf->GetElement ("geometry")->AddElement ("polyline")->Copy (polyline->GetSDF ()->Clone ());
	collisionSdf->GetAttribute ("name")->Set (getCollisionName (i));

	linkSdf->Copy (link->GetSDF ()->Clone ());
	linkSdf->GetElement ("collision")->Copy (collisionSdf->Clone ());
	linkSdf->GetAttribute ("name")->Set (newLink->GetName ());

	newLink->Load (linkSdf);
	newLink->Init ();

	model->AddChild (newLink);
	model->Init ();
	oldName[i] =  newLink->GetName ();
}


void CorridorRaiserPlugin::OnUpdate (const common::UpdateInfo &_info)
{
	if (update) {
		sem_wait (semaphore);
		updateCollision (0);
		updateCollision (1);

		update = false;
		time++;
		sem_post (semaphore);
	}
}


GZ_REGISTER_MODEL_PLUGIN(CorridorRaiserPlugin)
