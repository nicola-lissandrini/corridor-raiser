#include "corridor_raiser_visual_plugin.h"
#include "multi_array_manager.h"
#include <cstdarg>

using namespace gazebo;
using namespace rendering;
using namespace ignition::math;
using namespace physics;
using namespace std;
using namespace ros;

#define NODE_NAME "corridor_raiser_visual_plugin"

CorridorRaiserVisualPlugin::CorridorRaiserVisualPlugin():
	VisualPlugin ()
{
}

CorridorRaiserVisualPlugin::~CorridorRaiserVisualPlugin() {
	sem_unlink ("only_visual");
	cout << "---------------- unlinking" << endl;
}

void CorridorRaiserVisualPlugin::Load (VisualPtr _visual, sdf::ElementPtr _sdf)
{
	semaphore = sem_open ("/corridor_semaphore", O_CREAT, S_IWUSR|S_IRUSR, 1);

	if (semaphore == SEM_FAILED) {
		sem_unlink ("/corridor_semaphore");
		switch (errno) {
		case EACCES:
			cout << "The semaphore exists, but the caller does not have permission to open it." << endl;
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

	/*sem_t *unosolo = sem_open("/only_visual", O_CREAT|O_EXCL, S_IWUSR|S_IRUSR, 1);
	if (unosolo == SEM_FAILED) {
		cout << "GIA CREATOOOOOOO" << endl;
		sem_unlink ("/only_visual");
		return;
	} else {
		cout << "----- PRIMA VISUAL" << endl;
	}*/

	unique = ::getppid ();
	update = false;
	time = 0;
	scene = _visual->GetScene ();
	walls.resize (N_WALLS);
	visualOldId.resize (N_WALLS);
	first.resize (N_WALLS);
	first = {true, true};

	initROS ();

	updateConnection = event::Events::ConnectRender (
				boost::bind(&CorridorRaiserVisualPlugin::UpdateChild, this));
}

void CorridorRaiserVisualPlugin::queueThread () {
	static const double timeout = 0.01;
	while (rosNode->ok ()) {
		rosQueue.callAvailable (WallDuration(timeout));
	}
}


void CorridorRaiserVisualPlugin::initROS ()
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
															 boost::bind(&CorridorRaiserVisualPlugin::wallsCallback, this, _1),
															 ros::VoidPtr (), &rosQueue);

	rosQueueThread = thread (bind (&CorridorRaiserVisualPlugin::queueThread, this));

	wallsSub = rosNode->subscribe (so);
}


void CorridorRaiserVisualPlugin::wallsCallback (const std_msgs::Float32MultiArrayConstPtr &wallsMsg)
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

		// gzerr << "Muro 1\n" << walls[0][i] << endl << "Muro 2\n" << walls[1][i] << endl << endl;
	}

	update = true;
}

string CorridorRaiserVisualPlugin::getCurrentVisualName (int i) {
	return "corridor_1_visual_" + to_string (i) + "_" + to_string (time) + "_"  + std::to_string (unique);
}
void CorridorRaiserVisualPlugin::polylineFromWalls (msgs::Polyline *poly, int wallId)
{
	vector<Vector2d> wall = walls[wallId];
	semaphore = sem_open ("/corridor_semaphore", O_CREAT, 0777, 1);

	for (int i = 0; i < wall.size (); i++) {
		msgs::Set (poly->add_point (), wall[i]);
	}

	poly->set_height (0.3);
}


msgs::Visual CorridorRaiserVisualPlugin::buildVisualMsg (int i)
{
	msgs::Visual visualMsg;

	visualMsg.set_name (getCurrentVisualName (i));
	visualMsg.set_parent_name (scene->Name ());
	visualMsg.mutable_geometry ()->set_type (msgs::Geometry::POLYLINE);
	polylineFromWalls (visualMsg.mutable_geometry ()->add_polyline (), i);

	return visualMsg;
}

void CorridorRaiserVisualPlugin::replaceVisual (const msgs::Visual &visualMsg, int i)
{
	if (!first[i])
		scene->RemoveVisual (visualOldId[i]);
	else
		first[i] = false;

	auto msgPtr = new ConstVisualPtr (&visualMsg);
	VisualPtr visualNew;

	visualNew.reset (new Visual (getCurrentVisualName (i),scene));
	visualNew->LoadFromMsg (*msgPtr);
	visualNew->SetVisible (true);

	scene->AddVisual (visualNew);

	visualOldId[i] = visualNew->GetId ();
}

void CorridorRaiserVisualPlugin::updateVisual (int i)
{
	msgs::Visual visualMsg = buildVisualMsg (i);
	replaceVisual (visualMsg, i);
}

void CorridorRaiserVisualPlugin::UpdateChild ()
{
	if (update) {
		sem_wait (semaphore);
		updateVisual (0);
		updateVisual (1);

		update = false;
		time++;
		sem_post (semaphore);
	}
}


GZ_REGISTER_VISUAL_PLUGIN(CorridorRaiserVisualPlugin)
