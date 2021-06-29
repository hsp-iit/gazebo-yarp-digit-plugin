#include "RendererDigitPlugin.hh"

/*
* Suppose one link and one sensor
*/
using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(RendererDigitPlugin)

RendererDigitPlugin::RendererDigitPlugin()
{

}

RendererDigitPlugin::~RendererDigitPlugin()
{

}

void RendererDigitPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
	/* Store the pointer of the model. */
	this->model = model;

    const gazebo::physics::Link_V link = this->model->GetLinks();

    /* Link_V to LinkPtr. */
    const gazebo::physics::LinkPtr linkPtr = link[0];

    /* Get the sensor element from the SDF. */
    this->sdf = (linkPtr->GetSDF())->GetElement("sensor");

    /* Store the pointer to the contact sensor. */
    std::string localSensorName = this->sdf->GetAttribute("name")->GetAsString();

    /* Retrieve the scoped name of the sensor. */
    std::vector<std::string> scopedNameList = this->model->SensorScopedName(localSensorName);

    /* Istantiate the sensor manager. */
    gazebo::sensors::SensorManager *sensorMgr = gazebo::sensors::SensorManager::Instance();

	/* Check if the queue of the SensorManager is empty. */
	if (!sensorMgr->SensorsInitialized())
       return;

    /* Assign the sensor to the pointer. */
    gazebo::sensors::SensorPtr genericPtr = sensorMgr->GetSensor(scopedNameList[0]);
    this->sensor = std::dynamic_pointer_cast<sensors::ContactSensor>(genericPtr);

	/* Check the sensor. */
    if (!this->sensor)
    {
      gzerr<< "ContactSensorPlugin requires a ContactSensor .\n";
      return;
    }

    /* Activate the sensor .*/
    this->sensor->SetActive(true);

    /*Update the posiiton by calling the UpdatePosition method. */
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&RendererDigitPlugin::UpdatePosition, this));


}


void RendererDigitPlugin::UpdatePosition()
{

	/* Get the position from gazebo. */
	ignition::math::Pose3d pose = this->model->WorldPose();

	/* Retrieve the contacts. */
	msgs::Contacts contacts;
	contacts = this->sensor->Contacts();

	for (unsigned int i = 0; i < contacts.contact_size(); ++i)
	{
	    /* Print the name of the 2 objects in contact. */
	    std::cout<<"collision between []" << contacts.contact(i).collision1()<< "] and []"
		        << contacts.contact(i).collision2() << "]\n";

	    for (unsigned int j=0 ; j<contacts.contact(i).position_size(); ++j)
	    {
	        /* Print the posiition of the contacts. */
	        std::cout << j<< " Position of the contact (x,y,z):"
	              << contacts.contact(i).position(j).x() << " "
	              << contacts.contact(i).position(j).y() << " "
	              << contacts.contact(i).position(j).z() << "\n";

	        /* Print the magnitude of the normal forces. */
		    std::cout << " Normal forces (x,y,z):"
	              << contacts.contact(i).wrench(j).body_2_wrench().force().x()<< " "
	              << contacts.contact(i).wrench(j).body_2_wrench().force().y() << " "
	              << contacts.contact(i).wrench(j).body_2_wrench().force().z() << "\n";

	    }
	}
}
