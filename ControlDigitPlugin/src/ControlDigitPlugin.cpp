#include "ControlDigitPlugin.hh"

/*
* Suppose one link and one sensor
*/
using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(ControlPlugin)

/* Initilize the PID controllers. */
common::PID pidControlX(5, 5, 5);
common::PID pidControlY(5, 0, 5);
common::PID pidControlZ(5, 0, 5);

ControlPlugin::ControlPlugin()
{

}

ControlPlugin::~ControlPlugin()
{

}

void ControlPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
	/* Store the pointer of the model. */
	this->model_ = _model;

    /*Update the posiiton by calling the UpdatePosition method. */
	this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&ControlPlugin::UpdatePosition, this));



void ControlPlugin::UpdatePosition()
{
	/* Store the start time. */
	auto start = std::chrono::steady_clock::now();

	/* Check whether it is the first cycle or not. */
	if (this->flag_ == false)
	{
		this->lastTime_ = start;
		this->flag_ = true;
		return;
	}

	/* Get the position from gazebo. */
	ignition::math::Pose3d pose = this->model_->WorldPose();

	/* Store the time difference. */
	double elapsed = std::chrono::duration<double>(start - this->lastTime_).count();

	/* Compute the sinusoidal movement. */
	this->time_ += elapsed ;
	this->position_ = sin(this->time_ * 0.3) * 0.05;

	/* Compute the errors for the PIDs controllers. */
	/* Along the x axis, the sensor has to follow the sinusoidal movement. */
	/* Along the y axis, the sensor has to stay at 0 coordinate. */
	/* Along the z axis, the sensor has to stay at 0.0048, hard wired height of the sdf file. */
	double controlX = pidControlX.Update(pose.X() - this->position_, elapsed );
	double controlY = pidControlY.Update(pose.Y(), elapsed);
	double controlZ = pidControlZ.Update(pose.Z(), elapsed);

	this->model_->GetLink("base")->SetForce(ignition::math::Vector3d(controlX, controlY, controlZ));

	/* Store the time for the next cycle. */
	this->lastTime_ = start;
}
