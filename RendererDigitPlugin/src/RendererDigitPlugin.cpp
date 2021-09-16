/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

 #include "RendererDigitPlugin.hh"

using namespace pybind11::literals;
GZ_REGISTER_MODEL_PLUGIN(gazebo::RendererPlugin)


gazebo::RendererPlugin::RendererPlugin()
{}

gazebo::RendererPlugin::~RendererPlugin()
{}

void gazebo::RendererPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{

    /* Store the pointer of the sensor model. */
    sensor_model_ = model;

    /* Store the pointer of the ball model. */
    physics::WorldPtr world_ptr = sensor_model_->GetWorld();
    ball_model_ = world_ptr->ModelByName("sphere");

    /* Initialize the rendering thread. */
    std::thread rendering_thread(&gazebo::RendererPlugin::RenderingThread, this);
    rendering_thread.detach();

    /* Get the link of the model. */
    const gazebo::physics::Link_V link = sensor_model_->GetLinks();
    const gazebo::physics::LinkPtr linkPtr = link[0];

    /* Get the sensor element from the SDF. */
    sdf_ = (linkPtr->GetSDF())->GetElement("sensor");

    /* Store the pointer to the contact sensor. */
    std::string localSensorName = sdf_->GetAttribute("name")->GetAsString();

    /* Retrieve the scoped name of the sensor. */
    std::vector<std::string> scopedNameList = sensor_model_->SensorScopedName(localSensorName);

    /* Istantiate the sensor manager. */
    gazebo::sensors::SensorManager* sensorMgr = gazebo::sensors::SensorManager::Instance();

    /* Check if the queue of the SensorManager is empty. */
    if (!sensorMgr->SensorsInitialized())
    {
       return;
    }

    /* Assign the sensor to the pointer. */
    gazebo::sensors::SensorPtr genericPtr = sensorMgr->GetSensor(scopedNameList[0]);
    sensor_ = std::dynamic_pointer_cast<sensors::ContactSensor>(genericPtr);

    /* Check the sensor. */
    if (!sensor_)
    {
      gzerr << "ContactSensorPlugin requires a ContactSensor .\n";
      return;
    }

    /* Activate the sensor .*/
    sensor_->SetActive(true);

    /*Update the posiiton by calling the UpdatePosition method. */
    updateConnection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&RendererPlugin::UpdatePosition, this));
}

void gazebo::RendererPlugin::RenderingThread()
{
    /* Initialize the interpreter. */
    pybind11::scoped_interpreter guard;

    /**
     * Add the absolute path to the 'sensor' and 'tacto' python modules.
     * The definition 'SOURCE_PATH' is set in the CMakeLists.txt.
     */
    std::string tacto_cpp_wrapper_path_ = std::string(SOURCE_PATH) + "/tacto-cpp-wrapper";
    std::string tacto_path_ = std::string(SOURCE_PATH) + "/build/_deps/tacto-src";

    pybind11::object sys_path_insert = pybind11::module::import("sys").attr("path").attr("insert");
    sys_path_insert(0, tacto_cpp_wrapper_path_ + "/python");
    sys_path_insert(0, tacto_path_);

    /**
     * Running the interpreter from here will cause sys.argv to be empty.
     * Hence, a fake entry is added given that many python modules relies on the availability of sys.argv[0].
     */
    pybind11::module::import("sys").attr("argv").attr("insert")(0, "");
    pybind11::module::import("os").attr("environ");

    /* Import the class from the module. */
    pybind11::object sensor_module = pybind11::module::import("sensor").attr("Sensor");

    /* Initialize the name of the object. */
    pybind11::str ball_ = "ball";

    /* Initialize YARP port. */
    yarp::os::Network yarp;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> port;
    port.open("/gazebo-yarp-digit-plugin/output:o");
    pybind11::array_t<uint8_t> rgb;

    /* Initialize the output image. */
    cv::Mat img;

    /* Instantiate an object of the class. */
    pybind11::object sensor_digit = sensor_module
    (
        "background_path"_a = tacto_path_ + "/examples/conf/bg_digit_240_320.jpg",
        "configuration_path"_a = tacto_path_ + "/tacto/config_digit.yml"
    );

    /**
    * Get the pose of the ball.
    * At the moment, the object is fixed to the ground.
    */
    ignition::math::Pose3d pose_ball = ball_model_->GetLink()->WorldPose();

    /* Assign the position of the ball to a pybind list. */
    std::vector<double> position_vector_object{pose_ball.X(), pose_ball.Y(), pose_ball.Z()};
    pybind11::list position_object = pybind11::cast(position_vector_object);

    /* Assign the orientation of the ball to a pybind list. */
    std::vector<double> orientation_vector_object{pose_ball.Rot().Euler().X(), pose_ball.Rot().Euler().Y(), pose_ball.Rot().Euler().Z()};
    pybind11::list orientation_object = pybind11::cast(orientation_vector_object);

    /* Add the object to the scene. */
    pybind11::object add_object_ = sensor_digit.attr("add_object")
    (
        "mesh"_a = tacto_cpp_wrapper_path_ + "/mesh/textured_sphere_smooth_meters.obj",
        "object_name"_a = ball_,
        "position"_a = position_object,
        "orientation"_a = orientation_object
    );

    /* Initiliaze the vector and list for the sensor position computed by the main thread. */
    std::vector<double> position_vector_sensor;
    pybind11::list position_sensor_thread;

    /* Initiliaze the vector and list for the sensor position computed by the main thread. */
    std::vector<double> orientation_vector_sensor;
    pybind11::list orientation_sensor_thread;

    /* Initiliaze the force variable for the contact forces computed by the main thread. */
    float force;

    while(true)
    {
        /* Store the values computed by the principal thread inside the critical section. */
        mutex_.lock();

        position_vector_sensor = {pose_sensor_.X(), pose_sensor_.Y(), pose_sensor_.Z()};
        position_sensor_thread = pybind11::cast(position_vector_sensor);

        orientation_vector_sensor = {pose_sensor_.Rot().Euler().X(), pose_sensor_.Rot().Euler().Y(), pose_sensor_.Rot().Euler().Z()};
        orientation_sensor_thread = pybind11::cast(orientation_vector_sensor);

        /* Change the sign of the force since the renderer expects a positive force. */
        force=abs(forces_);

        mutex_.unlock();

        /* Call the renerer. */
        pybind11::array_t<uint8_t> rgb = sensor_digit.attr("render")\
        (
            "object_position"_a = position_object,
            "object_orientation"_a = orientation_object,
            "sensor_position"_a = position_sensor_thread,
            "sensor_orientation"_a = orientation_sensor_thread,
            "force"_a = force
        );
        /* Convert the image. */
        img = cv::Mat(rgb.shape(0), rgb.shape(1), CV_8UC3, (unsigned char*)rgb.data());

        /* Prepare the output and convert from OpenCV to YARP image. */
        yarp::sig::ImageOf<yarp::sig::PixelRgb>& output = port.prepare();
        output = yarp::cv::fromCvMat<yarp::sig::PixelRgb>(img);

        /* Write into the port. */
        port.write();

        std::this_thread::sleep_for(15ms);
    }
}

void gazebo::RendererPlugin::UpdatePosition()
{
    /* Store the contacts. */
    msgs::Contacts contacts;
    contacts = sensor_->Contacts();

    /* Initialize forces variables keeping into account DART behaviour. */
   float force1 = 0;
   float force2 = 0;
   float force3 = 0;
   float force4 = 0;
   float force5 = 0;
   float force6 = 0;

   for (unsigned int i = 0; i < contacts.contact_size(); ++i)
   {
       for (unsigned int j = 0 ; j<contacts.contact(i).position_size(); ++j)
       {
           force1 += (float)contacts.contact(i).wrench(j).body_1_wrench().force().x();
           force2 += (float)contacts.contact(i).wrench(j).body_2_wrench().force().x();
           force3 += (float)contacts.contact(i).wrench(j).body_1_wrench().force().y();
           force4 += (float)contacts.contact(i).wrench(j).body_2_wrench().force().y();
           force5 += (float)contacts.contact(i).wrench(j).body_1_wrench().force().z();
           force6 += (float)contacts.contact(i).wrench(j).body_2_wrench().force().z();
       }
   }

   /* Get the pose of the sensor and of the object. */
   ignition::math::Matrix3<double> sensor_transform (sensor_model_->GetLink()->WorldPose().Rot());
   ignition::math::Matrix3<double> object_transform (ball_model_->GetLink()->WorldPose().Rot());

   /* Move to world coordinate. */
   ignition::math::Vector3<double> Vector_sensor1 (sensor_transform*ignition::math::Vector3<double>(force1, force3, force5));
   ignition::math::Vector3<double> Vector_sensor2 (sensor_transform*ignition::math::Vector3<double>(force2, force4, force6));
   ignition::math::Vector3<double> Vector_object1 (object_transform*ignition::math::Vector3<double>(force1, force3, force5));
   ignition::math::Vector3<double> Vector_object2 (object_transform*ignition::math::Vector3<double>(force2, force4, force6));

   mutex_.lock();

   /**
    * DART might swap the assignment between body 1 / body 2.
    * See https://github.com/dartsim/dart/issues/1425
    **/
    forces_ = 0;
    /* Check if we are in contact. */
    if(contacts.contact_size() != 0)
    {
        if (abs(Vector_sensor2.X()+Vector_object1.X()) < abs(Vector_sensor1.X()+Vector_object2.X()))
        {
            forces_= force2;
        }
        else
        {
            forces_= force1;
        }

    }

    pose_sensor_ = sensor_model_->GetLink()->WorldPose();

    mutex_.unlock();
}
