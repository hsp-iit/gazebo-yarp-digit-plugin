/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include "RendererDigitPlugin.h"
#include <yarp/os/LogStream.h>

using namespace pybind11::literals;
GZ_REGISTER_MODEL_PLUGIN(gazebo::RendererPlugin)

gazebo::RendererPlugin::RendererPlugin()
{}


gazebo::RendererPlugin::~RendererPlugin()
{}


template<class T>
bool gazebo::RendererPlugin::LoadParameterFromSDF(sdf::ElementPtr sdf, const std::string& name, T& value)
{
    /* Check if the element exists. */
    if (!(sdf->HasElement(name)))
    {
        yError() << "At line " << __LINE__ << ", in function " << __FUNCTION__ << ", parameter " + name + " does not exist. Returning false.";

        return false;
    }

    /* Get the associated parameter. */
    sdf::ParamPtr parameter = sdf->GetElement(name)->GetValue();

    /* Check if the value can be intrepreted as a T. */
    if (!parameter->Get<T>(value))
    {
        yError() << "At line " << __LINE__ << ", in function " << __FUNCTION__ << ", parameter " + name + " is not valid. Returning false.";

        return false;
    }

    return true;
}


void gazebo::RendererPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
    /* Store the pointer of the sensor model. */
    sensor_model_ = model;

    /* Store the pointer of the world model. */
    physics::WorldPtr world_ptr = sensor_model_->GetWorld();

    /* Load the name of the object. */
    if(!LoadParameterFromSDF(sdf, "ObjectName", object_name_))
    {
        yError() << "At line " << __LINE__ << ", in function " << __FUNCTION__ << ", error with ObjectName parameter. Closing the plugin thread.";

        std::terminate();
    }

    object_model_ = world_ptr->ModelByName(object_name_);

    /* Load the absolute path of the object mesh. */
    if(!LoadParameterFromSDF(sdf, "ObjectMeshAbsolutePath", object_mesh_absolute_path_))
    {
        yError() << "At line " << __LINE__ << ", in function " << __FUNCTION__ << ",  error with ObjectMeshAbsolutePath parameter. Closing the plugin thread.";

        std::terminate();
    }

    /* Load the number of sensors. */
    std::string number_of_sensors_string;

    if(!LoadParameterFromSDF(sdf, "NumberOfSensors", number_of_sensors_string))
    {
        yError() << "At line " << __LINE__ << ", in function " << __FUNCTION__ << ",  error with NumberOfSensors parameter. Closing the plugin thread.";

        std::terminate();
    }

    /* Check if the string entered is a number. */
    int number_of_sensors;

    try
    {
        number_of_sensors = stoi(number_of_sensors_string);
    }
    catch (const std::invalid_argument& ia)
    {
        yError() << "At line " << __LINE__ << ", in function " << __FUNCTION__ << ", invalid argument: " << ia.what() << ". Closing the plugin thread.";

        std::terminate();
    }

    /* Laod the UpdatePeriod variable. */
    std::string update_period_string;

    if (!LoadParameterFromSDF(sdf, "UpdatePeriod", update_period_string))
    {
        yError() << "At line " << __LINE__ << ", in function " << __FUNCTION__ << ",  error with UpdatePeriod parameter. Closing the plugin thread.";

        std::terminate();
    }

    try
    {
        update_period_ = stoi(update_period_string);
    }
    catch (const std::invalid_argument& ia)
    {
        yError() << "At line " << __LINE__ << ", in function " << __FUNCTION__ << ", invalid argument: " << ia.what() << ". Closing the plugin thread.";

        std::terminate();
    }

    /* Storage for the sensor names. */
    std::vector<std::string> sensors_names;

    /* Loop over the links. */
    for (size_t i = 0; i < number_of_sensors; i++)
    {
        std::string link_name;
        std::string sensor_name;

        /* Load the links names. */
        if (!LoadParameterFromSDF(sdf, "LinkName"+ std::to_string(i+1), link_name))
        {
            yError() << "At line " << __LINE__ << ", in function " << __FUNCTION__ << ",  error with LinkName parameter. Closing the plugin thread.";

            std::terminate();
        }

        /* Laod the sensors names. */
        if (!LoadParameterFromSDF(sdf, "SensorName"+ std::to_string(i+1), sensor_name))
        {
            yError() << "At line " << __LINE__ << ", in function " << __FUNCTION__ << ",  error with LinkName parameter. Closing the plugin thread.";

            std::terminate();
        }
        sensors_link_name_.push_back(link_name);
        sensors_names.push_back(sensor_name);
    }

    /* Loop over the links that actually contain a sensor. */
    for (size_t i = 0; i < sensors_link_name_.size(); i++)
    {
        /* Istantiate the sensor manager. */
        gazebo::sensors::SensorManager *sensor_manager = gazebo::sensors::SensorManager::Instance();

        /* Check if the queue of the SensorManager is empty. */
        if (!sensor_manager->SensorsInitialized())
        {
            yError() << "At line " << __LINE__ << ", in function " << __FUNCTION__ << ", cannot intiialize the sensor manager. Closing the plugin thread.";

           std::terminate();
        }

        /* Assign the sensor to the pointer. */
        gazebo::sensors::SensorPtr generic_ptr = sensor_manager->GetSensor(sensors_names[i]);
        sensors_[sensors_link_name_[i]] = std::dynamic_pointer_cast<sensors::ContactSensor>(generic_ptr);

        /* Check the sensor. */
        if (!sensors_[sensors_link_name_[i]])
        {
          yError() << "At line " << __LINE__ << ", in function " << __FUNCTION__ << ", RendererDigitPlugin requires a ContactSensor. Closing the plugin thread.";

          std::terminate();
        }

        /* Activate the sensor .*/
        sensors_[sensors_link_name_[i]]->SetActive(true);

        /* Initialize the force to 0. */
        forces_[sensors_link_name_[i]] = 0;
    }

    /* Initialize the rendering thread. */
    std::thread rendering_thread(&gazebo::RendererPlugin::RenderingThread, this);
    rendering_thread.detach();

    /* Update the positon by calling the UpdateStates method. */
    updateConnection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&RendererPlugin::UpdateStates, this));
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
    pybind11::str object = object_name_;

    /* Initialize YARP port. */
    yarp::os::Network yarp;
    std::unordered_map<std::string, yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>>> port;

    /* Initialize the output image. */
    pybind11::array_t<uint8_t> rgb;
    cv::Mat img;

    /* Get the pose of the object. */
    pose_object_ = object_model_->GetLink()->WorldPose();

    /* Assign the position of the object to a pybind list. */
    std::vector<double> position_vector_object{pose_object_.X(), pose_object_.Y(), pose_object_.Z()};
    pybind11::list position_object = pybind11::cast(position_vector_object);

    /* Assign the orientation of the object to a pybind list. */
    std::vector<double> orientation_vector_object{pose_object_.Rot().Euler().X(), pose_object_.Rot().Euler().Y(), pose_object_.Rot().Euler().Z()};
    pybind11::list orientation_object = pybind11::cast(orientation_vector_object);

    /* Open the YARP ports. */
    for (auto elem : sensors_link_name_)
        port[elem].open("/gazebo-yarp-digit-plugin-" + elem + "/output:o");

    /* Instantiate an object of the sensor class. */
    pybind11::object sensor_digit = sensor_module
    (
        "background_path"_a = tacto_path_ + "/examples/conf/bg_digit_240_320.jpg",
        "configuration_path"_a = tacto_path_ + "/tacto/config_digit_shadow.yml"
    );

    /* Add the object to the scene. */
    pybind11::object add_object_ = sensor_digit.attr("add_object")
    (
        "mesh"_a = object_mesh_absolute_path_,
        "object_name"_a = object,
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

        for (auto elem : sensors_link_name_)
        {
            /* Store the values computed by the principal thread inside the critical section. */
            mutex_.lock();

            /* Get the real-time position of the sensor. */
            position_vector_sensor = {pose_sensors_[elem].X(), pose_sensors_[elem].Y(), pose_sensors_[elem].Z()};
            position_sensor_thread = pybind11::cast(position_vector_sensor);

            /* Get the real-time orientation of the sensor. */
            orientation_vector_sensor = { pose_sensors_[elem].Rot().Euler().X(), pose_sensors_[elem].Rot().Euler().Y(), pose_sensors_[elem].Rot().Euler().Z()};
            orientation_sensor_thread = pybind11::cast(orientation_vector_sensor);

            /* Get the real-time position of the object. */
            position_vector_object = {pose_object_.X(), pose_object_.Y(), pose_object_.Z()};
            position_object = pybind11::cast(position_vector_object);

            /* Get the real-time orientation of the object. */
            orientation_vector_object = {pose_object_.Rot().Euler().X(), pose_object_.Rot().Euler().Y(), pose_object_.Rot().Euler().Z()};
            orientation_object = pybind11::cast(orientation_vector_object);

            /* Change the sign of the force since the renderer expects a positive force. */
            force = abs(forces_[elem]);
            mutex_.unlock();

            /* Call the renderer. */
            pybind11::array_t<uint8_t> rgb = sensor_digit.attr("render")
            (
                "object_position"_a = position_object,
                "object_orientation"_a = orientation_object,
                "sensor_position"_a = position_sensor_thread,
                "sensor_orientation"_a = orientation_sensor_thread,
                "force"_a = force

            );

            /* Convert the image. */
            img = cv::Mat(rgb.shape(0), rgb.shape(1), CV_8UC3, (unsigned char*)rgb.data());

            /* Prepare the output and convert fromCVMat. */
            yarp::sig::ImageOf<yarp::sig::PixelRgb>& output = port[elem].prepare();
            output = yarp::cv::fromCvMat<yarp::sig::PixelRgb>(img);

            /* Write into the port. */
            port[elem].write();

        }

        std::this_thread::sleep_for(std::chrono::duration<double>(update_period_));
    }
}


std::unordered_map<std::string, float> gazebo::RendererPlugin::ComputeForces()
{

    std::unordered_map<std::string, float> forces;

    for (auto elem : sensors_link_name_)
    {
        /* Store the contacts. */
        msgs::Contacts contacts;
        contacts = sensors_[elem]->Contacts();

        /* Initialize forces keeping into account DART behaviour. */
        float force_x_body_1 = 0;
        float force_x_body_2 = 0;
        float force_y_body_1 = 0;
        float force_y_body_2 = 0;
        float force_z_body_1 = 0;
        float force_z_body_2 = 0;

        for (unsigned int i = 0; i < contacts.contact_size(); ++i)
        {
            for (unsigned int j = 0 ; j<contacts.contact(i).position_size(); ++j)
            {
                force_x_body_1 += (float)contacts.contact(i).wrench(j).body_1_wrench().force().x();
                force_x_body_2 += (float)contacts.contact(i).wrench(j).body_2_wrench().force().x();
                force_y_body_1 += (float)contacts.contact(i).wrench(j).body_1_wrench().force().y();
                force_y_body_2 += (float)contacts.contact(i).wrench(j).body_2_wrench().force().y();
                force_z_body_1 += (float)contacts.contact(i).wrench(j).body_1_wrench().force().z();
                force_z_body_2 += (float)contacts.contact(i).wrench(j).body_2_wrench().force().z();
            }
        }

        /* Get the pose of the sensor and of the object. */
        ignition::math::Matrix3<double> sensor_transform (sensor_model_->GetLink(elem)->WorldPose().Rot());
        ignition::math::Matrix3<double> object_transform (object_model_->GetLink()->WorldPose().Rot());

        /* Move to world coordinate. */
        ignition::math::Vector3<double> Vector_sensor1 (sensor_transform*ignition::math::Vector3<double>(force_x_body_1, force_y_body_1, force_z_body_1));
        ignition::math::Vector3<double> Vector_sensor2 (sensor_transform*ignition::math::Vector3<double>(force_x_body_2, force_y_body_2, force_z_body_2));
        ignition::math::Vector3<double> Vector_object1 (object_transform*ignition::math::Vector3<double>(force_x_body_1, force_y_body_1, force_z_body_1));
        ignition::math::Vector3<double> Vector_object2 (object_transform*ignition::math::Vector3<double>(force_x_body_2, force_y_body_2, force_z_body_2));


        /**
         * DART might swap the assignment between body 1 / body 2.
         * See https://github.com/dartsim/dart/issues/1425
         **/
        if (contacts.contact_size() != 0)
        {
            /**
             * Check the norm of the sum between the vectors.
             * The smaller value will indicate the right transform.
             */
            if (ignition::math::Vector3<double>(Vector_sensor2.X() + Vector_object1.X(), Vector_sensor2.Y() + Vector_object1.Y(), Vector_sensor2.Z() + Vector_object1.Z()).Length()<
                ignition::math::Vector3<double>(Vector_sensor1.X() + Vector_object2.X(), Vector_sensor1.Y() + Vector_object2.Y(), Vector_sensor1.Z() + Vector_object2.Z()).Length())
            {
                forces[elem] = force_x_body_2;
            }
            else
            {
                forces[elem] = force_x_body_1;
            }
        }
        else
        {
            forces[elem] = 0;
        }
    }

    return forces;
}


void gazebo::RendererPlugin::UpdateStates()
{
    std::unordered_map<std::string, float> forces = ComputeForces();
    std::unordered_map<std::string, ignition::math::Pose3<double>> poses;

    for (auto elem : sensors_link_name_)
        poses[elem] = sensor_model_->GetLink(elem)->WorldPose();

    mutex_.lock();

    forces_ = forces;
    pose_sensors_ = poses;
    pose_object_ = object_model_->GetLink()->WorldPose();

    mutex_.unlock();
}
