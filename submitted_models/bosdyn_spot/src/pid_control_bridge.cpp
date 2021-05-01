#include <memory>

#include <ignition/msgs/pid.pb.h>
#include <ignition/transport/Node.hh>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <dynamic_reconfigure/server.h>
#include <control_toolbox/ParametersConfig.h>

#include <pluginlib/class_list_macros.h>


namespace subt
{

/// \brief ROS-Ign bridge for dynamic reconfiguration of a PID.
class PidControlBridge : public nodelet::Nodelet
{

  protected: void onInit() override
  {
    if (!this->getPrivateNodeHandle().getParam("ign_topic", this->ignTopic))
    {
      ROS_ERROR("Please, provide parameter ~ign_topic");
      exit(1);
    }

    ROS_INFO("Connected to Ignition topic %s", this->ignTopic.c_str());

    ROS_DEBUG_STREAM_NAMED("pid","Initializing dynamic reconfigure in namespace "
      << this->getPrivateNodeHandle().getNamespace());

    this->reconfServer = std::make_unique<dynamic_reconfigure::Server<control_toolbox::ParametersConfig>>(
      this->getPrivateNodeHandle());
    this->reconfServer->setCallback(boost::bind(&PidControlBridge::dynamicReconfigCallback, this, _1, _2));
  }

  protected: void dynamicReconfigCallback(control_toolbox::ParametersConfig &config, uint32_t /*level*/)
  {
    const auto now = ros::Time::now();
    ignition::msgs::PID msg;
    msg.mutable_header()->mutable_stamp()->set_sec(now.sec);
    msg.mutable_header()->mutable_stamp()->set_nsec(now.nsec);

    msg.mutable_p_gain_optional()->set_data(config.p);
    msg.mutable_i_gain_optional()->set_data(config.i);
    msg.mutable_d_gain_optional()->set_data(config.d);
    msg.mutable_i_max_optional()->set_data(config.i_clamp_max);
    msg.mutable_i_min_optional()->set_data(config.i_clamp_min);

    this->node.Request(this->ignTopic, msg);
  }

  protected: ignition::transport::Node node;
  protected: std::string ignTopic;
  protected: std::unique_ptr<dynamic_reconfigure::Server<control_toolbox::ParametersConfig>> reconfServer;
};

}

PLUGINLIB_EXPORT_CLASS(subt::PidControlBridge, nodelet::Nodelet)