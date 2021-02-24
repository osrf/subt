/*
 * ign_gimbal.cpp
 * 
 * Created on:11Feb, 2021
 *      Author: Rowan Ramamurthy
 * 
*/

#include <memory>
#include <iostream>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/Link.hh>

#include <ignition/common/Util.hh>

#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

using namespace ignition;
using namespace gazebo;
using namespace systems;

namespace hovermap
{
/// \brief Plugin for stabalisation of 3-link gimbal used on the emesent_hovermap system
/// This plugin listens to cmd_vel msgs coming from ros to rotate the camera at a particular 
/// rate about the roll and tilt axis while maintaining stabilisation
///
/// # Parameters
///
/// `<pan>`: Name of the link linking the pan link to its parent link
///
/// `<roll>`: Name of the link linking the roll link to its parent link
///
/// `<tilt>`: Name of the link linking the tilt link to its parent link
///
/// `<pan_limit>`: Name of the link linking the pan link to its parent link
///
/// `<roll_limit>`: Name of the link linking the roll link to its parent link
///
/// `<tilt_limit>`: Name of the link linking the tilt link to its parent link
///
/// `<imu>`: imu topic name on the balance point of the gimbal
///
/// `<topic_cmd>`: Topic to receive commands from the gimbal system
/// This element is optional, and the default value is `/model/{name_of_model}/gimbal/cmd_vel`.
///
/// `<topic_enable>`: Topic to enable and disable the gimbal system
/// This element is optional, and the default value is `/model/{name_of_model}/velocity_controller/enable`.

class IGNGimbalControlPlugin: 
                public System, 
                public ISystemConfigure, 
                public ISystemPreUpdate
{

    public: IGNGimbalControlPlugin():System(){};

    public: ~IGNGimbalControlPlugin() override {};

    public: void Configure(const Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_sdf,
                EntityComponentManager &_ecm,
                EventManager& _eventMgr) override
    {
        this->model = Model(_entity);
        if(!this->model.Valid(_ecm))
        {
            ignerr << "Gimbal Control plugin should be attached to the entity. Failed to initialize." << std::endl;
            return;
        }

        if(!_sdf->HasElement("pan") && !_sdf->HasElement("roll") && !_sdf->HasElement("tilt") && !_sdf->HasElement("imu"))
        {
            ignerr << "GimbalControlPlugin has missing or incorrect parameters <PAN> <ROLL> <TILT> <imu>" <<std::endl;
            return;
        }
        this->_jointName[static_cast<int>(link::yaw)] = _sdf->Get<std::string>("pan");
        this->_jointName[static_cast<int>(link::roll)] = _sdf->Get<std::string>("roll");
        this->_jointName[static_cast<int>(link::pitch)] = _sdf->Get<std::string>("tilt");

        if (_sdf->HasElement("pan_limit"))
            this->_limit[static_cast<int>(link::yaw)] = _sdf->Get<double>("pan_limit");

        if (_sdf->HasElement("roll_limit"))
            this->_limit[static_cast<int>(link::roll)] = _sdf->Get<double>("roll_limit");
        
        if (_sdf->HasElement("tilt_limit"))
            this->_limit[static_cast<int>(link::pitch)] = _sdf->Get<double>("tilt_limit");

        std::string imuTopic = _sdf->Get<std::string>("imu");
        this->node.Subscribe(imuTopic, &IGNGimbalControlPlugin::OnImuCB, this);

        std::string topicVelCmd {"/model/" + this->model.Name(_ecm) + "/gimbal/cmd_vel"};
        if (_sdf->HasElement("topic_cmd"))
            topicVelCmd = _sdf->Get<std::string>("topic_cmd");
        this->node.Subscribe(topicVelCmd, &IGNGimbalControlPlugin::OnCmdCB, this);
        
        // std::string topicEnable {"/model/" + this->model.Name(_ecm) + "/velocity_controller/enable"};
        // if (_sdf->HasElement("topic_enable"))
        //     topicEnable = _sdf->Get<std::string>("topic_enable");
        // this->node.Subscribe(topicEnable, &IGNGimbalControlPlugin::EnableCB, this);

        for(int i=0; i<static_cast<int>(link::links); i++){
            this->_jointPub[i] = this->node.Advertise<ignition::msgs::Double>("/model/" + this->model.Name(_ecm) + "/joint/" + _jointName[i] + "/0/cmd_pos");
            this->_setPoint[i]=0;
            this->_speed[i]=0;
            this->_currState[i]=0;
        }

        ignmsg  << "IGNGimbalControl subscribing to cmd_vel messages on [" << topicVelCmd << "]" << std::endl;

    }

    public: void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override
    {
        if (_info.dt < std::chrono::steady_clock::duration::zero())
        {
            ignwarn << "Detected jump back in time ["
                    << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
                    << "s]. GimbalControlPlugin may not work as expected." << std::endl;
            return;
        }

        //if(_active){
        this->_currState[static_cast<int>(link::roll)] = -_rpy.Roll();
        this->_currState[static_cast<int>(link::pitch)] = -_rpy.Pitch();

        double dt = std::chrono::duration<double>(_info.dt).count();

        for(int i = 0; i < static_cast<int>(link::links); i++)
        {
            this->_setPoint[i] += dt*_speed[i];
            
            if(_limit[i]>0.0001 && abs(this->_setPoint[i])>this->_limit[i])
                this->_setPoint[i] = _limit[i]*((this->_setPoint[i]<0)?-1:1);

            ignition::msgs::Double _msg;
            _msg.set_data(this->_setPoint[i]+_currState[i]);
            this->_jointPub[i].Publish(_msg);
        }
        //}
    }

    // public: void EnableCB(const msgs::Boolean &_msg)
    // {
    //     this->_active=_msg.data();
    // }

    public: void OnImuCB(const msgs::IMU &_msg)
    {
        ignition::msgs::Quaternion wxyz = _msg.orientation();
        this->_rpy.Set(wxyz.w(),wxyz.x(),wxyz.y(),wxyz.z());
    }

    public: void OnCmdCB(const msgs::Twist &_msg)
    {
        this->_speed[static_cast<int>(link::roll)] = _msg.angular().x();
        this->_speed[static_cast<int>(link::pitch)] = _msg.angular().y();
        this->_speed[static_cast<int>(link::yaw)] = _msg.angular().z();
    }
    
    protected: enum class link{roll,pitch,yaw,links};
    
    protected: ignition::gazebo::Model model{kNullEntity};
    protected: std::string _jointName[static_cast<int>(link::links)];

    protected: ignition::math::Quaternion<double> _rpy;
    protected: ignition::transport::Node node;
    protected: ignition::transport::Node::Publisher _jointPub[static_cast<int>(link::links)];

    // protected: bool _active=false;
    protected: double _currState[static_cast<int>(link::links)];
    protected: double _setPoint[static_cast<int>(link::links)];
    protected: double _speed[static_cast<int>(link::links)];
    protected: double _limit[static_cast<int>(link::links)];
    
};

}

IGNITION_ADD_PLUGIN(hovermap::IGNGimbalControlPlugin,
                    ignition::gazebo::System,
                    hovermap::IGNGimbalControlPlugin::ISystemConfigure,
                    hovermap::IGNGimbalControlPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(hovermap::IGNGimbalControlPlugin, "emesent::hovermap::GimbalControlPlugin")