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
/// `<pan>`, `<roll>`, `<tilt>`: Joint parameters header
///
///     `<joint>`: Name of the Joint linking to its parent link
///
///     `<limit>`: Limit on the movement of the joint (rad)
///
///     `<p_gain>`: Proprtional parameter for joint PID controller
///
///     `<i_gain>`: Integral parameter for joint PID controller
///
///     `<d_gain>`: Derivative parameter for joint PID controller
///
///     `<i_max>`: The integral upper limit of the PID
///
///     `<i_min>`: The integral lower limit of the PID
///
///     `<cmd_max>`: Output max value of the PID
///
///     `<cmd_min>`: Output min value of the PID
///
/// `<imu>`: imu topic name on the balance point of the gimbal
///
/// `<topic_js>`: Topic to recieve joint state parameter (Only yaw joint is required to be published for controller to work)
///
/// `<topic_cmd>`: Topic to receive commands from the gimbal system
/// This element is optional, and the default value is `/model/{name_of_model}/gimbal/cmd_vel`.
///
/// `<topic_enable>`: Topic to enable and disable the gimbal system
/// This element is optional, and the default value is `/model/{name_of_model}/gimbal/enable`.
///


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

        auto sdfClone = _sdf->Clone();

        sdf::ElementPtr joints[static_cast<int>(link::links)];

        joints[static_cast<int>(link::yaw)] = sdfClone->GetElement("pan");
        joints[static_cast<int>(link::roll)] = sdfClone->GetElement("roll");
        joints[static_cast<int>(link::pitch)] = sdfClone->GetElement("tilt");

        for(int i=0; i<static_cast<int>(link::links); i++)
        {
            this->_jointName[i] = joints[i]->Get<std::string>("joint");
            this->_limit[i] = joints[i]->Get<double>("limit");

            double p = joints[i]->Get<double>("p_gain");
            double in = joints[i]->Get<double>("i_gain");
            double d = joints[i]->Get<double>("d_gain");
            double i_max = joints[i]->Get<double>("i_max");
            double i_min = joints[i]->Get<double>("i_min");
            double cmd_max = joints[i]->Get<double>("cmd_max");
            double cmd_min = joints[i]->Get<double>("cmd_min");
            _pid[i] = ignition::math::PID(p, in, d, i_max, i_min, cmd_max, cmd_min, 0);

            this->_jointPub[i] = this->node.Advertise<ignition::msgs::Double>("/model/" + this->model.Name(_ecm) + "/joint/" + _jointName[i] + "/cmd_vel");
            this->_setPoint[i]=0;
            this->_speed[i]=0;
            this->_currState[i]=0;
        }

        std::string imuTopic = _sdf->Get<std::string>("imu");
        this->node.Subscribe(imuTopic, &IGNGimbalControlPlugin::OnImuCB, this);

        std::string topicJS = _sdf->Get<std::string>("topic_js");
        this->node.Subscribe(topicJS, &IGNGimbalControlPlugin::OnJointCB, this);

        std::string topicVelCmd {"/model/" + this->model.Name(_ecm) + "/gimbal/cmd_vel"};
        if (_sdf->HasElement("topic_cmd"))
            topicVelCmd = _sdf->Get<std::string>("topic_cmd");
        this->node.Subscribe(topicVelCmd, &IGNGimbalControlPlugin::OnCmdCB, this);
        
        std::string topicEnable {"/model/" + this->model.Name(_ecm) + "/gimbal/enable"};
        if (_sdf->HasElement("topic_enable"))
            topicEnable = _sdf->Get<std::string>("topic_enable");
        this->node.Subscribe(topicEnable, &IGNGimbalControlPlugin::EnableCB, this);

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

        if(_armed){
            
            this->_currState[static_cast<int>(link::roll)] = _rpy.Roll();
            this->_currState[static_cast<int>(link::pitch)] = _rpy.Pitch();

            double dt = std::chrono::duration<double>(_info.dt).count();

            double force;
            double error;

            for(int i = 0; i < static_cast<int>(link::links); i++)
            {

                this->_setPoint[i] += dt*_speed[i];
                
                if(_limit[i]>0.0001 && abs(this->_setPoint[i])>this->_limit[i])
                    this->_setPoint[i] = _limit[i]*((this->_setPoint[i]<0)?-1:1);

                error = this->_currState[i] + this->_setPoint[i];
                force = _pid[i].Update(error,_info.dt);

                ignition::msgs::Double _msg;
                _msg.set_data(force);
                this->_jointPub[i].Publish(_msg);
            }
        }
    }

    public: void EnableCB(const msgs::Boolean &_msg)
    {
        this->_armed=_msg.data();
    }

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

    public: void OnJointCB(const msgs::Model &_msg)
    {
        for(auto joint : _msg.joint())
        {
            if(joint.name().compare(_jointName[static_cast<int>(link::yaw)])==0){
                this->_currState[static_cast<int>(link::yaw)] = joint.axis1().position();
                return;
            }
        }
    }
    
    protected: enum class link{roll,pitch,yaw,links};
    
    protected: ignition::gazebo::Model model{kNullEntity};
    protected: std::string _jointName[static_cast<int>(link::links)];

    protected: ignition::math::Quaternion<double> _rpy;
    protected: ignition::transport::Node node;
    protected: ignition::transport::Node::Publisher _jointPub[static_cast<int>(link::links)];

    protected: bool _armed=false;
    protected: double _currState[static_cast<int>(link::links)];
    protected: double _setPoint[static_cast<int>(link::links)];
    protected: double _speed[static_cast<int>(link::links)];
    protected: double _limit[static_cast<int>(link::links)];
    protected: ignition::math::PID _pid[static_cast<int>(link::links)];
};

}

IGNITION_ADD_PLUGIN(hovermap::IGNGimbalControlPlugin,
                    ignition::gazebo::System,
                    hovermap::IGNGimbalControlPlugin::ISystemConfigure,
                    hovermap::IGNGimbalControlPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(hovermap::IGNGimbalControlPlugin, "emesent::hovermap::GimbalControlPlugin")