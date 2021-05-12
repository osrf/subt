#include <memory>

#include <ignition/msgs/double.pb.h>
#include <ignition/msgs/camera_info.pb.h>
#include <ignition/msgs/image.pb.h>
#include <ignition/plugin/Register.hh>

#include <ignition/transport/Node.hh>

#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/World.hh>

#include <ignition/gazebo/Util.hh>

using namespace ignition;
using namespace gazebo;

namespace subt
{

/// \brief Attach this plugin inside a <sensor> tag to make it publish mono images in a sub-namespace 'mono'
/// The conversion to mono is a simple one with perceptually good results (non-uniform linear combination of RGB).
class MonoCameraSystem : public System, public ISystemConfigure, public ISystemPostUpdate
{
  public: void Configure(const Entity& _entity, const std::shared_ptr<const sdf::Element>& _sdf,
                         EntityComponentManager& _ecm, EventManager& _eventMgr) override
  {
    this->sensor = _entity;
  }

  protected: void PostUpdate(const UpdateInfo& _info, const EntityComponentManager& _ecm) override
  {
    if (this->initialized)
      return;

    // wait until the sensor is attached to world

    const World world(worldEntity(this->sensor, _ecm));
    const auto worldName = world.Name(_ecm);

    if (!worldName.has_value())
      return;

    this->initialized = true;

    // sensor is attached to world, now we can get full scoped name

    const auto sensorName = "/" + scopedName(this->sensor, _ecm);

    std::string subTopic { sensorName + "/image" };
    std::string subInfoTopic { sensorName + "/camera_info" };
    std::string pubTopic { sensorName + "/mono/image" };
    std::string pubInfoTopic { sensorName + "/mono/camera_info" };
    this->reqSetRateTopic = subTopic + "/set_rate";
    std::string advSetRateTopic { pubTopic + "/set_rate" };

    this->pub = this->node.Advertise<ignition::msgs::Image>(pubTopic);
    this->pubInfo = this->node.Advertise<ignition::msgs::CameraInfo>(pubInfoTopic);
    this->node.Subscribe(subTopic, &MonoCameraSystem::OnMsg, this);
    this->node.Subscribe(subInfoTopic, &MonoCameraSystem::OnInfoMsg, this);
    this->node.Advertise(advSetRateTopic, &MonoCameraSystem::OnSetRate, this);

    ignmsg << "MonoCameraSystem publishing on [" << pubTopic << "]" << std::endl;
  }

  protected: void OnMsg(const ignition::msgs::Image& msg)
  {
    ignition::msgs::Image outMsg;
    switch (msg.pixel_format_type())
    {
      case ignition::msgs::PixelFormatType::BAYER_BGGR8:
      case ignition::msgs::PixelFormatType::BAYER_GBRG8:
      case ignition::msgs::PixelFormatType::BAYER_GRBG8:
      case ignition::msgs::PixelFormatType::BAYER_RGGB8:
      {
        outMsg.CopyFrom(msg);
        outMsg.set_pixel_format_type(ignition::msgs::PixelFormatType::L_INT8);
        break;
      }
      case ignition::msgs::PixelFormatType::RGB_INT8:
      case ignition::msgs::PixelFormatType::BGR_INT8:
      {
        outMsg.mutable_header()->CopyFrom(msg.header());
        outMsg.set_pixel_format_type(ignition::msgs::PixelFormatType::L_INT8);
        outMsg.set_width(msg.width());
        outMsg.set_height(msg.height());
        outMsg.set_step(msg.width());

        const auto length = msg.width() * msg.height();
        if (this->data == nullptr)
          this->data = new char[length];

        const auto& inData = msg.data();
        for (size_t i = 0; i < length; ++i)
          this->data[i] = static_cast<char>((30 * inData[i*3] + 59 * inData[i*3+1] + 11 * inData[i*3+2]) / 100);

        outMsg.set_data(this->data, length);
        break;
      }
      default:
      {
        static bool warned = false;
        if (!warned)
        {
          ignwarn << "MonoCameraSystem: Unsupported pixel format [" << msg.pixel_format_type() << "]" << std::endl;
          warned = true;
          return;
        }
      }
    }

    this->pub.Publish(outMsg);
  }

  protected: void OnInfoMsg(const ignition::msgs::CameraInfo& msg)
  {
    this->pubInfo.Publish(msg);
  }

  protected: void OnSetRate(const ignition::msgs::Double &_rate)
  {
    this->node.Request(this->reqSetRateTopic, _rate);
  }

  public: ~MonoCameraSystem() override
  {
    delete[] this->data;
    this->data = nullptr;
  }

  protected: transport::Node node;
  protected: transport::Node::Publisher pub, pubInfo;
  protected: Entity sensor {kNullEntity};
  protected: bool initialized {false};
  protected: char* data {nullptr};
  protected: std::string reqSetRateTopic;
};

}

IGNITION_ADD_PLUGIN(subt::MonoCameraSystem, System, ISystemConfigure, ISystemPostUpdate)
IGNITION_ADD_PLUGIN_ALIAS(subt::MonoCameraSystem, "subt::MonoCameraSystem")