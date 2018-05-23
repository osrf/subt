#include "FlashLightPluginBase.hh"

namespace gazebo
{
  /// \brief An example usage of FlashLightPluginBase class
  // This sample plugin just waits for 10 seconds,
  // then it turns on all lights.
  class SampleFlashLightPlugin : public FlashLightPluginBase
  {
    private: common::Time start_time;
    private: int count;
    private: physics::WorldPtr world;

    /// \brief Called when the plugin is loaded.
    //  It sets the time to wait.
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // === must call this ===
      FlashLightPluginBase::Load(_parent, _sdf);

      // === User's code goes here ===
      gzmsg << "Plugin Loaded: SampleFlashLightPlugin\n" << std::endl;
      this->world = _parent->GetWorld();
      start_time = this->world->SimTime();
      this->count = 10;
      gzmsg << this->count << " sec to turn on..." << std::endl;
      this->TurnOffAll();
    }

    /// \brief Called when the world is updated
    //  It counts down as it checks the simulation time.
    //  When the count gets zero, it turns on the lights.
    public: void OnUpdate()
    {
      // === must call this ===
      FlashLightPluginBase::OnUpdate();

      // === User's code goes here ===
      common::Time current_time = this->world->SimTime();
      if (current_time.Double() - this->start_time.Double() > 1.0)
      {
        this->count--;
        this->start_time = current_time;
        if (this->count > 0)
        {
          gzmsg << this->count << " sec to turn on..." << std::endl;
        }
        else if (this->count == 0)
        {
          this->TurnOnAll();
          gzmsg << "Lights are on now." << std::endl;
        }
      }
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(SampleFlashLightPlugin)
}
