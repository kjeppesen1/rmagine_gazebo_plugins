#include <gazebo/gazebo.hh>
#include <rmagine_gazebo_plugins/rmagine_embree_o1dn_gzplugin.h>


namespace gazebo
{
  class RegisterRmagineEmbreeO1DnPlugin : public SystemPlugin
  {
    /////////////////////////////////////////////
    /// \brief Destructor
    public: virtual ~RegisterRmagineEmbreeO1DnPlugin()
    {
    }

    /////////////////////////////////////////////
    /// \brief Called after the plugin has been constructed.
    public: void Load(int /*_argc*/, char ** /*_argv*/)
    {
      sensors::RegisterRmagineEmbreeO1Dn();
    }

    /////////////////////////////////////////////
    // \brief Called once after Load
    private: void Init()
    {

    }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_SYSTEM_PLUGIN(RegisterRmagineEmbreeO1DnPlugin)
}