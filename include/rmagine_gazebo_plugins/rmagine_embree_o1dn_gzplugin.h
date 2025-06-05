#ifndef GAZEBO_RMAGINE_EMBREE_O1DN_PLUGIN_H
#define GAZEBO_RMAGINE_EMBREE_O1DN_PLUGIN_H

#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/RaySensor.hh>

#include <rmagine/math/types.h>
#include <rmagine/types/Memory.hpp>
#include <rmagine/types/sensor_models.h>
#include <rmagine/simulation/O1DnSimulatorEmbree.hpp>
#include <rmagine/noise/Noise.hpp>

#include <mutex>
#include <shared_mutex>
#include <memory>
#include <utility>
#include <vector>


namespace rm = rmagine;

namespace gazebo
{

namespace sensors
{
class RmagineEmbreeO1Dn : public Sensor
{
public:
    using Base = Sensor;

    RmagineEmbreeO1Dn();

    virtual ~RmagineEmbreeO1Dn();

    virtual void Load(const std::string& world_name) override;

    virtual void Init() override;
    
    virtual std::string Topic() const override;

    virtual bool IsActive() const override;

    void setMap(rm::EmbreeMapPtr map);

    void setLock(std::shared_ptr<std::shared_mutex> mutex);

    //void updateScanMsg(rm::MemoryView<float> ranges);

    //void ParseCsvToDirs(rm::O1DnModel& sensor_model);
    void ParseCsvToVector(std::vector<std::pair<float, float>>& coords);


    inline common::Time stamp() const 
    {
        return lastMeasurementTime;
    }

    inline rm::O1DnModel sensorModel() const
    {
        return m_sensor_model;
    }

    // inline rm::MemoryView<float, rm::RAM> ranges() const
    // {
    //     return m_ranges;
    // }

    rm::IntAttrAny<rm::RAM> sim_buffers;

protected:
    virtual bool UpdateImpl(const bool _force) override;

    virtual void Fini() override;

    bool m_needs_update = false;

    rm::O1DnModel m_sensor_model;
    rm::Transform m_Tsb;


    std::shared_ptr<std::shared_mutex> m_map_mutex;
    rm::EmbreeMapPtr m_map;
    rm::O1DnSimulatorEmbreePtr m_o1dn_sim;

    bool m_gz_publish = false;

    std::vector<rm::NoisePtr> m_noise_models;

    // Filepath to scan pattern csv config file
    // Contents are rows of comma-separated horizontal and vertical
    // scanning angles (spherical coordinates).
    std::string m_config_filename;


    /// \brief Parent entity pointer
    physics::EntityPtr parentEntity;

    /// \brief Publisher for the scans
    transport::PublisherPtr scanPub;

    /// \brief Laser message.
    msgs::LaserScanStamped laserMsg;

    /// \brief Mutex to protect laserMsg
    std::mutex mutex;

    bool m_waiting_for_map = false;

    
};

using RmagineEmbreeO1DnPtr = std::shared_ptr<RmagineEmbreeO1Dn>;

// will by generated in cpp
void RegisterRmagineEmbreeO1Dn();

} // namespace sensors

} // namespace gazebo

#endif // GAZEBO_RMAGINE_EMBREE_O1DN_PLUGIN_H