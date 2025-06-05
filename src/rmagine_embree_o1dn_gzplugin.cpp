#include <rmagine_gazebo_plugins/rmagine_embree_o1dn_gzplugin.h>

#include <gazebo/sensors/SensorFactory.hh>

#include <iostream>
#include <fstream>
#include <sstream>
#include <boost/algorithm/string/replace.hpp>

#include <rmagine/noise/GaussianNoise.hpp>
#include <rmagine/noise/UniformDustNoise.hpp>
#include <rmagine/noise/RelGaussianNoise.hpp>

#include <gazebo/common/Console.hh>


using namespace std::placeholders;

namespace gazebo
{

namespace sensors
{


static rm::Transform to_rm(const ignition::math::Pose3d& pose)
{
    rmagine::Transform T;
    T.R.x = pose.Rot().X();
    T.R.y = pose.Rot().Y();
    T.R.z = pose.Rot().Z();
    T.R.w = pose.Rot().W();
    T.t.x = pose.Pos().X();
    T.t.y = pose.Pos().Y();
    T.t.z = pose.Pos().Z();
    return T;
}

static rm::O1DnModel fetch_sensor_model(sdf::ElementPtr rayElem)
{
    gzdbg << "[RmagineEmbreeO1Dn] fetching parameters from sdf" << std::endl;

    sdf::ElementPtr scanElem = rayElem->GetElement("scan");
    rm::O1DnModel sensor_model;

    if(scanElem->HasElement("orig"))
    {
        sdf::ElementPtr origElem = scanElem->GetElement("orig");
        sensor_model.orig.x = origElem->Get<float>("x");
        sensor_model.orig.y = origElem->Get<float>("y");
        sensor_model.orig.z = origElem->Get<int>("z");
    } else {
        sensor_model.orig.x = 0.0;
        sensor_model.orig.y = 0.0;
        sensor_model.orig.z = 0.0;
    }

    sdf::ElementPtr rangeElem = rayElem->GetElement("range");
    sensor_model.range.min = rangeElem->Get<float>("min");
    sensor_model.range.max = rangeElem->Get<float>("max");

    return sensor_model;
}



RmagineEmbreeO1Dn::RmagineEmbreeO1Dn()
:Base(sensors::RAY) // if sensor is base class: Base(sensors::RAY)
{
    gzdbg << "[RmagineEmbreeO1Dn] Constructed." << std::endl;
}

RmagineEmbreeO1Dn::~RmagineEmbreeO1Dn()
{
    gzdbg << "[RmagineEmbreeO1Dn] Destroyed." << std::endl;
}

void RmagineEmbreeO1Dn::Load(const std::string& world_name)
{
    Base::Load(world_name);
    
    GZ_ASSERT(this->world != nullptr,
      "RaySensor did not get a valid World pointer");

    const char* config_file_tag = "config_file";
    if(!this->sdf->HasElement(config_file_tag))
    {
        gzerr << "[RmagineEmbreeO1Dn] Must contain tag <" 
            << config_file_tag << ">. Exiting setup." << std::endl;
        // Exit setup
        return;
    }
    else
    {
        m_config_filename = this->sdf->GetElement(config_file_tag)->Get<std::string>();
    }

    sdf::ElementPtr rayElem = this->sdf->GetElement("ray");

    // GET SENSOR MODEL
    if(rayElem->HasElement("scan"))
    {
        m_sensor_model = fetch_sensor_model(rayElem);

        // Parse scan pattern config file to vector
        std::vector<std::pair<float, float>> coords;
        ParseCsvToVector(coords);

        // TODO: have this shutdown the sensor/plugin as well
        if (coords.size() == 0)
        {
            gzerr << "[RmagineEmbreeO1Dn] Did not find any valid scan points "
            "in provided config file. Check that config file exists and is well-formed." << std::endl;
        }

        m_sensor_model.width = coords.size();
        m_sensor_model.height = 1;
        m_sensor_model.dirs.resize(m_sensor_model.width);

        // Set the coordinates in the sensor_model
        for (int i = 0; i < coords.size(); i++)
        {
            // Convert the spherical angles from config file
            // to cartesian coordinates
            float h = coords[i].first;
            float v = coords[i].second;
            m_sensor_model.dirs[i].x = cos(v) * cos(h);
            m_sensor_model.dirs[i].y = cos(v) * sin(h);
            m_sensor_model.dirs[i].z = sin(v);
            m_sensor_model.dirs[i].normalize();
        }

        sim_buffers.ranges.resize(m_sensor_model.size());
    }

    // GET NOISE MODEL
    if(rayElem->HasElement("noise"))
    {
        // has noise
        rm::Noise::Options opt = {};
        opt.max_range = m_sensor_model.range.max;

        sdf::ElementPtr noiseElem = rayElem->GetElement("noise");

        while(noiseElem)
        {
            std::string noise_type = noiseElem->Get<std::string>("type");
            if(noise_type == "gaussian")
            {
                gzdbg << "[RmagineEmbreeO1Dn] init noise: 'gaussian'" << std::endl;

                float mean = 0.0;
                if(noiseElem->HasElement("mean"))
                {
                    mean = noiseElem->Get<float>("mean");
                }

                float stddev = noiseElem->Get<float>("stddev");
                rm::NoisePtr gaussian_noise = std::make_shared<rm::GaussianNoise>(
                    mean,
                    stddev,
                    opt
                );

                m_noise_models.push_back(gaussian_noise);
            } else if(noise_type == "uniform_dust") {
                gzdbg << "[RmagineEmbreeO1Dn] init noise: 'uniform_dust'" << std::endl;

                float hit_prob = noiseElem->Get<float>("hit_prob");
                float return_prob = noiseElem->Get<float>("return_prob");

                rm::NoisePtr uniform_dust_noise = std::make_shared<rm::UniformDustNoise>(
                    hit_prob,
                    return_prob,
                    opt
                );

                m_noise_models.push_back(uniform_dust_noise);

            } else if(noise_type == "rel_gaussian") {

                gzdbg << "[RmagineEmbreeO1Dn] init noise: 'rel_gaussian'" << std::endl;

                float mean = 0.0;
                float range_exp = 1.0;
                float stddev = noiseElem->Get<float>("stddev");

                if(noiseElem->HasElement("mean"))
                {
                    mean = noiseElem->Get<float>("mean");
                }

                if(noiseElem->HasElement("range_exp"))
                {
                    range_exp = noiseElem->Get<float>("range_exp");
                }

                rm::NoisePtr gaussian_noise = std::make_shared<rm::RelGaussianNoise>(
                    mean,
                    stddev,
                    range_exp,
                    opt
                );

                m_noise_models.push_back(gaussian_noise);
            } else {
                gzwarn << "[RmagineEmbreeO1Dn] WARNING: SDF noise type '" << noise_type << "' unknown. skipping." << std::endl;
            }

            noiseElem = noiseElem->GetNextElement("noise");
        }
    }

    // COMPUTE
    if(rayElem->HasElement("compute"))
    {
        sdf::ElementPtr computeElem = rayElem->GetElement("compute");
        
        if(computeElem->HasElement("normals"))
        {
            if(computeElem->Get<bool>("normals"))
            {
                sim_buffers.normals.resize(m_sensor_model.size());
            }
        }

        if(computeElem->HasElement("object_ids"))
        {
            if(computeElem->Get<bool>("object_ids"))
            {
                sim_buffers.object_ids.resize(m_sensor_model.size());
            }
        }

        if(computeElem->HasElement("face_ids"))
        {
            if(computeElem->Get<bool>("face_ids"))
            {
                sim_buffers.face_ids.resize(m_sensor_model.size());
            }
        }
    }

    std::cout << "Get Parent Entity: " << this->ParentName() << std::endl;
    this->parentEntity = this->world->EntityByName(this->ParentName());

    GZ_ASSERT(this->parentEntity != nullptr,
      "Unable to get the parent entity.");

    auto pose = Pose();
    m_Tsb = to_rm(pose);

    // std::cout << "[RmagineEmbreeO1Dn] advertising topic " << this->Topic() << std::endl;
    
    if(m_gz_publish)
    {
        this->scanPub =
        this->node->Advertise<msgs::LaserScanStamped>(this->Topic(), 50);

        if (!this->scanPub || !this->scanPub->HasConnections())
        {
            gzwarn << "[RmagineEmbreeO1Dn] Gazebo internal publishing failed. Reason: ";

            if(!this->scanPub)
            {
                gzwarn << "- Reason: No scanPub" << std::endl;
            } else {
                gzwarn << "- Reason: No connections" << std::endl;
            }
        }
    }
}

void RmagineEmbreeO1Dn::Init()
{
    Base::Init();
    this->laserMsg.mutable_scan()->set_frame(this->ParentName());
}

std::string RmagineEmbreeO1Dn::Topic() const
{
    std::string topicName = "~/";
    topicName += this->ParentName() + "/" + this->Name() + "/scan";
    boost::replace_all(topicName, "::", "/");

    return topicName;
}

bool RmagineEmbreeO1Dn::IsActive() const
{
    return Sensor::IsActive() ||
        (this->scanPub && this->scanPub->HasConnections());
}

void RmagineEmbreeO1Dn::setMap(rm::EmbreeMapPtr map)
{
    std::cout << "!!!! SET MAP" << std::endl;
    
    m_map = map;
    
    if(m_o1dn_sim)
    {
        m_o1dn_sim->setMap(map);
    } else {
        m_o1dn_sim = std::make_shared<rm::O1DnSimulatorEmbree>(map);
        m_o1dn_sim->setTsb(m_Tsb);
        m_o1dn_sim->setModel(m_sensor_model);
    }

    if(m_waiting_for_map)
    {
        std::cout << "[RmagineEmbreeO1Dn] RmagineEmbreeMap found." << std::endl;
    }

    m_waiting_for_map = false;
}

void RmagineEmbreeO1Dn::setLock(std::shared_ptr<std::shared_mutex> mutex)
{
    m_map_mutex = mutex;
}

bool RmagineEmbreeO1Dn::UpdateImpl(const bool _force)
{
    if(m_o1dn_sim)
    {
        IGN_PROFILE("RmagineEmbreeO1Dn::UpdateImpl");
        IGN_PROFILE_BEGIN("Update");

        auto pose = parentEntity->WorldPose();
        rm::Memory<rm::Transform> Tbms(1);
        Tbms[0] = to_rm(pose);
        
        
        if(m_map_mutex)
        {
            m_map_mutex->lock_shared();
        }

        m_o1dn_sim->simulate(Tbms, sim_buffers);
        if(m_map_mutex)
        {
            m_map_mutex->unlock_shared();
        }

        // apply noise
        for(auto noise_model : m_noise_models)
        {
            noise_model->apply(sim_buffers.ranges);
        }

        this->lastMeasurementTime = this->world->SimTime();

        IGN_PROFILE_END();

        // std::cout << "[RmagineEmbreeO1Dn] Simulated " << m_ranges.size() << " ranges" << std::endl;
        
        if(m_gz_publish)
        {
            IGN_PROFILE_BEGIN("Publish");
            if (this->scanPub && this->scanPub->HasConnections())
            {
                this->scanPub->Publish(this->laserMsg);
            } else {
                gzwarn << "[RmagineEmbreeO1Dn] Publishing failed. " << std::endl;
            }
            IGN_PROFILE_END();
        }

        return true;
    } else {
        if(!m_waiting_for_map)
        {
            gzdbg << "[RmagineEmbreeO1Dn] waiting for RmagineEmbreeMap..." << std::endl;
            m_waiting_for_map = true;
        }
    }
    
    return false;
}

void RmagineEmbreeO1Dn::Fini()
{
    Base::Fini();
    this->scanPub.reset();
}

void RmagineEmbreeO1Dn::ParseCsvToVector(std::vector<std::pair<float, float>>& coords)
{
    std::ifstream file(m_config_filename);
    std::string line;
    while (std::getline(file, line))
    {
        // Read each line as a stream to easily separate and store variables
        std::istringstream ss(line);
        float h, v;
        char comma;
        ss >> h >> comma >> v;

        coords.push_back(std::pair<float, float>(h, v));
    }

    return;
}

GZ_REGISTER_STATIC_SENSOR("rmagine_embree_o1dn", RmagineEmbreeO1Dn)

} // namespace sensors

} // namespace gazebo