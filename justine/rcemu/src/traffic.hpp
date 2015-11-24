#ifndef ROBOCAR_TRAFFIC_HPP
#define ROBOCAR_TRAFFIC_HPP

/**
* @brief Justine - this is a rapid prototype for development of Robocar City Emulator
*
* @file smartcity.hpp
* @author  Norbert Bátfai <nbatfai@gmail.com>
* @version 0.0.10
*
* @section LICENSE
*
* Copyright(C) 2014 Norbert Bátfai, batfai.norbert@inf.unideb.hu
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
*(at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
* @section DESCRIPTION
* Robocar City Emulator and Robocar World Championship
*
* desc
*
*/

#include <thread>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <limits>
#include <memory>
#include <fstream>
#include <cstdlib>
#include <iterator>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>
#include <boost/asio.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/map.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/containers/string.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "carlexer.hpp"
#include "smartcity.hpp"
#include "car.hpp"
#include "robocar.pb.h"

namespace justine
{

namespace robocar
{

constexpr int kMaxBufferLen = 524288;

enum ClientCommand
{
  DISP      = 0,
  AUTH      = 1,
  INIT      = 2,
  CAR       = 3,
  GANGSTERS = 4,
  ROUTE     = 5,
  POS       = 6,
  STAT      = 7,
  STOP      = 8
};

enum class TrafficType: unsigned int
{
  NORMAL = 0,
  ANT,
  ANT_RND,
  ANT_RERND,
  ANT_MRERND
};

class Traffic
{
public:
  Traffic(std::string config):
    configfile_name_(config)
  {
    ProcessConfigFile();

    #ifdef DEBUG
      std::cout << "Attaching shared memory segment called "
              << shm_segment_name
              << "... " << std::endl;
    #endif

    std::cout << connnectivitySettings.shmName.c_str() << std::endl;

    shm_segment_ = new boost::interprocess::managed_shared_memory(
      boost::interprocess::open_only, connnectivitySettings.shmName.c_str());

    shm_map_ =
      shm_segment_->find<SharedMap>("JustineMap").first;

    running_time_elapsed_ = 0;

    // infinite mode
    if (simulationSettings.minutes == -1)
    {
      running_time_allowed_ = (std::numeric_limits<int>::max() / 60 / 1000) - 1;
    }
    else
    {
      running_time_allowed_ = simulationSettings.minutes * 60 * 1000;
    }

    if (entitySettings.pedestriansEnabled)
    {
      InitializePedestrians();
    }

    if (entitySettings.routineCarsEnabled)
    {
      InitializeRoutineCars();
    }

    OpenLogStream();

    is_running_ = true;

    m_cv.notify_one();

    std::cout << "The traffic server is ready." << std::endl;
  }

  ~Traffic()
  {
    is_running_ = false;

    m_thread.join();
    shm_segment_->destroy<SharedMap>("JustineMap");

    delete shm_segment_;
  }

  void ProcessConfigFile(void)
  {
    namespace pt = boost::property_tree;

    pt::ptree prop_tree;

    pt::read_json(configfile_name_, prop_tree);

    /* Parse connectivity */
    connnectivitySettings.shmName =
      prop_tree.get("connectivity.shm", "JustineSharedMemory");
    connnectivitySettings.portNumber =
      prop_tree.get("connectivity.port", 10007);

    /* Parse entity */
    entitySettings.routineCarsEnabled =
      prop_tree.get("entity.routineCars.enabled", true);
    entitySettings.pedestriansEnabled =
      prop_tree.get("entity.pedestrians.enabled", true);
    entitySettings.busesEnabled =
      prop_tree.get("entity.buses.enabled", false);

    entitySettings.routineCarCount =
      prop_tree.get("entity.routineCars.count", 100);
    entitySettings.pedestrianCount =
      prop_tree.get("entity.pedestrians.count", 100);
    entitySettings.gangsterCount =
      prop_tree.get("entity.gangsterCars.count", 0);

    /* Parse simulation */
    simulationSettings.delay =
      prop_tree.get("simulation.delay", 200);
    simulationSettings.minutes =
      prop_tree.get("simulation.minutes", 10.0);
    simulationSettings.catchDistance =
      prop_tree.get("simulation.catchDistance", 15.5);

    std::string tmpTrafficType =
      prop_tree.get("simulation.trafficType", "NORMAL");

    if (tmpTrafficType == "ANTS_RND")
      simulationSettings.trafficType = justine::robocar::TrafficType::ANT_RND;
    else if(tmpTrafficType == "ANTS_RERND")
      simulationSettings.trafficType = justine::robocar::TrafficType::ANT_RERND;
    else if(tmpTrafficType == "ANTS_MRERND")
      simulationSettings.trafficType = justine::robocar::TrafficType::ANT_MRERND;
    else if(tmpTrafficType == "ANTS")
      simulationSettings.trafficType = justine::robocar::TrafficType::ANT;
    else
      simulationSettings.trafficType = justine::robocar::TrafficType::NORMAL;

    simulationSettings.fullLog =
      prop_tree.get("simulation.fullLog", false);

    /* Parse debug */
    debugSettings.verboseMode =
      prop_tree.get("debug.verboseMode", false);
  }

  void OpenLogStream(void);

  void CloseLogStream(void);

  void InitializeRoutineCars(void);

  void InitializePedestrians(void);

  void SimulationLoop(void);

  std::string get_title(std::string name);

  virtual osmium::unsigned_object_id_type node();

  virtual void UpdateTraffic();

  void StepCars();

  inline void CheckIfCaught(void);

  size_t nedges(osmium::unsigned_object_id_type from) const;

  osmium::unsigned_object_id_type alist(osmium::unsigned_object_id_type from, int to) const;

  int alist_inv(osmium::unsigned_object_id_type from, osmium::unsigned_object_id_type to) const;

  osmium::unsigned_object_id_type salist(osmium::unsigned_object_id_type from, int to) const;

  void set_salist(osmium::unsigned_object_id_type from, int to , osmium::unsigned_object_id_type value);

  osmium::unsigned_object_id_type palist(osmium::unsigned_object_id_type from, int to) const;

  bool hasNode(osmium::unsigned_object_id_type node);

  void StartServer(void);

  int InitCmdHandler(CarLexer &car_lexer, char *buffer);

  int RouteCmdHandler(CarLexer &car_lexer, char *buffer);

  int CarCmdHandler(CarLexer &car_lexer, char *buffer);

  int GangstersCmdHandler(CarLexer &car_lexer, char *buffer);

  int StatCmdHandler(CarLexer &car_lexer, char *buffer);

  int PosCmdHandler(CarLexer &car_lexer, char *buffer);

  int AuthCmdHandler(CarLexer &car_lexer, char *buffer);

  int StopCmdHandler(CarLexer &car_lexer, char *buffer);

  void DispCmdHandler(boost::asio::ip::tcp::socket &socket);

  inline bool IsAuthenticated(CarLexer &car_lexer);

  void CommandListener(boost::asio::ip::tcp::socket socket);

  int addSmartCar(
      justine::robocar::CarType type,
      bool is_guided,
      char *team_name);

  osmium::unsigned_object_id_type naive_node_for_nearest_gangster(
    osmium::unsigned_object_id_type from,
    osmium::unsigned_object_id_type to,
    osmium::unsigned_object_id_type step);

  double Distance(osmium::unsigned_object_id_type n1,
             osmium::unsigned_object_id_type n2) const;

  double Distance(double lon1, double lat1, double lon2, double lat2) const;

  void toGPS(osmium::unsigned_object_id_type from,
             osmium::unsigned_object_id_type to,
             osmium::unsigned_object_id_type step,
             double *lo, double *la) const;

  osmium::unsigned_object_id_type naive_nearest_gangster(
    osmium::unsigned_object_id_type from,
    osmium::unsigned_object_id_type to,
    osmium::unsigned_object_id_type step);

  TrafficType get_type() const;

  int get_time() const;

protected:
  bool is_running_;

  boost::interprocess::managed_shared_memory* shm_segment_;
  boost::interprocess::offset_ptr<SharedMap> shm_map_;

private:
  struct ConnnectivitySettings
  {
    std::string shmName;
    int portNumber;
  };

  struct EntitySettings
  {
    bool routineCarsEnabled;
    bool pedestriansEnabled;
    bool busesEnabled;
    int routineCarCount;
    int pedestrianCount;
    int gangsterCount;
  };

  struct SimulationSettings
  {
    int delay;
    int minutes;
    double catchDistance;
    TrafficType trafficType;
    bool fullLog;
  };

  struct DebugSettings
  {
    bool verboseMode;
  };

  ConnnectivitySettings connnectivitySettings;
  EntitySettings entitySettings;
  SimulationSettings simulationSettings;
  DebugSettings debugSettings;

  int num_gangsters_;

  int running_time_allowed_, running_time_elapsed_;

  TrafficType traffic_type_;

  std::mutex m_mutex, cars_mutex;
  std::condition_variable m_cv;
  std::thread m_thread { &Traffic::SimulationLoop, this };

  std::vector<std::shared_ptr<Car>> cars;
  std::vector<std::shared_ptr<SmartCar>> m_smart_cars;
  std::vector<std::shared_ptr<CopCar>> m_cop_cars;

  std::map<int, std::shared_ptr<SmartCar>> m_smart_cars_map;
  std::map<int, char*> authenticated_teams_;

  boost::asio::io_service io_service_;

  std::string   logfile_name_, configfile_name_;
  std::fstream *logfile_stream_;

  friend std::ostream & operator<<(std::ostream & os, Traffic &);
};

}
} // justine::robocar::

#endif // ROBOCAR_TRAFFIC_HPP
