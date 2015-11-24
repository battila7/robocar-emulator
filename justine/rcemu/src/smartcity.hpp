#ifndef ROBOCAR_SMARTCITY_HPP
#define ROBOCAR_SMARTCITY_HPP

/**
* @brief Justine - this is a rapid prototype for development of Robocar City Emulator
*
* @file smartcity.hpp
* @author  Norbert Bátfai <nbatfai@gmail.com>
* @version 0.0.10
*
* @section LICENSE
*
* Copyright (C) 2014 Norbert Bátfai, batfai.norbert@inf.unideb.hu
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
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
#include <exception>
#include <stdexcept>
#include <iomanip>

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/map.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/containers/string.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "osmreader.hpp"

namespace justine
{

namespace robocar
{
  // too long, too ugly -> enough said
  namespace bip = boost::interprocess;

  // generic interprocess allocation stuff
  using SegmentManager = bip::managed_shared_memory::segment_manager;
  using VoidAllocator = bip::allocator<void, SegmentManager>;

  // unsigned int vector stuff
  using UIntAllocator = bip::allocator<unsigned int, SegmentManager>;
  using UIntVector = bip::vector<unsigned int, UIntAllocator>;
  using UIntVectorAllocator = bip::allocator<UIntVector, SegmentManager>;


  class SharedData
  {
  public:
    UIntVector m_alist;
    UIntVector m_salist;
    UIntVector m_palist;

    int lon, lat;

    SharedData (const VoidAllocator &voidAlloc):
     m_alist(voidAlloc), m_salist(voidAlloc), m_palist (voidAlloc)
    {}
  };

  // graph data interprocess container stuff
  using SharedMapPair = std::pair<const unsigned int, SharedData>;
  using SharedMapPairAllocator = bip::allocator<SharedMapPair, SegmentManager>;
  using SharedMap =
    bip::map<unsigned int, SharedData, std::less<unsigned int>, SharedMapPairAllocator>;

  class SmartCity
  {
  public:
    SmartCity(std::string config):
      configFile(config)
    {
      ProcessConfigFile();

      CreateSharedMemory();

      if (mapSettings.nodeToGpsFile != "")
      {
        BuildNodeToGpsFile();
      }

      std::cout << "\t" << mapSettings.cityName << " is ready." << std::endl;
    }

    void ProcessConfigFile()
    {
      namespace pt = boost::property_tree;

      std::cout << "\tReading config file: " << configFile
                << std::endl;

      pt::ptree propTree;

      pt::read_json(configFile, propTree);

      shmSegmentName =
        propTree.get("connectivity.shm", "JustineSharedMemory");

      mapSettings.osmFile =
        propTree.get("map.osmFile", "debrecen.osm");
      mapSettings.cityName =
        propTree.get("map.cityName", "Debrecen");
      mapSettings.nodeToGpsFile =
        propTree.get("map.nodeToGpsFile", "");
    }

    void BuildNodeToGpsFile()
    {
      // output Node ID Latitude Longitude data into the GPS file
      // the file can be used by the rcwin (or similar) display processes
      std::fstream gpsFile(mapSettings.nodeToGpsFile, std::ios_base::out);

      for (const auto& loc : waynodeLocations)
      {
        gpsFile << loc.first
                << " " << loc.second.lat()
                << " " << loc.second.lon()
                << std::endl;
      }

      gpsFile.close();

      std::cout << "\tNodeToGPS file built: " << mapSettings.nodeToGpsFile
                << std::endl;
    }

    void CreateSharedMemory()
    {
      NodeAdjacencyMap adjacencyList, palist;

      std::size_t estimatedSize;

      try
      {
        std::cout << "\tProcessing OSM file: " << mapSettings.osmFile
                  << std::endl;

        OSMReader osmReader(mapSettings.osmFile.c_str(), adjacencyList, palist,
                            waynodeLocations, buswayNodesMap,
                            wayToNodes);

        estimatedSize = 20 * 3 * osmReader.getEstimatedMemory();
      }
      catch (std::exception &err) //TODO: More specific exception
      {
        osmLoadedCondVar.notify_one();

        isRunning = false;

        thread.join();

        throw;
      }

      google::protobuf::ShutdownProtobufLibrary();

      // If there was a previously created shared memory segment with the same
      // name, it will be deleted
      // It returns true upon deletion but we really don't care
      bip::shared_memory_object::remove(shmSegmentName.c_str());

      std::cout << "\tCreating shared memory segment: " << shmSegmentName
                << std::endl;

      shmSegment =
        new bip::managed_shared_memory(bip::create_only, shmSegmentName.c_str(), estimatedSize);

      VoidAllocator  allocatorObject(shmSegment->get_segment_manager());

      SharedMap* sharedMap =
        shmSegment->construct<SharedMap>("JustineMap")(std::less<unsigned int>(), allocatorObject);

      try
      {
        // We cycle through the nodes and fill up the
        // shared data objects
        // Then we put this stuff into the shared memory
        // first - ID, second - the real adjacency list
        for (const auto& node : adjacencyList)
        {
          SharedData sharedData(allocatorObject);

          sharedData.lon = waynodeLocations[node.first].x();
          sharedData.lat = waynodeLocations[node.first].y();

          std::size_t i = 0;

          for (const osmium::unsigned_object_id_type adjacentNodeId: node.second)
          {
            sharedData.m_alist.push_back(adjacentNodeId);
            sharedData.m_salist.push_back(0u);
            sharedData.m_palist.push_back(palist[node.first][i] + 1);

            i++;
          }

          SharedMapPair mapPair(node.first, sharedData);

          sharedMap->insert(mapPair);
        }

        #ifdef DEBUG
        std::cout << " adjacencyList.size = " << adjacencyList.size() << " (deg- >= 1)"<< std::endl;
        std::cout << " SharedMap/alist.size = " << sharedMap->size() << std::endl;
        #endif
      }
      catch (boost::interprocess::bad_alloc e)
      {
        std::cerr << " Out of shared memory..." << std::endl
                  << e.what() << std::endl;

        std::cerr
          << "Shared memory usage: "
          << shmSegment->get_free_memory() / std::pow(1024, 2) << " Mbytes "
          << std::setprecision (2)
          << 100.0 - (100.0 * shmSegment->get_free_memory()) / shmSegment->get_size()
          << "% is free"
          << std::endl;

        osmLoadedCondVar.notify_one();

        isRunning = false;
        thread.join();

        throw e;
      }

      #ifdef DEBUG
      std::cout
        << "Shared memory usage: "
        << shmSegment->get_free_memory() / std::pow(1024, 2) << " Mbytes "
        << std::setprecision (2)
        << 100.0 - (100.0 * shmSegment->get_free_memory()) / shmSegment->get_size()
        << "% is free"
        << std::endl;
      #endif

      sharedMapOffsetPtr = shmSegment->find<SharedMap> ( "JustineMap" ).first;

      osmLoadedCondVar.notify_one();
    }

    ~SmartCity()
    {
      isRunning = false;

      thread.join();

      delete shmSegment;

      bip::shared_memory_object::remove(shmSegmentName.c_str());
    }

    void processes()
    {
      std::unique_lock<std::mutex> lock(mutex);

      osmLoadedCondVar.wait(lock);

      while (isRunning)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(delay));

        cityRun();
      }
    }

    void Serialize()
    {
      /*
      for ( shm_map_Type::iterator iter=t.shm_map->begin();
            iter!=t.shm_map->end(); ++iter )
      {
        std::cout
          << iter->first
          << " "
          << iter->second.lon
          << " "
          << iter->second.lat
          << " "
          << iter->second.m_alist.size()
          << " ";

        for ( auto noderef : iter->second.m_alist )
        {
          std::cout
            << noderef
            << " ";
        }

        for ( auto noderef : iter->second.m_salist )
        {
          std::cout
            << noderef
            << " ";
        }

        for ( auto noderef : iter->second.m_palist )
        {
          std::cout
          << noderef
          << " ";
        }

        std::cout << std::endl;
      }

      return os;*/
    }

    virtual void cityRun()
    {
      // activities that may occur in the city

      // std::cout << *this;
    }

    double busWayLength (bool verbose);

  protected:
    struct MapSettings {
      std::string osmFile, cityName, nodeToGpsFile;
    };

    MapSettings mapSettings;

    std::string shmSegmentName, configFile;

    bip::managed_shared_memory* shmSegment;
    bip::offset_ptr<SharedMap> sharedMapOffsetPtr;

    int delay = 5000;
    bool isRunning = true;

  private:
    std::mutex mutex;
    std::condition_variable osmLoadedCondVar;
    std::thread thread {&SmartCity::processes, this};

    NodeToLocationMap waynodeLocations;
    WayToNodeMap buswayNodesMap;
    WayToNodeMap wayToNodes;
  };

}

} // justine::robocar::


#endif // ROBOCAR_SMARTCITY_HPP
