#ifndef ROBOCAR_OSMREADER_HPP
#define ROBOCAR_OSMREADER_HPP

/**
 * @brief Justine - this is a rapid prototype for development of Robocar City Emulator
 *
 * @file osmreader.hpp
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

#include <iostream>
#include <map>
#include <set>
#include <vector>
#include <string>
#include <algorithm>
#include <fstream>
#include <exception>
#include <stdexcept>

#include <osmium/io/any_input.hpp>
#include <osmium/handler.hpp>
#include <osmium/visitor.hpp>
#include <osmium/osm/node.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/osm/relation.hpp>
#include <osmium/index/map/sparse_mem_table.hpp>
#include <osmium/index/map/sparse_mem_map.hpp>
#include <osmium/handler/node_locations_for_ways.hpp>
#include <osmium/geom/haversine.hpp>
#include <osmium/geom/coordinates.hpp>

namespace justine
{

namespace robocar
{

using OSMLocations =
  osmium::index::map::SparseMemMap<osmium::unsigned_object_id_type, osmium::Location>;

using OsmiumObjIdVector = std::vector<osmium::unsigned_object_id_type>;

using NodeToLocationMap = std::map<osmium::unsigned_object_id_type, osmium::Location>;
using WayNodesMap = std::map<std::string, OsmiumObjIdVector>;
//typedef osmium::index::map::StlMap<osmium::unsigned_object_id_type, osmium::Location> WaynodeLocations;
using WayToNodeMap = std::map<osmium::unsigned_object_id_type, OsmiumObjIdVector>;
using NodeAdjacencyMap = std::map<osmium::unsigned_object_id_type, OsmiumObjIdVector>;

// Thanks to this inheritance we have "callbacks" to
// the most important OSM data types: Node, Way, Relation
class OSMReader : public osmium::handler::Handler
{
public:
  OSMReader (const char* osmFile,
             NodeAdjacencyMap& nodeAdjacencyMap,
             NodeAdjacencyMap& palist,
             NodeToLocationMap& wayNodeLocations,
             WayToNodeMap& busWayMap,
             WayToNodeMap& wayToNode):
    nodeAdjacencyMap(nodeAdjacencyMap),
    palist(palist),
    wayNodeLocations(wayNodeLocations),
    busWayMap(busWayMap),
    wayToNode(wayToNode)
  {
    processFile(osmFile);
  }

  ~OSMReader()
  {
    google::protobuf::ShutdownProtobufLibrary();
  }

  void processFile(const char* osmFile)
  {
    try
    {
      #ifdef DEBUG
      std::cout << "\n OSMReader is running... " << std::endl;
      #endif

      osmium::io::File inputFile(osmFile);

      // read everything -> nodes, ways, relations
      osmium::io::Reader reader(inputFile);

      using SparseLocations =
        osmium::index::map::SparseMemMap<osmium::unsigned_object_id_type, osmium::Location>;

      osmium::handler::NodeLocationsForWays<SparseLocations> node_locations (locations);

      // starts OSM file processing
      // we pass ourselves to indicate that we are responsible for the "callbacks"
      osmium::apply (reader, node_locations, *this);

      reader.close();

      #ifdef DEBUG
      printStatistics();
      #endif

      // why?
      estimatedMemorySize *= 8;
    }
    catch ( std::exception &err )
    {
      google::protobuf::ShutdownProtobufLibrary();

      throw;
    }
  }

  void printStatistics() const
  {
    // the last thing I'm interested in
    /*
    std::cout << " #OSM Nodes: " << nOSM_nodes << "\n";
    std::cout << " #OSM Highways: " << nOSM_ways << "\n";
    std::cout << " #Kms (Total length of highways) = " << sum_highhway_length/1000.0 << std::endl;
    std::cout << " #OSM Relations: " << nOSM_relations << "\n";
    std::cout << " #Highway Nodes: " << sum_highhway_nodes << "\n";
    std::cout << " #Unique Highway Nodes: " << sum_unique_highhway_nodes << "\n";
    std::cout << " E= (#Highway Nodes - #OSM Highways): " << sum_highhway_nodes - nOSM_ways << "\n";
    std::cout << " V= (#Unique Highway Nodes):" << sum_unique_highhway_nodes << "\n";
    std::cout << " V^2/E= "
              <<
              ( ( long ) sum_unique_highhway_nodes * ( long ) sum_unique_highhway_nodes )
              / ( double ) ( sum_highhway_nodes - nOSM_ways )
              << "\n";
    std::cout << " Edge_multiplicity: " << edge_multiplicity << std::endl;
    std::cout << " max edle length: " << max_edge_length << std::endl;
    std::cout << " edle length mean: " << mean_edge_length/cedges << std::endl;
    std::cout << " cedges: " << cedges << std::endl;

    std::cout << " #Buses = " << busWayNodesMap.size() << std::endl;

    std::cout << " Node locations " << locations.used_memory() /1024.0/1024.0 << " Mbytes" << std::endl;
    //std::cout << " Waynode locations " << waynode_locations.used_memory() /1024.0/1024.0 << " Mbytes" << std::endl;
    std::cout << " Vertices " << vert.used_memory() /1024.0/1024.0 << " Mbytes" << std::endl;
    std::cout << " way2nodes " << way2nodes.size() << "" << std::endl;

    std::set<osmium::unsigned_object_id_type> sum_vertices;
    std::map<osmium::unsigned_object_id_type, size_t>  word_map;
    int sum_edges {0};
    for ( auto busit = begin ( alist );
          busit != end ( alist ); ++busit )
      {

        sum_vertices.insert ( busit->first );
        sum_edges+=busit->second.size();

        for ( const auto &v : busit->second )
          {
            sum_vertices.insert ( v );
          }

      }
    std::cout << " #citymap edges = "<< sum_edges<< std::endl;
    std::cout << " #citymap vertices = "<< sum_vertices.size() << std::endl;
    std::cout << " #citymap vertices (deg- >= 1) = "<< alist.size() << std::endl;
    std::cout << " #onewayc = "<< onewayc<< std::endl;
    */
  }

  std::size_t getEstimatedMemory() const
  {
    return estimatedMemorySize;
  }

  inline bool hasEdgeBetween(osmium::unsigned_object_id_type a, osmium::unsigned_object_id_type b)
  {
    const auto& adjList = nodeAdjacencyMap[a];

    return std::find(adjList.cbegin(), adjList.cend(), b) != adjList.cend();
  }

  // called on each node in the OSM
  void node(osmium::Node& node)
  {
    ++numNodes;
  }

  int numOneWays {0};
  int isOneWay {false};

  void way(osmium::Way& way)
  {
    const char* highway = way.tags() ["highway"];

    // highway tag does not exist
    if (highway == nullptr)
    {
      return;
    }

    // we don't want ways with the following highway values to
    // be processed
    std::vector<std::string> undesiredValues =
      { "footway", "cycleway", "bridleway", "steps", "path", "construction" };

    if (std::find(undesiredValues.cbegin(),
                  undesiredValues.cend(),
                  std::string(highway)) != undesiredValues.cend())
    {
      return;
    }

    isOneWay = false;

    const char* oneWay = way.tags()["oneway"];

    if (oneWay != nullptr)
    {
      isOneWay = true;

      ++numOneWays;
    }

    ++numWays;

    double wayLength = osmium::geom::haversine::distance (way.nodes());

    sumHighhwayLength += wayLength;

    int numNodeMembers = 0;

    // it's called ..Vertex, because it corresponds to a Vertex in the
    // graph that will be built from the OSM data
    // Clearly nodes == vertices, but we use node in an OSM context,
    // and vertex in the graph context
    osmium::unsigned_object_id_type previousVertex;

    // loop over all nodes in the current way
    for (const osmium::NodeRef& nr : way.nodes())
    {
      osmium::unsigned_object_id_type currentVertex = nr.positive_ref();

      // a map storing the nodes for each way
      wayToNode[way.id()].push_back(currentVertex);

      try
      {
        // if the node currently looped over is not found in the map
        // get() throws an exception
        nodesEncountered.get(currentVertex);
      }
      catch (std::exception& e)
      {
        // the value itself is not really important, only that it exists
        nodesEncountered.set(currentVertex, true);

        ++sumUniqueHighhwayNodes;

        wayNodeLocations[currentVertex] = nr.location();
      }

      // "A way can have between 2 and 2,000 nodes..."
      // Some faulty ways have 1 or 0 nodes, but we don't care about that
      // We can be sure, that there will be at least one edge
      if (numNodeMembers > 0)
      {
        if (!hasEdgeBetween(previousVertex, currentVertex))
        {
          makeEdgeBetween(previousVertex, currentVertex);
        }
        else
        {
          ++edgeMultiplicity;
        }

        // if the current way is not a oneway,
        // last --- current and current --- last
        // edges will be created
        if (!isOneWay)
        {
          if (!hasEdgeBetween(currentVertex, previousVertex))
          {
            makeEdgeBetween(currentVertex, previousVertex);
          }
          else
          {
            ++edgeMultiplicity;
          }
        }
      }

      previousVertex = currentVertex;

      ++numNodeMembers;
    }

    sumHighhwayNodes += numNodeMembers;
  }

  void relation (osmium::Relation& rel)
  {
    ++numRelations;

    // don't care, gonna be rewritten
    /*
    const char* bus = rel.tags()["route"];

    if (bus && !strcmp ( bus, "bus" ) )
      {

        ++nbuses;

        std::string ref_key;

        try
          {
            const char* ref = rel.tags() ["ref"];
            if ( ref )
              ref_key.append ( ref );
            else
              ref_key.append ( "Not specified" );

          }
        catch ( std::exception& e )
          {
            std::cout << "There is no bus number."<< e.what() << std::endl;
          }

        osmium::RelationMemberList& rml = rel.members();
        for ( osmium::RelationMember& rm : rml )
          {

            if ( rm.type() == osmium::item_type::way )
              {

                busWayNodesMap[ref_key].push_back ( rm.ref() );

              }
          }

      }*/
  }

protected:
  int numNodes = 0, numWays = 0, numRelations = 0, numEdges = 0;

  int sumUniqueHighhwayNodes = 0, sumHighhwayNodes = 0, sumHighhwayLength = 0;

  int edgeMultiplicity = 0;

  double maxEdgeLength = 0, meanEdgeLength = 0;

  OSMLocations  locations;

  osmium::index::map::SparseMemMap<osmium::unsigned_object_id_type, bool> nodesEncountered;

private:
  // Returns the real distance between two nodes identified by their IDs
  inline double distanceBetween(osmium::unsigned_object_id_type a, osmium::unsigned_object_id_type b)
  {
    osmium::geom::Coordinates coordA(locations.get(a)),
                              coordB(locations.get(b));

    return osmium::geom::haversine::distance(coordA, coordB);
  }

  // makes an edge between two given nodes
  // manipulates the nodeAdjacencyMap
  void makeEdgeBetween(osmium::unsigned_object_id_type a, osmium::unsigned_object_id_type b)
  {
    nodeAdjacencyMap[a].push_back(b);

    double edgeLength = distanceBetween(b, a);

    palist[a].push_back(edgeLength / 3.0 );

    if (edgeLength > maxEdgeLength)
    {
      maxEdgeLength = edgeLength;
    }

    meanEdgeLength += edgeLength;

    ++estimatedMemorySize;

    ++numEdges;
  }

  std::size_t estimatedMemorySize = 0u;

  NodeAdjacencyMap &nodeAdjacencyMap, &palist;
  NodeToLocationMap &wayNodeLocations;
  WayToNodeMap &busWayMap, &wayToNode;
};

}
} // justine::robocar::

#endif // ROBOCAR_OSMREADER_HPP
