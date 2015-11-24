#include <extendedgraph.hpp>

using GraphNodeID = osmium::unsigned_object_id_type;
using NodeRefGraph = boost::adjacency_list<boost::listS, boost::vecS, boost::directedS,boost::property<boost::vertex_name_t, GraphNodeID>,boost::property<boost::edge_weight_t, int>>;
using NRGVertex = boost::graph_traits<NodeRefGraph>::vertex_descriptor;
using NRGVertexIter = boost::graph_traits<NodeRefGraph>::vertex_iterator;
using NRGEdge = boost::graph_traits<NodeRefGraph>::edge_descriptor;
using NRGEdgeIter = boost::graph_traits<NodeRefGraph>::edge_iterator;
using NRGAdjacentVertexIter = boost::graph_traits<NodeRefGraph>::adjacency_iterator;
using VertexNameMap = boost::property_map<NodeRefGraph, boost::vertex_name_t>::type;
using VertexIndexMap = boost::property_map<NodeRefGraph, boost::vertex_index_t>::type;
using PredecessorMap = boost::iterator_property_map <NRGVertex*, VertexIndexMap, NRGVertex, NRGVertex&>;
using DistanceMap = boost::iterator_property_map <int*, VertexIndexMap, int, int&>;
using EdgeWeightMap = boost::property_map<NodeRefGraph, boost::edge_weight_t>::type;


	void justine::sampleclient::ExtendedGraph::ElliminateCircles(void)
	{
	;
	}

	std::pair<NRGVertexIter, NRGVertexIter> justine::sampleclient::ExtendedGraph::getVertices(void)
	{
		return boost::vertices(*nrg);
	}

	std::pair<NRGEdgeIter, NRGEdgeIter> justine::sampleclient::ExtendedGraph::getEdges(void)
	{
		return boost::edges(*nrg);
	}

	VertexIndexMap justine::sampleclient::ExtendedGraph::getVertexIndexMap(void)
	{
		return boost::get(boost::vertex_index, *nrg);
	}

	VertexNameMap justine::sampleclient::ExtendedGraph::getVertexNameMap(void)
	{
		return boost::get(boost::vertex_name, *nrg);
	}

	NRGVertex justine::sampleclient::ExtendedGraph::findObject(GraphNodeID id)
	{
		return nr2v[id];
	}

	std::pair<NRGAdjacentVertexIter, NRGAdjacentVertexIter> justine::sampleclient::ExtendedGraph::getAdjacentVertices(NRGVertex v)
	{
		return boost::adjacent_vertices(v, *nrg);
	}

	double justine::sampleclient::ExtendedGraph::getDistance(GraphNodeID n1, GraphNodeID n2)
	{
		justine::robocar::SharedMap::iterator iter1=shm_map->find ( n1 );
	    justine::robocar::SharedMap::iterator iter2=shm_map->find ( n2 );

	    if(iter1==shm_map->end()){
	    	VertexNameMap vertexNameMap = getVertexNameMap();
	    	iter1=shm_map->find ( vertexNameMap[n1] );
	    	iter2=shm_map->find ( vertexNameMap[n2] );
	    }

	    osmium::geom::Coordinates c1 {iter1->second.lon/10000000.0, iter1->second.lat/10000000.0};
	    osmium::geom::Coordinates c2 {iter2->second.lon/10000000.0, iter2->second.lat/10000000.0};

	    return osmium::geom::haversine::distance ( c1, c2 );
	}

	double justine::sampleclient::ExtendedGraph::pathLength(std::vector<GraphNodeID> path, int accuracy)
	{
		double distance = 0;
		if(path.size()<2) return distance;
		unsigned int step = std::ceil(100/(double)accuracy);
  		if(path.size()>step+1){
			for(unsigned int i=0;i<path.size()-step;i+=step){
				distance+=getDistance(path.at(i), path.at(i+step));
			}
		}
		return distance;
	}

	std::vector<GraphNodeID> justine::sampleclient::ExtendedGraph::DetermineDijkstraPath(GraphNodeID from, GraphNodeID to)
	{

    std::vector<NRGVertex> parents ( boost::num_vertices ( *nrg ) );
    std::vector<int> distances ( boost::num_vertices ( *nrg ) );

    VertexIndexMap vertexIndexMap = boost::get ( boost::vertex_index, *nrg );

    PredecessorMap predecessorMap ( &parents[0], vertexIndexMap );
    DistanceMap distanceMap ( &distances[0], vertexIndexMap );

    boost::dijkstra_shortest_paths ( *nrg, nr2v[from],
                                     boost::distance_map ( distanceMap ).predecessor_map ( predecessorMap ) );

    VertexNameMap vertexNameMap = boost::get ( boost::vertex_name, *nrg );

    std::vector<osmium::unsigned_object_id_type> path;

    NRGVertex tov = nr2v[to];
    NRGVertex fromv = predecessorMap[tov];


    while ( fromv != tov )
      {

        NRGEdge edge = boost::edge ( fromv, tov, *nrg ).first;


        path.push_back ( vertexNameMap[boost::target ( edge, *nrg )] );

        tov = fromv;
        fromv = predecessorMap[tov];
      }
    path.push_back ( from );

    std::reverse ( path.begin(), path.end() );

    return path;
	}

	std::vector<GraphNodeID> operator+ (std::vector<GraphNodeID> lhs, std::vector<GraphNodeID> rhs)
	{
		std::vector<GraphNodeID> v;
		for (std::vector<GraphNodeID>::iterator i = lhs.begin();i!=lhs.end();++i) { v.push_back(*i); }
		v.pop_back();
		for (std::vector<GraphNodeID>::iterator i = rhs.begin();i!=rhs.end();++i) { v.push_back(*i); }
		return v;
	}
