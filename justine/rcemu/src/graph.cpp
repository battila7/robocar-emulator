#include <graph.hpp>

using GraphNodeID = osmium::unsigned_object_id_type;

 GraphNodeID justine::sampleclient::Graph::palist ( GraphNodeID from, int to ) const
 {
    justine::robocar::SharedMap::iterator iter=shm_map->find ( from );
    return iter->second.m_palist[to];
 }

void justine::sampleclient::Graph::BuildGraph(void)
{
  	nrg = new NodeRefGraph();
  	std::map<GraphNodeID, NRGVertex>::iterator it;
  	NRGVertex f, t;

  	for(justine::robocar::SharedMap::iterator iter = shm_map->begin(); iter!=shm_map->end(); ++iter){

    	GraphNodeID u = iter->first;
      	it = nr2v.find(u);
      	if(it == nr2v.end()){
        	f = boost::add_vertex(u, *nrg);
        	nr2v[u] = f;}
      	else{f = it->second;}

	    for(justine::robocar::UIntVector::iterator noderefi = iter->second.m_alist.begin(); noderefi != iter->second.m_alist.end(); ++noderefi){

	      	it = nr2v.find(*noderefi);
	      	if(it == nr2v.end()){
	        	t = boost::add_vertex(*noderefi, *nrg);
	        	nr2v[*noderefi] = t;}
	      	else{t = it->second;}

	      	int to = std::distance(iter->second.m_alist.begin(), noderefi);
	      	boost::add_edge(f, t, palist(iter->first, to), *nrg);
	    }
  	}
}
