#
# Robocar City Emulator and Robocar World Championship
# Justine - this is a rapid prototype for development of Robocar City Emulator
# Copyright (C) 2014 Norbert Bátfai, batfai.norbert@inf.unideb.hu
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <http://www.gnu.org/licenses/>.
#


bin_PROGRAMS = smartcity.out traffic.out sampleshmclient.out samplemyshmclient.out

smartcity_out_CXXFLAGS = $(CXXFLAGS) $(BOOST_CPPFLAGS)
smartcity_out_LDFLAGS =  $(BOOST_LDFLAGS) $(BOOST_PROGRAM_OPTIONS_LDFLAGS) $(BOOST_GRAPH_LDFLAGS) $(SHM_OPEN_LIBS) $(LIBS) $(BOOST_SYSTEM_LDFLAGS) $(BOOST_FILESYSTEM_LDFLAGS)  $(BOOST_THREAD_LDFLAGS)
smartcity_out_LIBS =  $(BOOST_PROGRAM_OPTIONS_LIBS) $(BOOST_THREAD_LIBS) $(BOOST_GRAPH_LIBS) $(SHM_OPEN_LIBS) $(LIBS) $(BOOST_FILESYSTEM_LIBS) $(BOOST_SYSTEM_LIBS) $(BOOST_THREAD_LIBS)
smartcity_out_LDADD=  $(BOOST_PROGRAM_OPTIONS_LIBS) $(BOOST_THREAD_LIBS) $(BOOST_GRAPH_LIBS) $(SHM_OPEN_LIBS) $(LIBS) $(BOOST_SYSTEM_LIBS) $(BOOST_FILESYSTEM_LIBS) $(BOOST_THREAD_LIBS)
smartcity_out_SOURCES = smartcity.hpp smartcity.cpp osmreader.hpp

AM_YFLAGS=-d

FLEX=@FLEX@
FLEXFLAGS=@FLEXFLAGS@
AM_FLEXFLEX=@AM_FLEXFLAGS@

.ll.cc:	$*.ll Makefile.am
	$(FLEX) $(FLEXFLAGS) $(AM_FLEXFLAGS) $<
	$(AWK) '/#include <FlexLexer\.h>/ { print "#include \"$*.hpp\""; getline; } \
		/class istream;/ { print "#include <iosfwd>"; print "using namespace std;"; getline; } \
		/#include <iostream\.hpp>/ { print "#include <iostream>"; getline; } \
		{ gsub( "lex\.yy\.c", "$@" ); print; }' lex.yy.c > $@
	rm -f lex.yy.c

traffic_out_CXXFLAGS = $(CXXFLAGS)
traffic_out_LDFLAGS = $(BOOST_LDFLAGS) $(BOOST_DATE_TIME_LDFLAGS) $(BOOST_PROGRAM_OPTIONS_LDFLAGS) $(BOOST_THREAD_LDFLAGS) $(BOOST_ASIO_LDFLAGS) $(SHM_OPEN_LIBS) $(BOOST_LOG_LDFLAGS) $(LIBS) $(BOOST_SYSTEM_LDFLAGS) $(BOOST_FILESYSTEM_LDFLAGS) -lprotobuf
traffic_out_LIBS = $(BOOST_PROGRAM_OPTIONS_LIBS) $(BOOST_DATE_TIME_LIBS) $(BOOST_THREAD_LIBS) $(BOOST_ASIO_LIBS) $(SHM_OPEN_LIBS) $(LIBS) $(BOOST_SYSTEM_LIBS) $(BOOST_LOG_LIBS) $(BOOST_FILESYSTEM_LIBS) -lprotobuf
traffic_out_LDADD = $(BOOST_LDFLAGS) $(BOOST_PROGRAM_OPTIONS_LIBS) $(BOOST_DATE_TIME_LDFLAGS) $(BOOST_THREAD_LDFLAGS) $(BOOST_ASIO_LDFLAGS) $(SHM_OPEN_LIBS) $(LIBS) $(BOOST_SYSTEM_LDFLAGS) $(BOOST_LOG_LDFLAGS) $(BOOST_FILESYSTEM_LDFLAGS) -lprotobuf
traffic_out_SOURCES = carlexer.ll carlexer.hpp smartcity.hpp traffic.hpp traffic.cpp traffic-main.cpp car.hpp car.cpp robocar.pb.h robocar.pb.cc

sampleshmclient_out_CXXFLAGS = $(CXXFLAGS)
sampleshmclient_out_LDFLAGS = $(BOOST_LDFLAGS) $(BOOST_PROGRAM_OPTIONS_LDFLAGS) $(BOOST_THREAD_LDFLAGS) $(BOOST_ASIO_LDFLAGS) $(SHM_OPEN_LIBS) $(BOOST_LOG_LDFLAGS) $(LIBS) $(BOOST_SYSTEM_LDFLAGS)
sampleshmclient_out_LIBS = $(BOOST_PROGRAM_OPTIONS_LIBS) $(BOOST_THREAD_LIBS) $(BOOST_ASIO_LIBS) $(SHM_OPEN_LIBS) $(LIBS) $(BOOST_SYSTEM_LIBS) $(BOOST_LOG_LIBS)
sampleshmclient_out_LDADD = $(BOOST_LDFLAGS) $(BOOST_PROGRAM_OPTIONS_LIBS) $(BOOST_THREAD_LDFLAGS) $(BOOST_ASIO_LDFLAGS) $(SHM_OPEN_LIBS) $(LIBS) $(BOOST_SYSTEM_LDFLAGS) $(BOOST_LOG_LDFLAGS)
sampleshmclient_out_SOURCES = server.hpp server.cpp graph.hpp graph.cpp extendedgraph.hpp extendedgraph.cpp smartcity.hpp car.hpp shmclient.hpp shmclient.cpp shmclient-main.cpp

samplemyshmclient_out_CXXFLAGS = $(CXXFLAGS)
samplemyshmclient_out_LDFLAGS = $(BOOST_LDFLAGS) $(BOOST_PROGRAM_OPTIONS_LDFLAGS) $(BOOST_THREAD_LDFLAGS) $(BOOST_ASIO_LDFLAGS) $(SHM_OPEN_LIBS) $(BOOST_LOG_LDFLAGS) $(LIBS) $(BOOST_SYSTEM_LDFLAGS)
samplemyshmclient_out_LIBS = $(BOOST_PROGRAM_OPTIONS_LIBS) $(BOOST_THREAD_LIBS) $(BOOST_ASIO_LIBS) $(SHM_OPEN_LIBS) $(LIBS) $(BOOST_SYSTEM_LIBS) $(BOOST_LOG_LIBS)
samplemyshmclient_out_LDADD = $(BOOST_LDFLAGS) $(BOOST_PROGRAM_OPTIONS_LIBS) $(BOOST_THREAD_LDFLAGS) $(BOOST_ASIO_LDFLAGS) $(SHM_OPEN_LIBS) $(LIBS) $(BOOST_SYSTEM_LDFLAGS) $(BOOST_LOG_LDFLAGS)
samplemyshmclient_out_SOURCES = server.hpp server.cpp graph.hpp graph.cpp extendedgraph.hpp extendedgraph.cpp smartcity.hpp car.hpp shmclient.hpp shmclient.cpp myshmclient.hpp myshmclient.cpp myshmclient-main.cpp
