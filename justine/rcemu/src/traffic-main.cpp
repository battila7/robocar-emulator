/**
 * @brief Justine - this is a rapid prototype for development of Robocar City Emulator
 *
 * @file traffic-main.cpp
 * @author  Norbert B�tfai <nbatfai@gmail.com>
 * @version 0.0.10
 *
 * @section LICENSE
 *
 * Copyright(C) 2014 Norbert B�tfai, batfai.norbert@inf.unideb.hu
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
 */

#include <boost/program_options.hpp>

#include "traffic.hpp"

int main(int argc, char* argv[])
{
  namespace po = boost::program_options;

  int t = std::time(nullptr);

  #ifdef DEBUG
  std::cout << "srand init =  " << t << std::endl;
  #endif

  std::srand(t);

  po::options_description desc("Options");

  desc.add_options()
    ("version,v",  "produce version message")
    ("help,h",   "produce help message")
    ("config,c", po::value<std::string>()->default_value("traffic.json"),
                 "The file containing the configuration settings");

  std::string traffic_type;

  po::variables_map vm;

  po::store(po::parse_command_line(argc, argv, desc), vm);

  po::notify(vm);

  if (vm.count("version"))
  {
    std::cout << "Robocar City Emulator and Robocar World Championship, Traffic Server" << std::endl
              << "Copyright(C) 2014, 2015 Norbert B�tfai\n" << std::endl
              << "License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>" << std::endl
              << "This is free software: you are free to change and redistribute it." << std::endl
              << "There is NO WARRANTY, to the extent permitted by law." << std::endl;

    return 0;
  }

  if (vm.count("help"))
  {
    std::cout << "Robocar City Emulator and Robocar World Championship home page: https://code.google.com/p/robocar-emulator/" << std::endl
              << desc << std::endl
              << "Please report bugs to: nbatfai@gmail.com" << std::endl;
    return 0;
  }

  std::string configFile = vm["config"].as<std::string>();

  justine::robocar::Traffic traffic(configFile);

  try
  {
    traffic.StartServer();
  }
  catch(std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
  }
}
