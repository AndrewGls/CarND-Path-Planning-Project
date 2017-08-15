#include <fstream>
#define _USE_MATH_DEFINES
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "Waypoints.h"
#include "Utils.hpp"
#include "Vehicle.h"

// This define was added to fix the running of PID controller with simulator on Windows 10.
// Uncomment it to run PID on Windows.
//#define  use_ipv4

using namespace std;
using namespace Utils;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  Waypoints map;
  Vehicle vehicle(map);

  h.onMessage([&vehicle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
			CarLocalizationData vLocal;
			vLocal.x = j[1]["x"];
			vLocal.y = j[1]["y"];
			vLocal.s = j[1]["s"];
			vLocal.d = j[1]["d"];
			vLocal.yaw = j[1]["yaw"];
			vLocal.speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	//double end_path_s = j[1]["end_path_s"];
          	//double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

			vector<SensorFusionData>& sf_data = vehicle.SensorFusionStorage();
			sf_data.resize(sensor_fusion.size());
			for (int i = 0; i < sensor_fusion.size(); i++)
			{
				SensorFusionData& sfd = sf_data [i];
				sfd.id = sensor_fusion [i][0];
				sfd.x = sensor_fusion [i][1];
				sfd.y = sensor_fusion [i][2];
				sfd.vx = sensor_fusion [i][3];
				sfd.vy = sensor_fusion [i][4];
				sfd.s = sensor_fusion [i][5];
				sfd.d = sensor_fusion [i][6];
			}

			vLocal.d = -vLocal.d;
			vehicle.UpdateTrajectory(vLocal, previous_path_x, previous_path_y);

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
			msgJson["next_x"] = vehicle.get_next_x_vals();
			msgJson["next_y"] = vehicle.get_next_y_vals();

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
#ifdef use_ipv4
  if (h.listen("0.0.0.0", port)) {
#else
  if (h.listen(port)) {
#endif
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
