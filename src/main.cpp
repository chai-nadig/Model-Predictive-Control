#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "MPC.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];


          // std::cout << "px: " << px;
          // std::cout << " py: " << py;
          // std::cout << " psi: " << psi;
          // std::cout << " v: " << v << std::endl;
          //
          // std::cout << "ptsx: ";
          // for (std::vector<double>::iterator it = ptsx.begin() ; it != ptsx.end(); ++it)
          //     std::cout << ' ' << *it;
          // std::cout << std::endl;
          //
          // std::cout << "ptsy: ";
          // for (std::vector<double>::iterator it = ptsy.begin() ; it != ptsy.end(); ++it)
          //     std::cout << ' ' << *it;
          // std::cout << std::endl;
          //

          vector<double> transformed_x;
          vector<double> transformed_y;

          for (int i = 0; i < ptsx.size(); i++) {
            vector<double> transformed = transform({ptsx[i], ptsy[i], -psi}, {px, py});
            transformed_x.push_back(transformed[0]);
            transformed_y.push_back(transformed[1]);
          }

          VectorXd ptsx2 = Eigen::Map<VectorXd, Eigen::Unaligned>(transformed_x.data(), transformed_x.size());
          VectorXd ptsy2 = Eigen::Map<VectorXd, Eigen::Unaligned>(transformed_y.data(), transformed_y.size());

          /**
          * fit a polynomial to the transformed x and y coordinates
          */
          auto coeffs = polyfit(ptsx2, ptsy2, 3);
          /**
           * calculate the cross track error
           */
          double cte = polyeval(coeffs, 0);
          /**
           * calculate the orientation error
           */

          double psides = coeffs[1];
          double epsi = - atan(psides) ;

          VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi;

          /**
           * Calculate steering angle and throttle using MPC.
           * Both are in between [-1, 1].
           */
          auto vars = mpc.Solve(state, coeffs);

          std::cout << "steer_value: " << rad2deg(-vars[0]);
          //   Divide by deg2rad(25) otherwise the steer_value will be in between
          //   [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          //   Multiply by -1 as the turn value is opposite in the simulator
          double steer_value = -vars[0] / deg2rad(25);
          double throttle_value = vars[1];

          json msgJson;

          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          /**
           *   add (x,y) points to list here, points are in reference to
           *   the vehicle's coordinate system the points in the simulator are
           *   connected by a Green line
           */

           for (int i = 2; i < vars.size(); i+=2) {
             mpc_x_vals.push_back(vars[i]);
             mpc_y_vals.push_back(vars[i+1]);
           }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           *   add (x,y) points to list here, points are in reference to
           *   the vehicle's coordinate system the points in the simulator are
           *   connected by a Yellow line
           */
           for (int i = 0; i < 100; i++) {
             next_x_vals.push_back(i);
             next_y_vals.push_back(polyeval(coeffs, i));
           }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          //   the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          //   around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
