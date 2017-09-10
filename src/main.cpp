#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

//====== namespace ======
using json = nlohmann::json;

//====== CONSTANTS ======
#define POLY_ORDER (3)
#define LF (2.67)
#define LATENCY (0.1)

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto nIdx1 = s.find_first_of("[");
  auto nIdx2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (nIdx1 != string::npos && nIdx2 != string::npos) {
    return s.substr(nIdx1, nIdx2 - nIdx1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

// Get derivative at x
double polyderive(Eigen::VectorXd coeffs, double dX) {
  double dResult = 0.0;
  for (int i = 1; i < coeffs.size(); i++) {
    dResult += i * coeffs[i] * pow(dX, i - 1);
  }
  return dResult;
}

// Transform to vehicle coords
void coordsTransform(
                     const vector<double>& vX, const vector<double>& vY,
                     double dMapX, double dMapY, double dMapPsi,
                     Eigen::VectorXd& vVehicleX, Eigen::VectorXd& vVehicleY) {
  const int n = vX.size();
  Eigen::MatrixXd mPositions(n, 2);
  mPositions << Eigen::Map<const Eigen::VectorXd>(vX.data(), vX.size()),
                Eigen::Map<const Eigen::VectorXd>(vY.data(), vY.size());
  
  Eigen::Matrix2d mRotate;
  mRotate << cos(-dMapPsi), -sin(-dMapPsi), sin(-dMapPsi), cos(-dMapPsi);
  mPositions.rowwise() -= Eigen::RowVector2d(dMapX, dMapY);
  mPositions = mPositions * mRotate.transpose();
  vVehicleX = mPositions.col(0);
  vVehicleY = mPositions.col(1);
}

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
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          
          // j[1] is the data JSON object
          const vector<double> ptsx = j[1]["ptsx"];
          const vector<double> ptsy = j[1]["ptsy"];
          const double px = j[1]["x"];
          const double py = j[1]["y"];
          const double psi = j[1]["psi"];
          const double v = j[1]["speed"];
          const double steer = j[1]["steering_angle"];
          const double throttle = j[1]["throttle"];

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          Eigen::VectorXd xVehicle;
          Eigen::VectorXd yVehicle;
          coordsTransform(ptsx, ptsy, px, py, psi, xVehicle, yVehicle);
          const auto coeffs = polyfit(xVehicle, yVehicle, POLY_ORDER);
          const double cte = polyeval(coeffs, 0.0);
          const double epsi = -atan(polyderive(coeffs, 0.0)); // invert angle in vehicle frame
          
          // form the new state, taking latency into account
          const double multiplier = (steer / LF); // compute optimizer
          const double xNew = 0.0 + (v * cos(0.0) * LATENCY);
          const double yNew = 0.0 + (v * sin(0.0) * LATENCY);
          const double psiNew = 0.0 - (v * multiplier * LATENCY);
          const double vNew = v + (throttle * LATENCY);
          const double cteNew = cte - (v * sin(epsi) * LATENCY);
          const double epsiNew = epsi - (v * multiplier * LATENCY);
          Eigen::VectorXd newState(6);
          newState << xNew, yNew, psiNew, vNew, cteNew, epsiNew;
          
          // solve MPC
          const auto solution = mpc.Solve(newState, coeffs);
          
//          double steer_value;
//          double throttle_value;
          
          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = solution[0];
          msgJson["throttle"] = solution[1];

          //Display the MPC predicted trajectory 
//          vector<double> mpc_x_vals;
//          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc.getXs();
          msgJson["mpc_y"] = mpc.getYs();

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          for(int i = 0; i < xVehicle.size(); i++){
            if(xVehicle[i] >= 0) {
              next_x_vals.push_back(xVehicle[i]);
              next_y_vals.push_back(yVehicle[i]);
            }
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
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
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
