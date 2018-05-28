//
// Implementation of a model predictive controller
//
// Note: units in this project is not self-consistent
// We assume the speed is in MPH and the time is in second
//
#include <vector>
#include <chrono>
#include <thread>

#include <uWS/uWS.h>
#include <nlohmann/json.hpp>

#include "mpc.h"

// for convenience
using json = nlohmann::json;

//
// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
//
static inline string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_last_of("}]");

    if (found_null != string::npos) {
        return "";
    }
    if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

int main()
{
    uWS::Hub h;

    // mpc is initialized here!
    MPC mpc;

    h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                       uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        string sdata = string(data).substr(0, length);
        // cout << sdata << endl;
        if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
            string s = hasData(sdata);
            if (s != "") {
                auto j = json::parse(s);
                string event = j[0].get<string>();
                if (event == "telemetry") {
                    // j[1] is the data JSON object

                    // global x/y positions of the waypoints
                    vector<double> ptsx = j[1]["ptsx"];
                    vector<double> ptsy = j[1]["ptsy"];

                    // current global x/y positions of the vehicle in meter
                    double px = j[1]["x"];
                    double py = j[1]["y"];

                    // orientation of the vehicle in radians converted
                    // from the Unity format to the standard format
                    double psi = j[1]["psi"];

                    // current velocity in MPH
                    double v = j[1]["speed"];

                    // current steering angle in Radians
                    double steering = j[1]["steering_angle"];

                    // current throttle between [-1, 1]
                    double throttle = j[1]["throttle"];

                    // search the optimized solution with given initial state
                    vector<double> state0 {px, py, psi, v};
                    vector<double> actuator0 {steering, throttle};
                    mpc.solve(state0, actuator0, ptsx, ptsy);

                    double steering_value = mpc.getSteering();
                    double throttle_value = mpc.getThrottle();

                    json msgJson;

                    msgJson["steering_angle"] = steering_value;
                    msgJson["throttle"] = throttle_value;

                    // the mpc predicted trajectory
                    //
                    // - Points are in reference to the vehicle's coordinate system
                    // - The points in the simulator are connected by a green line
                    msgJson["mpc_x"] = mpc.getPredx();
                    msgJson["mpc_y"] = mpc.getPredy();

                    // the reference trajectory
                    //
                    // - Points are in reference to the vehicle's coordinate system
                    // - The points in the simulator are connected by a Yellow line
                    msgJson["next_x"] = mpc.getRefx();
                    msgJson["next_y"] = mpc.getRefy();

                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    // cout << msg << endl;

                    int latency_in_ms = mpc.getLatency()*1000;  // convert s to ms

                    this_thread::sleep_for(chrono::milliseconds(latency_in_ms));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            } else {
                // Manual driving
                string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                       size_t, size_t) {
        const string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        cout << "Connected!!!" << endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length) {
        ws.close();
        cout << "Disconnected" << endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        cout << "Listening to port " << port << endl;
    } else {
        cerr << "Failed to listen to port" << endl;
        return -1;
    }
    h.run();
}
