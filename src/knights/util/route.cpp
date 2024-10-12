#include "knights/autonomous/path.h"
#include "knights/util/position.h"

#include "api.h"

#include <fstream>

knights::Route::Route(std::vector<Pos> positions) {
    this->positions = positions;
}

knights::Route::Route() {
    this->positions = {};
}

knights::RouteAction::RouteAction(knights::action_type type, std::string route_name, float end_tolerance, int timeout) :
    type(type), route_name(route_name), end_tolerance(end_tolerance), timeout(timeout) {}

knights::RouteAction::RouteAction(action_type type, float specific, float end_tolerance, int timeout) :
    type(type), end_tolerance(end_tolerance), timeout(timeout) {}

knights::RouteAction::RouteAction(action_type type, void (*bound_function)()) :
    type(type), bound_function(bound_function) {}

knights::AdvancedRoute::AdvancedRoute() {
    this->actions = std::vector<knights::RouteAction>();
    this->routes = std::map<std::string, Route>();
}

knights::AdvancedRoute::AdvancedRoute(std::map<std::string, Route> routes, std::vector<RouteAction> actions) :
    routes(routes), actions(actions) {}

knights::Route knights::operator+(const Route &r1, const Route &r2) {
    return Route(r1.positions + r2.positions);
};

knights::Route knights::operator+(knights::Route r1, const knights::Pos &p1) {
    r1.positions.push_back(p1);
    return r1;
};

knights::Route knights::operator-(knights::Route r1, const int &amt) {
    r1.positions.resize(r1.positions.size()-std::min((int)r1.positions.size(), amt));
    return r1;
}

knights::Route knights::init_route_from_sd(std::string route_name) {

    if (pros::usd::is_installed()) {
        route_name.insert(0, "/usd/");

        std::fstream read_file(route_name, std::ios_base::in);

        if (read_file) {
            std::vector<knights::Pos> positions;

            float x,y;

            while (read_file >> x && read_file >> y) {
                positions.emplace_back(x,y,0);

                printf("Pos: %lf %lf\n", x , y);
            }

            return knights::Route(positions);

        } else {
            return knights::Route();
        }
    } else {
        printf("SD card not found\n");
        return knights::Route();
    }
}

knights::AdvancedRoute advanced_route_from_file(std::string file_name) {
        if (pros::usd::is_installed()) {
        printf("Found SD card\n");
        file_name.insert(0, "/usd/");

        std::fstream read_file(file_name, std::ios_base::in);

        if (read_file) {
            std::vector<knights::RouteAction> ar_actions;
            std::map<std::string, knights::Route> ar_routes;

            std::string read_string;
            int route_amt = 0;
            while (read_file >> read_string) {
                std::string identifier; float x, y, z;
                if (read_string == "rs") { // follow route
                    // x and y are position points in route
                    // need to add route title
                    std::vector<knights::Pos> positions;
                    while (identifier != "re") {
                        read_file >> identifier;
                        if (identifier == "p") {
                            read_file >> x >> y;
                            positions.emplace_back(x, y, 0);
                        }
                    }
                    ar_actions.emplace_back(knights::action_type::FOLLOW, std::to_string(route_amt), 10.0, 7500);
                    ar_routes[std::to_string(route_amt)] = knights::Route(positions);
                    route_amt++;
                }
                else if (read_string == "ps") { // move for distance
                    // x = distance, y = end_tolerance, z = timeout
                    read_file >> x >> y >> z;
                    ar_actions.emplace_back(knights::action_type::LATERAL, x, y, z);
                }
                else if (read_string == "ts") { // turn to angle
                    // x = angle, y = end_tolerance, z = timeout
                    read_file >> x >> y >> z;
                    ar_actions.emplace_back(knights::action_type::TURN, x, y, z);
                }
                else if (read_string == "cs") { // command start
                    // logic for commands here
                }
                else if (read_string == "eof")
                    break;
	        }

            return knights::AdvancedRoute(ar_routes, ar_actions);

        } else {
            return knights::AdvancedRoute();
        }
    } else {
        printf("SD card not found\n");
        return knights::AdvancedRoute();
    }
}