#include "knights/api.hpp"
#include "api.h"
#include "knights/autonomous/profile.hpp"

#include <fstream>
#include <string>

knights::Route::Route(std::vector<Pos> positions) {
    this->positions = positions;
}

knights::Route::Route() {
    this->positions = {};
}

float knights::Route::length_dist() {
    if (this->positions.size() < 2)
        return 0.0;

    float dist = 0.0;

    for (int i = 1; i < this->positions.size(); i++) {
        dist += distance_btwn(this->positions[i], this->positions[i-1]);
    }

    return dist;
}

knights::RouteAction::RouteAction(knights::action_type type, std::string route_name, float end_tolerance, int timeout, float lookahead) :
    type(type), route_name(route_name), end_tolerance(end_tolerance), timeout(timeout), lookahead(lookahead) {}

knights::RouteAction::RouteAction(action_type type, float specific, float end_tolerance, int timeout) :
    type(type), end_tolerance(end_tolerance), timeout(timeout), specific(specific) {}

knights::RouteAction::RouteAction(action_type type, float x, float y, float theta, float end_tolerance, int timeout) :
    type(type), end_tolerance(end_tolerance), timeout(timeout), move_to(Pos(x, y, theta)) {}

knights::RouteAction::RouteAction(action_type type, std::string function_name) :
    type(type), function_name(function_name) {}

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

knights::Route knights::init_route_from_asset(AssetStream &buffer) {
    if (buffer) {
        std::vector<knights::Pos> positions;

        float x,y,z;

        while (buffer >> x && buffer >> y && buffer >> z) {
            positions.emplace_back(x,y,z);
        }

        return knights::Route(positions);

    } else {
        return knights::Route();
    }
}

knights::Route knights::init_route_from_sd(std::string route_name) {

    if (pros::usd::is_installed()) {
        route_name.insert(0, "/usd/");

        std::fstream read_file(route_name, std::ios_base::in);

        if (read_file) {
            std::vector<knights::Pos> positions;

            float x,y,z;

            while (read_file >> x && read_file >> y && read_file >> z) {
                positions.emplace_back(x,y,z);
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
                    read_file >> x >> y >> z;
                    std::cout << x << y << z << "\n";
                    float end_tol = x; int timeout = y; float lookahead = z;
                    std::vector<knights::Pos> positions;
                    while (identifier != "re") {
                        read_file >> identifier;
                        if (identifier == "p") {
                            read_file >> x >> y >> z;
                            positions.emplace_back(x, y, z);
                        }
                    }
                    ar_actions.emplace_back(knights::action_type::FOLLOW, std::to_string(route_amt), end_tol, timeout, lookahead);
                    ar_routes[std::to_string(route_amt)] = knights::Route(positions);
                    route_amt++;
                }
                else if (read_string == "ps") { // move for distance
                    // x = distance, y = end_tolerance, z = timeout
                    read_file >> x >> y >> z;

                    knights::RouteAction new_action(knights::action_type::LATERAL, x, y, z);

                    ar_actions.push_back(new_action);

                    knights::logger::red(knights::logger::string_format("lateral: %lf %lf %lf", 
                        x, y, z));
                    // printf("lateral: %lf %lf %d\n", 
                    //         new_action.specific, new_action.end_tolerance, new_action.timeout);
                }
                else if (read_string == "ts") { // turn to angle
                    // x = angle, y = end_tolerance, z = timeout
                    read_file >> x >> y >> z;
                    ar_actions.emplace_back(knights::action_type::TURN, x, y, z);
                }
                else if (read_string == "lps") { // turn to angle
                    // x = angle, y = end_tolerance, z = timeout
                    float a,b;
                    read_file >> x >> y >> z >> a >> b;
                    ar_actions.emplace_back(knights::action_type::LATERAL_TO_POS, x, y, z, a, b);
                }
                else if (read_string == "cs") { // command start
                    // logic for commands here
                    read_file >> identifier;
                    ar_actions.emplace_back(knights::action_type::COMMAND, identifier);
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

knights::AdvancedRoute advanced_route_from_asset(AssetStream &buffer) {
    if (buffer) {
        std::vector<knights::RouteAction> ar_actions;
        std::map<std::string, knights::Route> ar_routes;

        std::string read_string;
        int route_amt = 0;
        while (buffer >> read_string) {
            std::string identifier; float x, y, z;
            if (read_string == "rs") { // follow route
                // x and y are position points in route
                // need to add route title
                buffer >> x >> y >> z;
                std::cout << x << y << z << "\n";
                float end_tol = x; int timeout = y; float lookahead = z;
                std::vector<knights::Pos> positions;
                while (identifier != "re") {
                    buffer >> identifier;
                    if (identifier == "p") {
                        buffer >> x >> y >> z;
                        positions.emplace_back(x, y, z);
                    }
                }
                ar_actions.emplace_back(knights::action_type::FOLLOW, std::to_string(route_amt), end_tol, timeout, lookahead);
                ar_routes[std::to_string(route_amt)] = knights::Route(positions);
                route_amt++;
            }
            else if (read_string == "ps") { // move for distance
                // x = distance, y = end_tolerance, z = timeout
                buffer >> x >> y >> z;

                knights::RouteAction new_action(knights::action_type::LATERAL, x, y, z);

                ar_actions.push_back(new_action);

                knights::logger::red(knights::logger::string_format("lateral: %lf %lf %lf", 
                    x, y, z));
                // printf("lateral: %lf %lf %d\n", 
                //         new_action.specific, new_action.end_tolerance, new_action.timeout);
            }
            else if (read_string == "ts") { // turn to angle
                // x = angle, y = end_tolerance, z = timeout
                buffer >> x >> y >> z;
                ar_actions.emplace_back(knights::action_type::TURN, x, y, z);
            }
            else if (read_string == "lps") { // turn to angle
                // x = angle, y = end_tolerance, z = timeout
                float a,b;
                buffer >> x >> y >> z >> a >> b;
                ar_actions.emplace_back(knights::action_type::LATERAL_TO_POS, x, y, z, a, b);
            }
            else if (read_string == "cs") { // command start
                // logic for commands here
                buffer >> identifier;
                ar_actions.emplace_back(knights::action_type::COMMAND, identifier);
            }
            else if (read_string == "eof")
                break;
        }

        return knights::AdvancedRoute(ar_routes, ar_actions);

    } else {
        return knights::AdvancedRoute();
    }
}

void knights::AdvancedRoute::execute(knights::RobotChassis *chassis, knights::RobotController *robotControl, knights::input::AutonomousInputMap *input_map) {
    
    for (RouteAction curr_action : this->actions) {
        if (curr_action.type == knights::action_type::LATERAL) {
            robotControl->lateral_move(curr_action.specific, curr_action.end_tolerance, curr_action.timeout);
            knights::logger::red(knights::logger::string_format("lateral %lf", curr_action.specific));
        }
        else if (curr_action.type == knights::action_type::TURN) {
            robotControl->turn_to_angle(curr_action.specific, 0,curr_action.end_tolerance, curr_action.timeout, true);
            knights::logger::green(knights::logger::string_format("turn %lf", curr_action.specific));
        }
        else if (curr_action.type == knights::action_type::FOLLOW && this->routes.contains(curr_action.route_name)) {
            robotControl->follow_route_pursuit(
                this->routes[curr_action.route_name], 
                curr_action.lookahead, 
                robotControl->lateral_pid->get_max_speed(), 
                true,
                curr_action.end_tolerance, 
                curr_action.timeout
            );
            knights::logger::cyan(knights::logger::string_format("follow: %s , pos: %lf %lf %lf , error: %lf", curr_action.route_name.c_str(), 
                chassis->get_position().x, chassis->get_position().y, chassis->get_position().heading, 
                knights::distance_btwn(chassis->get_position(), this->routes[curr_action.route_name].positions.back())));
            // for (knights::Pos pos : this->routes[curr_action.route_name].positions) {
            //     // knights::logger::yellow(knights::logger::string_format("p: %lf %lf %lf", pos.x, pos.y, pos.heading));
            // }
        }
        else if (curr_action.type == knights::action_type::LATERAL_TO_POS) {
            int dir = (curr_action.end_tolerance >= 0) ? 1 : -1;
            robotControl->lateral_to_position(curr_action.move_to, dir, fabs(curr_action.end_tolerance), curr_action.timeout);
        }
        else if (curr_action.type == knights::action_type::COMMAND) {
            input_map->execute_action(curr_action.function_name);
            knights::logger::blue(knights::logger::string_format("command %s", curr_action.function_name.c_str()));
            pros::delay(100);
        }
        pros::delay(200);
    }
}