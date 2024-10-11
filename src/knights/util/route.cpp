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

knights::RouteAction::RouteAction(knights::ACTION_TYPE type, std::string route_name, float end_tolerance, int timeout) :
    type(type), route_name(route_name), end_tolerance(end_tolerance), timeout(timeout) {}

knights::RouteAction::RouteAction(ACTION_TYPE type, float specific, float end_tolerance, int timeout) :
    type(type), end_tolerance(end_tolerance), timeout(timeout) {}

knights::RouteAction::RouteAction(ACTION_TYPE type, void (*bound_function)()) :
    type(type), bound_function(bound_function) {}


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
        printf("Found SD card\n");
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
            std::vector<knights::Pos> positions;

            float x,y;

            while (read_file >> x && read_file >> y) {
                positions.emplace_back(x,y,0);

                printf("Pos: %lf %lf\n", x , y);
            }

            return knights::AdvancedRoute();

        } else {
            return knights::AdvancedRoute();
        }
    } else {
        printf("SD card not found\n");
        return knights::AdvancedRoute();
    }
}


// int main() {
// 	string s;
// 	while (cin >> s) {
// 		char v; int x,y,z;
// 		cout << s << endl;
// 		if (s == "rs") {
// 			while (cin >> v) {
// 				if (strcmp(&v, "p") == 0) {
// 					cin >> x >> y;
// 					cout << v << x << y << endl;
// 				}
// 				else {
// 					break;
// 				}
// 			}
// 			cout << "end" << endl;
// 		}
// 		if (s == "ps") {
// 			cin >> x >> y >> z;
// 			cout << "e" << x << y << z << endl;
// 		}
// 	}
// 	cout << "yes" << endl;
// }