#include "knights/driver/input.h"

#include "api.h"

knights::input::InputAction::InputAction(pros::controller_digital_e_t button, void (*bound_function)(), bool hold) 
    : button(button), bound_function(bound_function), hold(hold) {
}

void knights::input::InputAction::run_function() {
    this->bound_function();
}

void knights::input::InputMap::bind_action(pros::controller_digital_e_t button, void (*bound_function)(), bool hold) {
    this->action_list.emplace_back(button, bound_function, hold);
}

void knights::input::InputMap::execute_actions(pros::Controller controller) {
    for (InputAction action : this->action_list) { // For action added to map
        if (controller.get_digital_new_press(action.button)) { // If button just pressed
            action.run_function();
        } else if (controller.get_digital(action.button) && action.hold) { // If button held and set to hold
            action.run_function();
        }
    }
}

void knights::input::AutonomousInputMap::bind_action(std::string name, void (*bound_function)()) {
    if (!this->action_map.contains(name)) {
        this->action_map[name] = bound_function;
    }
}

void knights::input::AutonomousInputMap::execute_action(std::string name) {
    if (this->action_map.contains(name)) {
        this->action_map[name]();
    }
}