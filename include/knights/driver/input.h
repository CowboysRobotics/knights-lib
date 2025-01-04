#pragma once

#ifndef _INPUT_H
#define _INPUT_H

#include <map>
#include <vector>

#include "api.h"

namespace knights::input {
    
    class InputAction {
        private:
            void (*bound_function)();
        public:
            /**
             * @brief Controller button to associate with action
             * 
             */
            pros::controller_digital_e_t button;

            /**
             * @brief Should the action be executed on hold
             * 
             */
            bool hold;

            /**
             * @brief Construct a new Input Action
             * 
             * @param button Controller button to associate with the action
             * @param bound_function Function to execute on the controller button press
             * @param hold Should the action be executed on hold
             */
            InputAction(pros::controller_digital_e_t button, void (*bound_function)(), bool hold);

            /**
             * @brief Run associated function with the action
             * 
             */
            void run_function();
    };

    class InputMap {
        private:
            std::vector<InputAction> action_list;
        public:
            /**
             * @brief Add an action to the input map
             * 
             * @param button Controller button to associate with the action
             * @param bound_function Function to execute on the controller button press
             * @param hold Should the action be executed on hold
             */
            void bind_action(pros::controller_digital_e_t button, void (*bound_function)(), bool hold);

            /**
             * @brief Determine which actions to run based on controller input
             * 
             * @param controller Controller to query for button presses
             */
            void execute_actions(pros::Controller controller);

    };


    class AutonomousInputMap {
        private:
            std::map<std::string, void(*)()> action_map;
        public:
            /**
             * @brief Add an action to the input map
             * 
             * @param bound_function Function to execute on the controller button press
             * @param name Key to add the function under
             */
            void bind_action(std::string name, void (*bound_function)());

            /**
             * @brief Run an action within input map
             * 
             * @param name Key of function to run
             */
            void execute_action(std::string name);
    };
}

#endif