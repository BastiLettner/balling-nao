// Tactile class header
// The tactile module retrieves the state of the capacitive buttons on NAO's head
// stucture : single class

//
// Created by hrs_b on 14.01.19.
//

#ifndef BALLING_NAO_TACTILE_H
#define BALLING_NAO_TACTILE_H

#include <ros/node_handle.h>
#include <naoqi_bridge_msgs/HeadTouch.h>

// States of a button
enum ButtonStates {
        RELEASED,
        WAS_PRESSED,
};

// Holds the state for the three buttons on NAOs head
struct ButtonTracker {

    std::map<std::string, ButtonStates> buttons;

    ButtonTracker() {
        buttons["front"] = ButtonStates::RELEASED;
        buttons["rear"] = ButtonStates::RELEASED;
        buttons["middle"] = ButtonStates::RELEASED;
    }

};


class Tactile {

    // Can detect if a button was pressed

public:

    // Constructs object.
    // - Subscribes to "/tactile_though"  (roslaunch nao_apps tactile.launch)
    explicit Tactile(ros::NodeHandle& node_handle);
    ~Tactile() = default;

    // Prohibit copying of Tactile.
    Tactile(Tactile&& tactile) = delete;

    // Function prints log messages
    void LOG(std::string message) { std::cout << "[TACTILE]: " << message << std::endl; }

    // Callback for keeping track of the button states
    // The button states are collected in the ButtonTracker structure
    // and updated by this function.
    // When a button is pressed the function simply sets its state to WAS_PRESSED.
    // The useful logic is implemented in the detect_button_pressed function.
    // The topic publishes only when a button is pressed.
    void update_button_tracker(const naoqi_bridge_msgs::HeadTouch::ConstPtr& tactile_state);

    // This function can detect if a button is pressed after the call of this function.
    // It blocks as long as the button has not been pressed
    //
    // Args:
    //     button_name: The name of the button. Can be 'rear', 'front', 'middle'
    void detect_button_pressed(std::string button_name);


private:

    ros::Subscriber _tactile_head;

    ButtonTracker _button_tracker;

};

#endif //BALLING_NAO_TACTILE_H
