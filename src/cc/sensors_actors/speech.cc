//
// Created by hrs_b on 10.01.19.
//

#include "speech.h"
#include <naoqi_bridge_msgs/SpeechWithFeedbackActionGoal.h>
#include <naoqi_bridge_msgs/SetSpeechVocabularyActionGoal.h>
#include <std_srvs/Empty.h>
#include <unistd.h>
#include "../errors/cmd_not_understood_error.h"
#include <assert.h>
#include <boost/algorithm/string.hpp>
#include "../states/state.h"
#include <thread>


Speech::Speech(ros::NodeHandle& node_handle) {
    // Create all nodes.
    _speech_pub = node_handle.advertise<naoqi_bridge_msgs::SpeechWithFeedbackActionGoal>("/speech_action/goal", 1);
    _vocab_pub = node_handle.advertise<naoqi_bridge_msgs::SetSpeechVocabularyActionGoal>(
            "/speech_vocabulary_action/goal", 1);
    _recog_start_srv = node_handle.serviceClient<std_srvs::Empty>("/start_recognition");
    _recog_stop_srv = node_handle.serviceClient<std_srvs::Empty>("/stop_recognition");
    _speech_recog_sub = node_handle.subscribe("/word_recognized", 1, &Speech::speech_recognition_callback, this);
    _speech_status = node_handle.subscribe("/speech_action/status", 1, &Speech::speech_status, this);
}


void Speech::talk(std::string sentence) {

    LOG("Talking: " + sentence);
    std_srvs::Empty empty;  // Create Empty msg for stop record service

    naoqi_bridge_msgs::SpeechWithFeedbackActionGoal msg_goal;
    msg_goal.goal_id.id = std::to_string(_vocab_id);
    msg_goal.goal.say = sentence;
    _vocab_id ++; // Increase id counter to guarantee uniqueness
    // Todo: commented out for debugging, uncomment!
    //_speech_pub.publish(msg_goal);

    // Wait 500 milli seconds to make sure the action status publishing
    // is received and we can check the speech status
    ros::Rate loop_rate(10);
    int i = 0;
    while(i < 5) {
        ros::spinOnce();
        loop_rate.sleep();
        i++;
    }
    // Wait until speaking is done
    while(_currently_speaking) {
        // Wait
        ros::spinOnce();
        loop_rate.sleep();

    }

}


void Speech::listen(
        std::vector<std::string> &available_sentences,
        std::string& result,
        uint32_t recording_duration
        ) {

    LOG("Listening");

    assert(result.empty());  // Make sure the result is empty now.

    ros::Rate loop_rate(10);
    // Check if Nao is currently speaking
    while(_currently_speaking) {
        // Wait
        ros::spinOnce();
        loop_rate.sleep();
    }

    publish_vocab(available_sentences);  // Publish the current vocab

    _matches.clear();  // Clear the vector so that the speech recognition callback can fill it.

    std_srvs::Empty empty;
    if(_recog_start_srv.call(empty)) {

        LOG("Started Recording");
        int wait_iters = 0;
        // Wait the specified duration
        // In the mean time the callback should have filled out the _matches
        while(wait_iters < recording_duration * 10) {
            // Wait
            ros::spinOnce();
            loop_rate.sleep();
            wait_iters ++;
        }

        // TODO: Catch else
        // Stop recording
        if( _recog_stop_srv.call(empty)) {
            LOG("Stopped Recording");
        }

        // Check results
        if(_matches.empty()) {
            throw CmdNotUnderstoodError("No word recognized. Maybe the recording duration was too short.", available_sentences, "");
        }

        else {
            // Constructs sentence
            for (const auto &match : _matches) {
                result.append(match);
                result.append(" ");
            }
            LOG(result);
            // Confirm understood command.
            talk("i understood " + result);
            return;
        }
    }

    else {
        throw CmdNotUnderstoodError("Could not start Recording", available_sentences, "");
    }

}


void Speech::LOG(std::string msg) {
    std::cout << "[SPEECH] " << msg << std::endl;
}


void Speech::speech_recognition_callback(const naoqi_bridge_msgs::WordRecognized::ConstPtr &msg) {

    LOG("Speech recognition Callback");

    for (int i = 0; i < msg->words.size(); i++) {

        // Check confidence values
        if (msg->confidence_values[i] > 0.4f) {
            std::string log_msg = "MATCHED INPUT TO WORD " +
                    msg->words[i] + " WITH " + std::to_string(int(100*msg->confidence_values[i])) + "% CONFIDENCE";
            LOG(log_msg);
            _matches.push_back(msg->words[i]);
        }
        else{
            std::string log_msg = "MATCHED INPUT TO WORD " +
                    msg->words[i] + " WITH " + std::to_string(int(100*msg->confidence_values[i])) + "% CONFIDENCE";
            LOG(log_msg);
        }
    }

}


void Speech::publish_vocab(std::vector<std::string> &vocab) {

    // Create emtpy message
    std_srvs::Empty empty;
    naoqi_bridge_msgs::SetSpeechVocabularyActionGoal msg_goal;
    msg_goal.goal_id.id = std::to_string(_vocab_id);
    msg_goal.goal.words = vocab;
    ros::Rate loop_rate(100);
    for(size_t i = 0; i < 10; i++) {
        _vocab_pub.publish(msg_goal);  // Publish the currently available command(s)
        ros::spinOnce();
        loop_rate.sleep();
    }

}


void Speech::speech_status(const actionlib_msgs::GoalStatusArray::ConstPtr& msg) {

    // Check if there is a entry in the status list which contains a status value of on
    // indicating that Nao is currently busy speaking
    _currently_speaking = false;

    for(const auto& status: msg->status_list) {
        if(status.status == 1/* or status.status == 3*/) {
            _currently_speaking = true;
        }
    }

}

void Speech::request_response_block(State *state, std::string &request, std::string &response) {

    while(true) {
        try {
            // First Nao make the request
            // The call blocks until he is finished talking
            talk(request);
            // Wait for the answer. Fills the response parameter is the command was understood
            // Now we listen
            listen(state->get_available_cmds(), response, 3);
        }
        catch(CmdNotUnderstoodError& e) {
            LOG(e.what());  // Dump the information to commandline
            e.handle(*this, state);  // Handle the exception.
            response.clear();  // Since the response was unexpected, discard it and make it usable for the next try
            continue;  // try_again
        }
        break;

    }

}


