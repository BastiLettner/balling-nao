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

    // Make sure NAO is not currently listening
    naoqi_bridge_msgs::SpeechWithFeedbackActionGoal msg_goal;
    msg_goal.goal_id.id = std::to_string(_vocab_id);
    msg_goal.goal.say = sentence;
    _vocab_id ++; // Increase id counter to guarantee uniqueness
    _speech_pub.publish(msg_goal);

    // Wait 100 milli seconds to make sure the action status publishing
    // is received and we can check the speech status
    ros::Duration(0, 500).sleep();
    ros::spinOnce();
    ros::Rate loop_rate(10);
    loop_rate.sleep();
    ros::spinOnce();
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
    LOG(std::to_string(_currently_speaking));
    while(_currently_speaking) {
        // Wait
        ros::spinOnce();
        loop_rate.sleep();
    }

    std::vector<std::string> vocab;  // Create the vocab
    get_words_from_sentences(available_sentences, vocab);  // Extract the words from the sentences

    publish_vocab(vocab);  // Publish the current vocab

    _matches.clear();  // Clear the vector so that the callback can fill it.

    std_srvs::Empty empty;
    if(_recog_start_srv.call(empty)) {

        LOG("Started Recording");
        int wait_iters = 0;
        while(wait_iters < recording_duration * 10) {
            // Wait
            ros::spinOnce();
            loop_rate.sleep();
            wait_iters ++;
            LOG("RECORDING");
        }
        // Collect the results. In the mean time the callback should have filled out the _matches

        // TODO: Catch else
        if( _recog_stop_srv.call(empty)) {
            LOG("Stopped Recording");
        }

        if(_matches.empty()) {
            throw CmdNotUnderstoodError("No word recognized. Maybe the recording duration was too short.", vocab, "");
        }

        else {

            for (const auto &match : _matches) {
                result.append(match);
                result.append(" ");
            }
            // Look if the sentence matches one of the available sentences
            for(const auto &sent: available_sentences) {
                if(result == sent) {
                    return;
                }
            }
            // If no match was found: exception
            LOG(result);
            throw CmdNotUnderstoodError(
                    "Nao recorded some words but there was no match to the requested sentences",
                    available_sentences,
                    result
                    );
        }
    }

    else {
        throw CmdNotUnderstoodError("Could not start Recording", vocab, "");
    }

}


void Speech::LOG(std::string msg) {
    std::cout << "[SPEECH] " << msg << std::endl;
}


void Speech::speech_recognition_callback(const naoqi_bridge_msgs::WordRecognized::ConstPtr &msg) {

    LOG("Speech recognition Callback");

    for (int i = 0; i < msg->words.size(); i++) {
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


void Speech::get_words_from_sentences(std::vector<std::string> &sentences, std::vector<std::string> &result) {

    assert(result.empty());

    std::vector<std::string> tmp_result;

    for(const std::string& sentence: sentences) {
        boost::split(tmp_result, sentence, boost::is_any_of(" "));
        for(const std::string& word: tmp_result) {

            result.push_back(word);  // Copied into the results
        }
        tmp_result.clear();
    }
}


void Speech::publish_vocab(std::vector<std::string> &vocab) {

    // Create the
    std_srvs::Empty empty;
    naoqi_bridge_msgs::SetSpeechVocabularyActionGoal msg_goal;
    msg_goal.goal_id.id = std::to_string(_vocab_id);
    msg_goal.goal.words = vocab;

    _vocab_pub.publish(msg_goal);  // Publish the currently available command

}


void Speech::speech_status(const actionlib_msgs::GoalStatusArray::ConstPtr& msg) {

    _currently_speaking = false;
    for(const auto& status: msg->status_list) {
        if(status.status == 1) {
            _currently_speaking = true;
        }
    }

}

void Speech::request_response_block(State *state, std::string &request, std::string &response) {

    while(true) {
        try {
            // First Nao says what he wants
            // The call blocks until he is finished
            talk(request);
            LOG("FINISHED TALKING");
            // Wait for answer. Create empty msg to fill.
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


