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


Speech::Speech(ros::NodeHandle& node_handle) {
    _speech_pub = node_handle.advertise<naoqi_bridge_msgs::SpeechWithFeedbackActionGoal>("/speech_action/goal", 1);
    _vocab_pub = node_handle.advertise<naoqi_bridge_msgs::SetSpeechVocabularyActionGoal>(
            "/speech_vocabulary_action/goal", 1);
    _recog_start_srv = node_handle.serviceClient<std_srvs::Empty>("/start_recognition");
    _recog_stop_srv = node_handle.serviceClient<std_srvs::Empty>("/stop_recognition");
    _speech_recog_sub = node_handle.subscribe("/word_recognized", 1, &Speech::speech_recognition_callback, this);
}


bool Speech::talk(std::string sentence) {

    LOG("Talking: " + sentence);
    std_srvs::Empty empty;  // Create Empty msg for stop record service

    // Make sure NAO is not currently listening
    if(_recog_stop_srv.call(empty)) {
        naoqi_bridge_msgs::SpeechWithFeedbackActionGoal msg_goal;
        msg_goal.goal_id.id = std::to_string(_vocab_id);
        msg_goal.goal.say = sentence;
        _vocab_id ++; // Increase id counter to guarantee uniqueness
        _speech_pub.publish(msg_goal);
        return true;
    }
    else {
        LOG("Stop record call was unsuccessful");
        return false;
    }

}


void Speech::listen(
        std::vector<std::string> &available_sentences,
        std::string& result,
        uint32_t recording_duration) {

    LOG("Listening");

    assert(result.empty());  // Make sure the result is empty now.

    std::vector<std::string> vocab;  // Create the vocab
    get_words_from_sentences(available_sentences, vocab);  // Extract the words from the sentences

    publish_vocab(vocab);  // Publish the current vocab

    _matches.clear();  // Clear the vector so that the callback can fill it.

    std_srvs::Empty empty;
    if(_recog_start_srv.call(empty)) {

        LOG("Started Recording");
        sleep(recording_duration);  // Wait for the user to say something.
        // Collect the results. In the mean time the callback should have filled out the _matches

        if(_matches.empty()) {
            throw CmdNotUnderstoodError("No word recognized. Maybe the recording duration was too short.", vocab, "");
        }

        else {

            std::string sentence;
            for (const auto &match : _matches) {
                sentence.append(match);
                sentence.append(" ");
            }
            // Look if the sentence matches one of the available sentences
            for(const auto &sent: available_sentences) {
                if(sentence == sent) {
                    result = sentence;
                }
            }
            // If no mathc was found: exception
            if(result.empty()) {
                throw CmdNotUnderstoodError(
                        "Nao recorded some words but there was no match to the requested sentences",
                        available_sentences,
                        sentence
                        );
            }
        }
        // TODO: Catch else
        if( _recog_stop_srv.call(empty)) {
            LOG("Stopped Recording");
        }
    }  // End if

    else {
        throw CmdNotUnderstoodError("Could not start Recording", vocab, "");
    }

}


void Speech::LOG(std::string msg) {
    std::cout << "[SPEECH] " << msg << std::endl;
}


void Speech::speech_recognition_callback(const naoqi_bridge_msgs::WordRecognized::ConstPtr &msg) {

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
            tmp_result.clear();
        }
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


