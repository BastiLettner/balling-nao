//
// Created by hrs_b on 10.01.19.
//

#ifndef BALLING_NAO_SPEECH_H
#define BALLING_NAO_SPEECH_H

#include <ros/node_handle.h>
#include <naoqi_bridge_msgs/WordRecognized.h>
#include <actionlib_msgs/GoalStatusArray.h>

class State;


class Speech {

    // Speech module serves as an API to let NAO talk and listen to commands.
    // The main functions are listen() and talk().

public:
    // Constructs object.
    // - advertises "/speech_action/goal"
    // - advertises "/speech_vocabulary_action/goal"
    // - service "/start_recognition"
    // - service "/stop_recognition"
    // - subscription "/word_recognized"
    explicit Speech(ros::NodeHandle& node_handle);
    ~Speech() = default;

    // Prohibit Copying the Speech Module.
    Speech(Speech&& speech) = delete;

    // Make NAO say a sentence.
    // Call blocks until Nao finished speaking
    //
    // Args:
    //     sentence: The sentence Nao should say
    //
    // Returns:
    //
    void talk(std::string sentence);

    // Make a recording.
    // If Nao is currently speaking this function waits until he is done.
    // If there was no match this function throws a CmdNotUnderstoodError containing useful information
    //
    // Args:
    //     available_sentences: The available sentences for this call.
    //     results: Will be filled with the results. Won't be empty if there is no exception.
    //     record_duration: The duration of the recording in seconds
    //
    void listen(std::vector<std::string>& available_sentences, std::string& results, uint32_t record_duration);

    // Callback for speech recognition
    // If the results have the required confidence this function puts them into the _matches vector.
    void speech_recognition_callback(const naoqi_bridge_msgs::WordRecognized::ConstPtr& msg);

    // Prints a log message
    void LOG(std::string msg);

    // Publishes the vocab
    //
    // Args:
    //     vocab: List of available words. Words can be contained twice
    //
    void publish_vocab(std::vector<std::string>& vocab);

    // Call for checking if Nao is currently speaking
    // Stores value in the _currently_speaking variable.
    void speech_status(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);

    // Implements a request response block for a dialog with nao
    // Nao states a request and expects the user to answer.
    // Nao will speak out the request. After speaking he starts listening to the user.
    // Then he compares what he recorded to the expected answers.
    // If the answer Nao recorded does not match any of the expected answers (with more than 40% confidence)
    // Nao will say that he did not understand and then ask again.
    // This goes on until Nao recorded an expected answer which will then be written in the response parameter
    //
    // Args:
    //     state: The state from where the function is called
    //     request: The request nao will make (question asked)
    //     response: Will contain the command of the user
    //
    void request_response_block(State* state, std::string& request, std::string& response);

private:

    ros::Publisher _speech_pub;

    ros::Publisher _vocab_pub;

    ros::ServiceClient _recog_start_srv;

    ros::ServiceClient _recog_stop_srv;

    ros::Subscriber _speech_recog_sub;

    ros::Subscriber _speech_status;

    uint32_t _vocab_id = 0;

    std::vector<std::string> _matches;

    bool _currently_speaking = false;

};
#endif //BALLING_NAO_SPEECH_H
