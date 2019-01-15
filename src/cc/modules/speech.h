//
// Created by hrs_b on 10.01.19.
//

#ifndef BALLING_NAO_SPEECH_H
#define BALLING_NAO_SPEECH_H

#include <ros/node_handle.h>
#include <naoqi_bridge_msgs/WordRecognized.h>


class Speech {

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
    //
    // Args:
    //     sentence: The sentence Nao should say
    //
    // Returns:
    //     true: The call was successful
    //     false: The call was not successful and NAO did not say the sentence.
    //
    bool talk(std::string sentence);

    // Make a recording.
    // If there was no match this function throws a CmdNotUnderstoodError containing useful information
    // Args:
    //     available_sentences: The available sentences for this call. Will be split by space into words.
    //     results: Will be filled with the results. Won't be empty if there is no exception.
    //     record_duration: The duration of the recording in seconds
    //
    void listen(std::vector<std::string>& available_sentences, std::string& results, uint32_t record_duration);

    // Callback for speech recognition
    // If the results have the reqired confidence this function puts them into the matches.
    void speech_recognition_callback(const naoqi_bridge_msgs::WordRecognized::ConstPtr& msg);

    // Prints a log message
    void LOG(std::string msg);

    // Takes a list of sentences and extracts the contained workds from them.
    // The words won't necessarily be unique.
    //
    // Args:
    //     sentences: List of sentences
    //     result: The result will be stored in this vector. Is emtpy at call.
    //
    void get_words_from_sentences(std::vector<std::string> &sentences, std::vector<std::string>& result);

    // Publishes the vocab
    //
    // Args:
    //     vocab: List of available words
    //
    void publish_vocab(std::vector<std::string>& vocab);

private:

    ros::Publisher _speech_pub;

    ros::Publisher _vocab_pub;

    ros::ServiceClient _recog_start_srv;

    ros::ServiceClient _recog_stop_srv;

    ros::Subscriber _speech_recog_sub;

    uint32_t _vocab_id = 0;

    std::vector<std::string> _matches;

};
#endif //BALLING_NAO_SPEECH_H
