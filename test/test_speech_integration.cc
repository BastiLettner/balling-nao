//
// Created by hrs_b on 15.01.19.
//

#include "../src/cc/sensors_actors/speech.h"
#include "../src/cc/errors/cmd_not_understood_error.h"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "speech_test");
    ros::NodeHandle nh;
    Speech speech(nh);  // Create speech module

    std::string answer;
    std::vector<std::string> vocab = {"no", "yes"};  // The vocab to use

    ros::Rate loop_rate(10);

    int warmup = 10;  // makes everything smother
    while (warmup > 0) {
        ros::spinOnce();
        loop_rate.sleep();
        warmup--;
    }


    ros::spinOnce();
    speech.talk("can i have the ball");  // State command
    try {
        speech.listen(vocab, answer, 3);  // record answer
    }
    catch (CmdNotUnderstoodError& e) {
        std::cout << e.what() << std::endl;
        std::cout << "Failed" << std::endl;
    }
    loop_rate.sleep();
    std::cout << answer << std::endl;


}


