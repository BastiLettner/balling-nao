//
// Created by hrs_b on 15.01.19.
//

#include "../src/cc/modules/speech.h"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "speech_test");
    ros::NodeHandle nh;
    Speech speech(nh);

    std::string answer;
    std::vector<std::string> vocab = {"no thanks", "yes please "};

    ros::Rate loop_rate(10);


    while(ros::ok()) {

        ros::spinOnce();
        speech.talk("Hello I am Nao");
        loop_rate.sleep();

    }

//    while(ros::ok()) {
//
//        ros::spinOnce();
//        speech.listen(vocab, answer, 4);
//        std::cout << "Answer " << answer << std::endl;
//        loop_rate.sleep();
//        answer = "";
//
//    }


}


