#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "Executioner.h"
#include "common/CallbackHandler.hpp"
#include "Parser.h"


int main(int argc, char** argv){



    Parser p;
    Executioner e;

    p.loadFile("lists/list.txt");
    std::cout << "Parsing ....." << std::endl;

    if(p.parse()){
        std::cout << ".......Parsed" << std::endl;
        e.setList(p.getTaskListParsed());
    }else{

        std::cout << "Unable to parse the file" << std::endl;

    }
    e.init();
    lcm::LCM handler;

    if (!handler.good())
        return 1;
    CallbackHandler call;


    handler.subscribe("vision_position_estimate", &CallbackHandler::visionEstimateCallback, &call);

    while(0==handler.handle()){



            //AndOr

            //Run state machine
            e.setState(call._vision_pos);
            e.run();
            //Publish next task
            if(e.readyToPublish()) {

                std::cout << "publishing task" << std::endl;
                handler.publish("actual_task", &e._actualTask);

            }

    }

}