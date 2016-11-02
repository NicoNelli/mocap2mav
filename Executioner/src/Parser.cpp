//
// Created by andreanisti on 27/10/16.
//

#include "Parser.h"



Parser::Parser() {



}

bool Parser::loadFile(std::string file) {

    std::ifstream myfile (file);
    if (myfile.is_open()) {
        //Clear token vector
        _tokens.clear();
        // Why not?
        _tokens.shrink_to_fit();

        //Build tokenized vector line by line
        std::string line;
        while ( getline (myfile,line)){

            std::vector<std::string> str_vector;
            strtk::parse(line,"=",str_vector);

            _tokens.push_back(str_vector);

        }
/*
        for (int i = 0; i < _tokens.size()-1; ++i) {

            std::cout<<_tokens[i][0] << " " << _tokens[i][1] << std::endl;

        }
*/
        myfile.close();
        return true;
    }
    else{

        std::cout << "Unable to open file" << std::endl;
        return false;


    }


}

bool Parser::parse() {

    //Save type position
    std::vector<int> type_pos;

    for (int i = 0; i < _tokens.size(); ++i) {

        if(_tokens[i][0] == "type") type_pos.push_back(i);

    }
    for (int k = 0; k < type_pos.size(); ++k) {
        std::cout << type_pos[k] << std::endl;
    }

    //Start Parsing
    if(type_pos.size() > 0) {


        for (int j = 0; j < type_pos.size(); ++j) {

            //Store the position in the token vector of the value "type"
            int pos = type_pos[j];
            std::string action = _tokens[pos][1];
            //
            if(!parseAction(action,pos)) return false;

        }

    } else {

        std::cout << "Invalid tokens count" << std::endl;
        return false;

    }
    return true;

}

bool Parser::parseAction(std::string a, int pos) {

    //create task
    exec::task task;

    //Select the right action and parse it
    if(a == "takeoff"){
        std::cout << "Takeoff found" << std::endl;
        bool zFound = false;
        task.action = common::actions::TAKE_OFF;

        for (int i = pos+1; i < _tokens.size() && _tokens[i][0] != "type" ; ++i) {

            std::string field = _tokens[i][0];

            if(field == "z"){

                //atof Helper, cast string into double
                char * cstr = new char [_tokens[i][1].length()+1];
                std::strcpy (cstr, _tokens[i][1].c_str());
                double value = atof(cstr);
                delete(cstr);

                if(std::isfinite(value)){
                    zFound = true;
                    task.z = value;
                    std::cout << "Z found, take off parsed: " << value << std::endl;

                }else{
                    std::cout << "value is not finite" << std::endl;
                    return false;
                }


            }else{

                std::cout << "unrecognized field" << std::endl;
                return false;

            }

        }
        if(zFound) _taskListParsed.push_back(task);


    }
    else if(a == "move"){
        std::cout << "Move found" << std::endl;
        const unsigned char xFound = 0x01; // hex for 0000 0001
        const unsigned char yFound = 0x02; // hex for 0000 0010
        const unsigned char zFound = 0x04; // hex for 0000 0100
        const unsigned char aFound = 0x08; // hex for 0000 1000
        unsigned char mask = 0;
        task.action = common::actions::MOVE;

        for (int i = pos+1; i < _tokens.size() && _tokens[i][0] != "type" ; ++i) {

            std::string field = _tokens[i][0];

            if(field == "x") {

                //atof Helper, cast string into double
                char *cstr = new char[_tokens[i][1].length() + 1];
                std::strcpy(cstr, _tokens[i][1].c_str());
                double value = atof(cstr);
                delete (cstr);

                if (std::isfinite(value)) {
                    mask |= xFound;
                    task.x = value;
                    std::cout << "X: " <<value<< " found for action: " << pos << std::endl;

                } else {
                    std::cout << "value is not finite" << std::endl;
                    return false;
                }

            }
            else if(field == "y"){

                //atof Helper, cast string into double
                char *cstr = new char[_tokens[i][1].length() + 1];
                std::strcpy(cstr, _tokens[i][1].c_str());
                double value = atof(cstr);
                delete (cstr);

                if (std::isfinite(value)) {
                    mask |= yFound;
                    task.y = value;
                    std::cout << "Y: " <<value<< " found for action: " << pos << std::endl;

                } else {
                    std::cout << "value is not finite" << std::endl;
                    return false;
                }


            }
            else if(field == "z"){

                //atof Helper, cast string into double
                char *cstr = new char[_tokens[i][1].length() + 1];
                std::strcpy(cstr, _tokens[i][1].c_str());
                double value = atof(cstr);
                delete (cstr);

                if (std::isfinite(value)) {
                    mask |= zFound;
                    task.z = value;
                    std::cout << "Z: " <<value<< " found for action: " << pos << std::endl;

                } else {
                    std::cout << "value is not finite" << std::endl;
                    return false;
                }

            }
            else if(field == "alpha"){

                //atof Helper, cast string into double
                char *cstr = new char[_tokens[i][1].length() + 1];
                std::strcpy(cstr, _tokens[i][1].c_str());
                double value = atof(cstr);
                delete (cstr);

                if (std::isfinite(value)) {
                    mask |= aFound;
                    task.params[0] = value;
                    std::cout << "Alpha: " <<value<< " found for action: " << pos << std::endl;

                } else {
                    std::cout << "value is not finite" << std::endl;
                    return false;
                }

            }

            else{

                std::cout << "unrecognized field" << std::endl;

            }

        }
        if(mask & xFound & yFound & zFound & aFound) _taskListParsed.push_back(task);
        else {
            std::cout << "field missing in action: " << pos << std::endl;
            return false;
        }


    }
    else if (a == "rotate") {
        //TODO: rotate is bugged, do the parsing when bu is fixed.


    }
    else if (a == "land"){
        std::cout << "Land found" << std::endl;
        bool hFound = false;
        task.action = common::actions::LAND;
        double height = 0;

        for (int i = pos+1;  i < _tokens.size() && _tokens[i][0] != "type" ; ++i) {
            std::cout << i << std::endl;
            std::string field = _tokens[i][0];

            if(field == "height"){

                //atof Helper, cast string into double
                char * cstr = new char [_tokens[i][1].length()+1];
                std::strcpy (cstr, _tokens[i][1].c_str());
                double value = atof(cstr);
                delete(cstr);

                if(std::isfinite(value)){
                    hFound = true;
                    task.params[0] = value;
                    std::cout << "H found, land parsed: " << value << std::endl;

                }else{
                    std::cout << "value is not finite" << std::endl;
                    return false;
                }


            }else{

                std::cout << "unrecognized field" << std::endl;
                return false;

            }



        }
        if(hFound) _taskListParsed.push_back(task);
        else{
            task.params[0] = 0;
            _taskListParsed.push_back(task);

        }



    }else {
        std::cout << "Unrecognised type at position: "<< pos << std::endl;
        return false;
    }
    //No failures at this point, return true
    return true;

}

