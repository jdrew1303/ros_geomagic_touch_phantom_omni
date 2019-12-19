#pragma once

#include<string>

class Link {
    public:
        std::string name;
        Link(const std::string link_name){
            name = link_name;
        }
};