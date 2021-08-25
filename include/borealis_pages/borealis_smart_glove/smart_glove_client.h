#pragma once
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <stretchsense/smartGloveServerAction.h>

namespace smart_glove_client {
    class SmartGloveClient {
        
        public:
            SmartGloveClient();
    };
}