#pragma once
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <stretchsense_server_action_msgs/smartGloveServerAction.h>
#include <stretchsense_server_action_msgs/smartGloveServerActionGoal.h>
#include <vector>

namespace smart_glove_client {
    class SmartGloveClient {

        private: // typedefs
            typedef actionlib::SimpleActionClient<stretchsense_server_action_msgs::smartGloveServerAction> actionLibClient;
            typedef stretchsense_server_action_msgs::smartGloveServerResult msg_smartGloveServerResult;
            typedef stretchsense_server_action_msgs::smartGloveServerActionGoal msg_smartGloveServerGoal;
        
        public:
            SmartGloveClient();
            ~SmartGloveClient();

            void callServer();
            void scanForDevices();
            void connectToPeripheral();
            void selectModel();
            void calibrate();
            void stopServerThread();
            void disconnectPeripheral();
            void allocateStringSpace();

            actionLibClient* client;
            msg_smartGloveServerGoal* goal;

            std::string* action[2];
    };
}