#include <borealis_pages/borealis_smart_glove/smart_glove_client.h>

namespace smart_glove_client {

    SmartGloveClient::SmartGloveClient()
    {
        client = new SmartGloveClient::actionLibClient("smartglove", true);
        goal = new SmartGloveClient::msg_smartGloveServerGoal();
        client->waitForServer();
        
    };

    void SmartGloveClient::callServer()
    {
        
    };

    void SmartGloveClient::allocateStringSpace()
    {

    }


    
    void SmartGloveClient::scanForDevices()
    {   
        this->goal->goal.goal = "scan";
        this->goal->goal.goal_arg = "empty";
        this->callServer();
    };


    SmartGloveClient::~SmartGloveClient()
    {
        delete client;
        delete goal;
    };

}