#include <borealis_pages/borealis_map_and_drone/borealis_map_and_drone.h>
#include <image_frame/image_interactive_layout.h>

namespace borealis_map_and_drone {

    BorealisMapAndDrone::BorealisMapAndDrone(ros::NodeHandlePtr rosNode)
    : node(rosNode)
    {
        this->setEnabled(true);
        this->setGeometry(0,0,600,600);

        horizontalLayout = new QHBoxLayout(this);
        verticalLayout = new QVBoxLayout();

        droneVideoLayout();

        verticalLayout->setMargin(10);
        horizontalLayout->addLayout(verticalLayout);
    }

    void BorealisMapAndDrone::enterPage()
    {

    }

    void BorealisMapAndDrone::exitPage()
    {

    }

    void BorealisMapAndDrone::droneVideoLayout(){
        ImageInteractiveLayout* mylayout_A = new ImageInteractiveLayout(this, node);
        ImageInteractiveLayout* mylayout_B = new ImageInteractiveLayout(this, node);
        ImageInteractiveLayout* mylayout_C = new ImageInteractiveLayout(this, node);

        verticalLayout->addLayout(mylayout_A);
        verticalLayout->addLayout(mylayout_B);
        verticalLayout->addLayout(mylayout_C);
    };

}