#include <borealis_pages/borealis_smart_glove/borealis_smart_glove.h>

namespace borealis_smart_glove
{
    BorealisSmartGlove::BorealisSmartGlove(ros::NodeHandlePtr rosNode, std::shared_ptr<borealis_rviz::RViz> rvizPtr)
        : node(rosNode),
          rvizPointer(rvizPtr)
    {
        // overall horizontal layout
        // bluetooth vertical layout
        // rviz layout on the right

        // function to run scan
        // list available peripherals
        // choose from available peripherals
        // list available models
        // choose available models or choose to calibrate again

        this->setEnabled(true);
        this->setGeometry(0, 0, 600, 600);

        verticalLayout = new QVBoxLayout(this);
        horizontalLayout = new QHBoxLayout();

        horizontalLayout->addLayout(setGloveSelectLayout());
        horizontalLayout->addWidget(rvizPointer->rvizFrameWidget_);
        
        verticalLayout->addLayout(horizontalLayout);
    };

    QVBoxLayout* BorealisSmartGlove::setGloveSelectLayout()
    {
        // button for scan
        // dropdown to select device
        // dropdown to select model
        // button stop and disconnect

        scan = new QPushButton();
        disconnect = new QPushButton();

        selectDevice = new QComboBox();
        selectDevice->setSizeAdjustPolicy(QComboBox::AdjustToContents);

        selectModel = new QComboBox();
        selectModel->setSizeAdjustPolicy(QComboBox::AdjustToContents);

        gloveVerticalLayout = new QVBoxLayout();
        gloveVerticalLayout->addWidget(scan);
        gloveVerticalLayout->addWidget(selectDevice);
        gloveVerticalLayout->addWidget(selectModel);
        gloveVerticalLayout->addWidget(disconnect);

        return gloveVerticalLayout;
    };

}