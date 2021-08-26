#include <borealis_pages/borealis_smart_glove/borealis_smart_glove.h>

namespace borealis_smart_glove
{
    BorealisSmartGlove::BorealisSmartGlove(ros::NodeHandlePtr rosNode)
        : node(rosNode)
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

        vertical_layout = new QVBoxLayout(this);
        horizontal_layout = new QHBoxLayout();

        vertical_layout->addLayout(setGloveSelectLayout());
    };

    QVBoxLayout* BorealisSmartGlove::setGloveSelectLayout()
    {
        glove_vertical_layout = new QVBoxLayout();
        glove_vertical_layout->addLayout(setButtonLayout());
        glove_vertical_layout->setContentsMargins(0,20,0,20);

        return glove_vertical_layout;
    };

    QFormLayout* BorealisSmartGlove::setButtonLayout()
    {
        // button for scan
        // dropdown to select device
        // dropdown to select model
        // button stop and disconnect

        click_to_scan = new QLabel("Scan for new devices");
        scan = new QPushButton("Scan");

        click_to_disconnect = new QLabel("Disconnect from device");
        disconnect = new QPushButton("Disconnect");

        select_device_to_connect = new QLabel("Select a device");
        select_device = new QComboBox();
        select_device->setSizeAdjustPolicy(QComboBox::AdjustToContents);

        select_model_to_use = new QLabel("Select a model");
        select_model = new QComboBox();
        select_model->setSizeAdjustPolicy(QComboBox::AdjustToContents);

        button_layout = new QFormLayout();
        button_layout->addRow(click_to_scan, scan);
        button_layout->addRow(select_device_to_connect, select_device);
        button_layout->addRow(select_model_to_use, select_model);
        button_layout->addRow(click_to_disconnect, disconnect);
        
        return button_layout;
    };

    void BorealisSmartGlove::enterPage()
    {

    }

    void BorealisSmartGlove::exitPage()
    {

    }


}