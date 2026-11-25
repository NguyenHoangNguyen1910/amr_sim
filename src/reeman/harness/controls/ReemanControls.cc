#include <gz/gui/Plugin.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

class ReemanControls final : public gz::gui::Plugin
{
    Q_OBJECT

public:
    Q_INVOKABLE void command(int direction)
    {
        gz::msgs::Twist message;
        switch (direction)
        {
            case 1: message.mutable_linear()->set_x(0.30); break;
            case 2: message.mutable_linear()->set_x(-0.30); break;
            case 3: message.mutable_angular()->set_z(0.50); break;
            case 4: message.mutable_angular()->set_z(-0.50); break;
        }
        velocity.Publish(message);
    }

private:
    gz::transport::Node node;
    gz::transport::Node::Publisher velocity =
        node.Advertise<gz::msgs::Twist>("/sim/teleop");
};

GZ_ADD_PLUGIN(ReemanControls, gz::gui::Plugin)

#include "ReemanControls.moc"
