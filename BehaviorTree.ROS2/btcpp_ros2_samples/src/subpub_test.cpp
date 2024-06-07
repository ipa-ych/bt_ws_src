#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include <behaviortree_ros2/bt_action_node.hpp>
#include <std_msgs/msg/string.hpp>
#include <regex>


using namespace BT;

class ReceiveString : public RosTopicSubNode<std_msgs::msg::String>
{
public:
  ReceiveString(const std::string& name, const NodeConfig& conf,
                const RosNodeParams& params)
    : RosTopicSubNode<std_msgs::msg::String>(name, conf, params)
  {  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  NodeStatus onTick(const std::shared_ptr<std_msgs::msg::String>& last_msg) override
  {
    
    if(last_msg)  // empty if no new message received, since the last tick
    {
      RCLCPP_INFO(logger(), "[%s] new message: %s", name().c_str(),
                  last_msg->data.c_str());

      int flag_10 = 0;
      std::regex re(R"(Hello World: (\d+))");
      std::smatch match;
      if (std::regex_search(last_msg->data, match, re) && match.size() > 1)
      {
        int number = std::stoi(match.str(1));
        if (number == 10)
        {
          flag_10 = 1;
          RCLCPP_INFO(logger(),"flag_is10: %d", flag_10);
          return NodeStatus::SUCCESS;
        }
        else
          return NodeStatus::FAILURE;
      }
    }
    else
      return NodeStatus::FAILURE;
    
    // return NodeStatus::SUCCESS;
  }

};

// Create Publisher if BT-Node A returns SUCCESS
class SendString : public RosTopicPubNode<std_msgs::msg::String>
{
public:
  SendString(const std::string& name, const NodeConfig& conf,
                const RosNodeParams& params)
    : RosTopicPubNode<std_msgs::msg::String>(name, conf, params)
  {  }

  bool setMessage(std_msgs::msg::String &msg) override
  {
    msg.data="message 10 received";
    return true;
  }


private: 

};


// Simple tree, used to execute once each action.
static const char* xml_text = R"(
  <root BTCPP_format="4">
    <BehaviorTree>
      <Sequence>
        <ReceiveString name="A"/>
        <SendString name="B"/>
      </Sequence>
    </BehaviorTree>
  </root>
 )";

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("subpub_test");

  BehaviorTreeFactory factory;

  RosNodeParams params_sub;
  params_sub.nh = nh;
  params_sub.default_port_value = "btcpp_string";

  RosNodeParams params_pub;
  params_pub.nh = nh;
  params_pub.default_port_value = "btcpp_string1";

  factory.registerNodeType<ReceiveString>("ReceiveString", params_sub);
  factory.registerNodeType<SendString>("SendString", params_pub);

  auto tree = factory.createTreeFromText(xml_text);

  while(rclcpp::ok())
  {
    tree.tickWhileRunning();
  }

  return 0;
}
