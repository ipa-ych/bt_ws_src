#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/bt_service_node.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav2_msgs/nav2_msgs/action/navigate_to_pose.hpp>
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

class CheckBaseInit: public RosServiceNode<std_srvs::srv::Trigger>
{
  public:
    CheckBaseInit(const std::string& name,
                    const NodeConfig& conf,
                    const RosNodeParams& params)
      : RosServiceNode<std_srvs::srv::Trigger>(name, conf, params)
    {}

    static PortsList providedPorts()
    {
      return providedBasicPorts({
        InputPort<int>("node_id"),
        InputPort<std::string>("service_name"),
        InputPort<int>("target_pos"),
        // OutputPort<int>("position")
      });
    }

    bool setRequest(Request::SharedPtr& request) override
    {
      // must return true if we are ready to send the request
      // (void)request;
      return true;
    }

    NodeStatus onResponseReceived(const Response::SharedPtr& response) override
    {
      if (response->success = 1)
      {
        RCLCPP_INFO(logger(),"Base Inited");
        return NodeStatus::SUCCESS;
      }
      else
      {
        RCLCPP_INFO(logger(),"Base NOT Inited !");
                return NodeStatus::FAILURE;
      }
    }
};

class SetLightCyanBreath: public RosServiceNode<std_srvs::srv::Trigger>
{
  public:
    SetLightCyanBreath(const std::string& name,
                    const NodeConfig& conf,
                    const RosNodeParams& params)
      : RosServiceNode<std_srvs::srv::Trigger>(name, conf, params)
    {}

    static PortsList providedPorts()
    {
      return providedBasicPorts({
        InputPort<int>("node_id"),
        InputPort<std::string>("service_name"),
        InputPort<int>("target_pos"),
        // OutputPort<int>("position")
      });
    }

    bool setRequest(Request::SharedPtr& request) override
    {
      // must return true if we are ready to send the request
      // (void)request;
      return true;
    }

    NodeStatus onResponseReceived(const Response::SharedPtr& response) override
    {
      if (response->success = 1)
      {
        RCLCPP_INFO(logger(),"Set Light to Cyan Breath");
        return NodeStatus::SUCCESS;
      }
      else
      {
        RCLCPP_INFO(logger(),"Set Light FAILED !");
                return NodeStatus::FAILURE;
      }
    }

};

class SetLightGreen: public RosServiceNode<std_srvs::srv::Trigger>
{
  public:
    SetLightGreen(const std::string& name,
                    const NodeConfig& conf,
                    const RosNodeParams& params)
      : RosServiceNode<std_srvs::srv::Trigger>(name, conf, params)
    {}

    static PortsList providedPorts()
    {
      return providedBasicPorts({
        InputPort<int>("node_id"),
        InputPort<std::string>("service_name"),
        InputPort<int>("target_pos"),
        // OutputPort<int>("position")
      });
    }

    bool setRequest(Request::SharedPtr& request) override
    {
      // must return true if we are ready to send the request
      // (void)request;
      return true;
    }

    NodeStatus onResponseReceived(const Response::SharedPtr& response) override
    {
      if (response->success = 1)
      {
        RCLCPP_INFO(logger(),"Set Light to Green");
        return NodeStatus::SUCCESS;
      }
      else
      {
        RCLCPP_INFO(logger(),"Set Light FAILED !");
                return NodeStatus::FAILURE;
      }
    }

};

class SetNaviGoal: public RosActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  SetNaviGoal(const std::string& name,
                  const NodeConfig& conf,
                  const RosNodeParams& params)
    : RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params)
  {  }
  
  static PortsList providedPorts() {
        return providedBasicPorts({InputPort<int>("node_id"),
                                  InputPort<std::string>("action_name"), 
                                  InputPort<double>("x"),   // Added input port for x
                                  InputPort<double>("y") });
    }

  bool setGoal(RosActionNode::Goal& goal) override {
    // Retrieve target_position from the input port

    auto x = getInput<double>("x");
    auto y = getInput<double>("y");
    // Check if the inputs were successfully retrieved
    if (!x || !y) {
      RCLCPP_ERROR(logger(), "Failed to get x or y from input ports");
      return false;
    }

    goal.pose.header.frame_id = "map";
    // goal.pose.pose.position.x = 1.0;
    // goal.pose.pose.position.y = -1.0;
    goal.pose.pose.position.x = x.value();
    goal.pose.pose.position.y = y.value();
    goal.pose.pose.orientation.x = 0.0;
    goal.pose.pose.orientation.y = 0.0;
    goal.pose.pose.orientation.w = 1.0;
    goal.pose.pose.orientation.z = 0.0;

    RCLCPP_INFO(logger(), "Goal set\n");
    RCLCPP_INFO(logger(), "Goal set to x: %f, y: %f", x.value(), y.value());
    // Return true, indicating successful goal setting
    return true;
  }
  
  // Callback executed when the reply is received.
  // Based on the reply you may decide to return SUCCESS or FAILURE.
  NodeStatus onResultReceived(const WrappedResult& wr) override
  {
      RCLCPP_INFO(logger(), "Goal reached\n");
      return NodeStatus::SUCCESS;
  }

  // Callback invoked when there was an error at the level
  // of the communication between client and server.
  // This will set the status of the TreeNode to either SUCCESS or FAILURE,
  // based on the return value.
  // If not overridden, it will return FAILURE by default.
  NodeStatus onFailure(ActionNodeErrorCode error) override
  {
    RCLCPP_INFO(logger(), "Error: %d", error);
    return NodeStatus::FAILURE;
  }

  // we also support a callback for the feedback, as in
  // the original tutorial.
  // Usually, this callback should return RUNNING, but you
  // might decide, based on the value of the feedback, to abort
  // the action, and consider the TreeNode completed.
  // In that case, return SUCCESS or FAILURE.
  // The Cancel request will be send automatically to the server.
  NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
  {
    return NodeStatus::RUNNING;
  }
};

// Simple tree, used to execute once each action.

// static const char* xml_text = R"(
//   <root BTCPP_format="4">
//     <BehaviorTree>
//       <Sequence>
//         <CheckBaseInit name="CheckBaseInit"/>
//         <SetLightCyanBreath name="SetLightCyanBreath"/>
//       </Sequence>
//     </BehaviorTree>
//   </root>
//  )";

//  static const char* xml_text = R"(
//   <root BTCPP_format="4">
//     <BehaviorTree>
//       <Sequence>
//         <RunOnce then_skip="true">
//           <CheckBaseInit name="CheckBaseInit"/>
//         </RunOnce>
//         <RunOnce then_skip="true">
//           <SetLightCyanBreath name="SetLightCyanBreath"/>
//         </RunOnce>
//       </Sequence>
//     </BehaviorTree>
//   </root>
//  )";

 static const char* xml_text = R"(
  <root BTCPP_format="4">
    <BehaviorTree>
      <Sequence>
        <RunOnce then_skip="true">
          <SetNaviGoal name="SetNaviGoal" x="1.0" y="-1.0"/>
        </RunOnce>
        <SetNaviGoal name="SetNaviGoal" x="5.0" y="5.0"/>
        <SetNaviGoal name="SetNaviGoal" x="1.0" y="-1.0"/>
      </Sequence>
    </BehaviorTree>
  </root>
 )";

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("cob_light_test");

  BehaviorTreeFactory factory;

  // topic
  RosNodeParams params_sub;
  params_sub.nh = nh;
  params_sub.default_port_value = "btcpp_string";

  RosNodeParams params_pub;
  params_pub.nh = nh;
  params_pub.default_port_value = "btcpp_string1";

  // service

  RosNodeParams params_srv_BaseInit;
  params_srv_BaseInit.nh = nh;
  params_srv_BaseInit.default_port_value = "/base/driver/init";

  RosNodeParams params_srv_setLightCyanBreath;
  params_srv_setLightCyanBreath.nh = nh;
  params_srv_setLightCyanBreath.default_port_value = "/behavior/setLightCyanBreath";

  RosNodeParams params_srv_setLightGreen;
  params_srv_setLightGreen.nh = nh;
  params_srv_setLightGreen.default_port_value = "/behavior/setLightGreen";

  // action
  RosNodeParams params_act_setNaviGoal;
  params_act_setNaviGoal.nh = nh;
  params_act_setNaviGoal.default_port_value = "/navigate_to_pose";

  factory.registerNodeType<ReceiveString>("ReceiveString", params_sub);
  factory.registerNodeType<SendString>("SendString", params_pub);

  factory.registerNodeType<CheckBaseInit>("CheckBaseInit", params_srv_BaseInit);

  factory.registerNodeType<SetLightCyanBreath>("SetLightCyanBreath", params_srv_setLightCyanBreath);
  factory.registerNodeType<SetLightGreen>("SetLightGreen", params_srv_setLightGreen);

  factory.registerNodeType<SetNaviGoal>("SetNaviGoal", params_act_setNaviGoal);

  auto tree = factory.createTreeFromText(xml_text);

  while(rclcpp::ok())
  {
    tree.tickWhileRunning();
  }

  return 0;
}
