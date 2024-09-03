#include <cnoid/SimpleController>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <thread>
#include <mutex>

class MobileRobotTrackController : public cnoid::SimpleController
{
public:
    virtual bool configure(cnoid::SimpleControllerConfig* config) override;
    virtual bool initialize(cnoid::SimpleControllerIO* io) override;
    virtual bool control() override;
    virtual void unconfigure() override;

private:
    cnoid::Link* trackUnits[2];
    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription;
    geometry_msgs::msg::Twist command;
    std::unique_ptr<rclcpp::executors::StaticSingleThreadedExecutor> executor;
    std::thread executorThread;
    std::mutex commandMutex;
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(MobileRobotTrackController)

bool MobileRobotTrackController::configure(cnoid::SimpleControllerConfig* config)
{
    node = std::make_shared<rclcpp::Node>(config->controllerName());

    subscription = node->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 1,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg){
            std::lock_guard<std::mutex> lock(commandMutex);
            command = *msg;
        });

    executor = std::make_unique<rclcpp::executors::StaticSingleThreadedExecutor>();
    executor->add_node(node);
    executorThread = std::thread([this](){ executor->spin(); });

    return true;
}

bool MobileRobotTrackController::initialize(cnoid::SimpleControllerIO* io)
{
    auto body = io->body();
    trackUnits[0] = body->joint("LeftTracks");
    trackUnits[1] = body->joint("RightTracks");
    for(int i=0; i < 2; ++i){
        auto trackUnit = trackUnits[i];
        trackUnit->setActuationMode(JointVelocity);
        io->enableOutput(trackUnit);
    }
    return true;
}

bool MobileRobotTrackController::control()
{
    {
        std::lock_guard<std::mutex> lock(commandMutex);
        double v_angular = command.angular.z * trackUnits[0]->offsetTranslation().y();
        trackUnits[0]->dq_target() = command.linear.x - v_angular;
        trackUnits[1]->dq_target() = command.linear.x + v_angular;
    }
    return true;
}


void MobileRobotTrackController::unconfigure()
{
    if(executor){
        executor->cancel();
        executorThread.join();
        executor->remove_node(node);
        executor.reset();
    }
}
