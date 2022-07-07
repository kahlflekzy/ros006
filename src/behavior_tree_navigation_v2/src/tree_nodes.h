#include <behavior_tree_navigation_v2/LoadMapAction.h>
#include <behavior_tree_navigation_v2/LocalizeAction.h>
#include <behavior_tree_navigation_v2/TasksListenerAction.h>
#include <behavior_tree_navigation_v2/TasksAction.h>

#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include "behaviortree_cpp_v3/action_node.h"

using namespace BT;
using namespace behavior_tree_navigation_v2;
using std::cout;
using std::endl;
using std::string;

typedef actionlib::SimpleActionClient<LoadMapAction> LoadMapClient;
typedef actionlib::SimpleActionClient<LocalizeAction> LocalizeClient;
typedef actionlib::SimpleActionClient<TasksListenerAction> TaskListenerClient;
typedef actionlib::SimpleActionClient<TasksAction> ExecuteTaskClient;


class LoadMap : public BT::StatefulActionNode {
    private:
        LoadMapClient client;
        LoadMapGoal goal;
        int status = -1;
        bool flag;
    public:
        LoadMap(const std::string& name):
                    BT::StatefulActionNode(name, {}), client("serve_map", true){
                        ROS_INFO(">>> Waiting for LoadMap Server.");
                        flag = client.waitForServer(ros::Duration(3.0));
                    }

        NodeStatus onStart() override {
            if (!flag)
            {   
                ROS_INFO(">>> Failed to connect to task_listener server on time.");
                return NodeStatus::FAILURE;
            }
            goal.start = 1; // start = 1 specifies that server should start listening for goals.
//            ROS_INFO(">>> Sending first goal.");
            client.sendGoal(goal,
                boost::bind(&LoadMap::onActionCompleted, this, _1, _2),
                LoadMapClient::SimpleActiveCallback(),
                LoadMapClient::SimpleFeedbackCallback()
            );
//            ROS_INFO(">>> Sent first goal.");
            return NodeStatus::RUNNING;
        }
        
        void onHalted() override {
            // notify the server that the operation has been aborted. NOT_IMPLEMENTED
            client.cancelAllGoals();
            cout<<">>> LoadMap was Halted"<<endl;
        }

        NodeStatus onRunning() override {
            if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && status != -1){
                if (status == 0)
                {
                    ROS_INFO("Failed to LoadMap. Returning FAILURE.");
                    return NodeStatus::FAILURE; // when 0 is sent, stop running tree.
                }
                status = -1;
                return NodeStatus::SUCCESS;
            }else{
                return NodeStatus::RUNNING;
            }
        }

        void onActionCompleted(const actionlib::SimpleClientGoalState& state,
                             const LoadMapResultConstPtr& result){
                // ROS_INFO("Finished in state [%s]", state.toString().c_str());
                status = result->status;
            }
};

class Localization : public BT::StatefulActionNode {
    private:
        LocalizeClient client;
        LocalizeGoal goal;
        string pose = "-1";
        bool flag;
    public:
        Localization(const std::string& name, const BT::NodeConfiguration& config):
                    BT::StatefulActionNode(name, config), client("localize", true){
                        ROS_INFO(">>> Waiting for Localization Server.");
                        flag = client.waitForServer(ros::Duration(3.0));
                    }
        
        static BT::PortsList providedPorts(){
            return {BT::OutputPort<string>("message")};
        }

        NodeStatus onStart() override {
            if (!flag)
            {   
                ROS_INFO(">>> Failed to connect to localize server on time.");
                return NodeStatus::FAILURE;
            }
            goal.start = 1; // start = 1 specifies that server should start listening for goals.
//            ROS_INFO(">>> Sending first goal.");
            client.sendGoal(goal,
                boost::bind(&Localization::onActionCompleted, this, _1, _2),
                LocalizeClient::SimpleActiveCallback(),
                LocalizeClient::SimpleFeedbackCallback()
            );
//            ROS_INFO(">>> Sent first goal.");
            return NodeStatus::RUNNING;
        }
        
        void onHalted() override {
            // notify the server that the operation has been aborted. NOT_IMPLEMENTED
            client.cancelAllGoals();
            cout<<">>> TasksListener was Halted"<<endl;
        }

        NodeStatus onRunning() override {
            if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && pose != "-1"){
                if (pose == "")
                {
                    ROS_INFO("Failed to Localize. Returning FAILURE.");
                    return NodeStatus::FAILURE; // when an empty pose array is sent, stop running tree.
                }
                pose = "-1";
                return NodeStatus::SUCCESS;
            }else{
                return NodeStatus::RUNNING;
            }
        }

          void onActionCompleted(const actionlib::SimpleClientGoalState& state,
                                 const LocalizeResultConstPtr& result){
                // ROS_INFO("Finished in state [%s]", state.toString().c_str());
                pose = result->pose;
                setOutput("message", pose);
            }
};

class TaskListener : public BT::StatefulActionNode {
    private:
        TaskListenerClient client;
        TasksListenerGoal goal;
        string tasks;
        bool flag;
    public:
        TaskListener(const std::string& name, const BT::NodeConfiguration& config):
                    BT::StatefulActionNode(name, config), client("task_listener", true){
                        ROS_INFO(">>> Waiting for TaskListener Server.");
                        flag = client.waitForServer(ros::Duration(3.0));
                    }
        
        static BT::PortsList providedPorts(){
            return {BT::OutputPort<string>("message")};
        }

        NodeStatus onStart() override {
            if (!flag)
            {   
                ROS_INFO(">>> Failed to connect to task_listener server on time.");
                return NodeStatus::FAILURE;
            }
            goal.start = 1; // start = 1 specifies that server should start listening for goals.
//            ROS_INFO(">>> Sending first goal.");
            client.sendGoal(goal,
                boost::bind(&TaskListener::onActionCompleted, this, _1, _2),
                TaskListenerClient::SimpleActiveCallback(),
                TaskListenerClient::SimpleFeedbackCallback()
            );
//            ROS_INFO(">>> Sent first goal.");
            return NodeStatus::RUNNING;
        }
        
        void onHalted() override {
            // notify the server that the operation has been aborted. NOT_IMPLEMENTED
            client.cancelAllGoals();
            cout<<">>> TasksListener was Halted"<<endl;
        }

        NodeStatus onRunning() override {
            if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                if (tasks == "0")
                {
                    ROS_INFO("Got 0 tasks. Returning FAILURE.");
                    return NodeStatus::FAILURE; // when an empty pose array is sent, stop running tree.
                }
                return NodeStatus::SUCCESS;
            }else{
                return NodeStatus::RUNNING;
            }
        }

          void onActionCompleted(const actionlib::SimpleClientGoalState& state,
                                 const TasksListenerResultConstPtr& result){
                // ROS_INFO("Finished in state [%s]", state.toString().c_str());
                tasks = result->tasks;
                setOutput("message", tasks);
            }
};

class ExecuteTask : public BT::StatefulActionNode {
    private:
        ExecuteTaskClient client;
        TasksGoal goal;
        int status;
        bool flag;
    public:
        ExecuteTask(const std::string& name, const BT::NodeConfiguration& config):
                    BT::StatefulActionNode(name, config), client("move_robot", true){
                        ROS_INFO(">>> Waiting for ExecuteTask Server.");
                        flag = client.waitForServer(ros::Duration(3.0));
                    }
        
        static BT::PortsList providedPorts(){
            return {BT::InputPort<string>("message")};
        }

        NodeStatus onStart() override {
            if (!flag)
            {   
                ROS_INFO(">>> Failed to connect to move_robot server on time.");
                return NodeStatus::FAILURE;
            }
            
            getInput("message", goal.tasks);
            //goal.tasks = tasks; // or use a string tasks and pass to getInput then assign here

            ROS_INFO(">>> Executing tasks.");
            client.sendGoal(goal,
                boost::bind(&ExecuteTask::onActionCompleted, this, _1, _2),
                ExecuteTaskClient::SimpleActiveCallback(),
                ExecuteTaskClient::SimpleFeedbackCallback()
            );
//            ROS_INFO(">>> Sent first goal.");
            return NodeStatus::RUNNING;
        }
        
        void onHalted() override {
            // notify the server that the operation has been aborted. NOT_IMPLEMENTED
            client.cancelAllGoals();
            cout<<">>> TasksListener was Halted"<<endl;
        }

        NodeStatus onRunning() override {
            if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                if (status != 0)
                {
                    ROS_INFO("Failed to complete task execution. Returning FAILURE.");
                    return NodeStatus::FAILURE; // when an empty pose array is sent, stop running tree.
                }
                return NodeStatus::SUCCESS;
            }else{
                return NodeStatus::RUNNING;
            }
        }

        void onActionCompleted(const actionlib::SimpleClientGoalState& state,
                                const TasksResultConstPtr& result){
            // ROS_INFO("Finished in state [%s]", state.toString().c_str());
            status = result->status;
        }
};

class MoveToBase : public BT::StatefulActionNode {
    private:
        ExecuteTaskClient client;
        TasksGoal goal;
        int status;
        bool flag;
    public:
        MoveToBase(const std::string& name, const BT::NodeConfiguration& config):
                    BT::StatefulActionNode(name, config), client("move_to_base", true){
                        ROS_INFO(">>> Waiting for MoveToBase Server.");
                        flag = client.waitForServer(ros::Duration(3.0));
                    }
        
        static BT::PortsList providedPorts(){
            return {BT::InputPort<string>("message")};
        }

        NodeStatus onStart() override {
            if (!flag)
            {   
                ROS_INFO(">>> Failed to connect to MoveToBase server on time.");
                return NodeStatus::FAILURE;
            }
            
            getInput("message", goal.tasks);
            ROS_INFO(">>> Moving to base.");
            client.sendGoal(goal,
                boost::bind(&MoveToBase::onActionCompleted, this, _1, _2),
                ExecuteTaskClient::SimpleActiveCallback(),
                ExecuteTaskClient::SimpleFeedbackCallback()
            );
            return NodeStatus::RUNNING;
        }
        
        void onHalted() override {
            // notify the server that the operation has been aborted. NOT_IMPLEMENTED
            client.cancelAllGoals();
            cout<<">>> TasksListener was Halted"<<endl;
        }

        NodeStatus onRunning() override {
            if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                if (status != 0)
                {
                    ROS_INFO("Failed to complete task execution. Returning FAILURE.");
                    return NodeStatus::SUCCESS; // will be inverted
                }
                return NodeStatus::FAILURE; // will be inverted
            }else{
                return NodeStatus::RUNNING;
            }
        }

        void onActionCompleted(const actionlib::SimpleClientGoalState& state,
                                const TasksResultConstPtr& result){
            // ROS_INFO("Finished in state [%s]", state.toString().c_str());
            status = result->status;
        }
};