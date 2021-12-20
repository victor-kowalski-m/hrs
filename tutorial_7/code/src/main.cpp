#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/JointState.h"
#include "message_filters/subscriber.h"
#include <string.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>
#include <naoqi_bridge_msgs/Bumper.h>
#include <naoqi_bridge_msgs/HeadTouch.h>
#include <naoqi_bridge_msgs/HandTouch.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeedAction.h>
#include <std_srvs/Empty.h>
#include <boost/algorithm/string.hpp>
#include <boost/date_time.hpp>
#include <naoqi_bridge_msgs/SpeechWithFeedbackActionGoal.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <naoqi_bridge_msgs/BlinkActionGoal.h>
#include <naoqi_bridge_msgs/SetSpeechVocabularyActionGoal.h>
#include <std_msgs/ColorRGBA.h>
#include <naoqi_bridge_msgs/WordRecognized.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>
#include "actionlib/client/simple_action_client.h"
#include <tutorial_7/BlinkAction.h>



using namespace std;
std::string greeting;

bool stop_thread=false;
void spinThread()
{
  while(!stop_thread)
  {
    ros::spinOnce();
    //ROS_INFO_STREAM("Spinning the thing!!");
  }
}

class Nao_control
{

protected:

  // ROS node handler
  ros::NodeHandle nh_;

  // Subscriber to bumpers states
  ros::Subscriber bumper_sub;

  // Subscriber to head tactile states
  ros::Subscriber tactile_sub;

  // Publisher for nao speech
  ros::Publisher speech_pub;

  // Publisher for nao vocabulary parameters
  ros::Publisher voc_params_pub;

  // Client for starting speech recognition
  ros::ServiceClient recog_start_srv;

  // Client for stoping speech recognition
  ros::ServiceClient recog_stop_srv;

  // Subscriber to speech recognition
  ros::Subscriber recog_sub;

  // Publisher to nao walking
  ros::Publisher walk_pub;

  // Subscriber for foot contact
  ros::Subscriber footContact_sub;

  // Client for stoping walking
  ros::ServiceClient stop_walk_srv;

public:
  
  bool walking = false;
  boost::thread *spin_thread;

  // Create the action client
  actionlib::SimpleActionClient<tutorial_7::BlinkAction> my_actionClient;

  Nao_control(): my_actionClient(nh_, "blink", true) // Init the action client: target the /blink action server. Here "true" causes the action client to spin its own thread
  {
    // Subscribe to topic bumper and specify that all data will be processed by function bumperCallback
    bumper_sub=nh_.subscribe("/bumper",1, &Nao_control::bumperCallback, this);

    // Subscribe to topic tactile_touch and specify that all data will be processed by function tactileCallback
    tactile_sub=nh_.subscribe("/tactile_touch",1, &Nao_control::tactileCallback, this);

    speech_pub = nh_.advertise<naoqi_bridge_msgs::SpeechWithFeedbackActionGoal>("/speech_action/goal", 1);

    voc_params_pub= nh_.advertise<naoqi_bridge_msgs::SetSpeechVocabularyActionGoal>("/speech_vocabulary_action/goal", 1);

    recog_start_srv=nh_.serviceClient<std_srvs::Empty>("/start_recognition");

    recog_stop_srv=nh_.serviceClient<std_srvs::Empty>("/stop_recognition");

    recog_sub=nh_.subscribe("/word_recognized",1, &Nao_control::speechRecognitionCallback, this);

    footContact_sub = nh_.subscribe<std_msgs::Bool>("/foot_contact", 1, &Nao_control::footContactCallback, this);

    walk_pub=nh_.advertise<geometry_msgs::Pose2D>("/cmd_pose", 1);

    stop_walk_srv=nh_.serviceClient<std_srvs::Empty>("/stop_walk_srv");

    stop_thread=false;
    spin_thread=new boost::thread(&spinThread);
  }
  ~Nao_control()
  {
    stop_thread=true;
    sleep(1);
    spin_thread->join();
  }

  void footContactCallback(const std_msgs::BoolConstPtr& contact)
  {
    if (!(contact->data)){
      stopWalk();
      walking = false;
    }
  }

  void speechRecognitionCallback(const naoqi_bridge_msgs::WordRecognized::ConstPtr& msg)
  {

    //greeting = msg->words[0];
    greeting.append(msg->words[0]);
;
  }

  void bumperCallback(const naoqi_bridge_msgs::Bumper::ConstPtr& bumperState)
  {
    tutorial_7::BlinkGoal blinkGoal;
    std_msgs::ColorRGBA color;
    std::vector<std_msgs::ColorRGBA> colors;
    bool blinkFlag = false;
    double time(0.0);

    if (bumperState->bumper==bumperState->right)
    {
      if (bumperState->state == bumperState->statePressed){
        blinkFlag = true;
        ROS_WARN_STREAM("Right Bumper has been Pressed");
        color.r = 0;
        color.g = 1;
        color.b = 0;
        color.a = 1;
        colors.push_back(color);
        time = 4.0;
      }
    }
    else if(bumperState->bumper==bumperState->left)
    {
      if (bumperState->state == bumperState->statePressed){
        blinkFlag = true;
        ROS_WARN_STREAM("Left Bumper has been Pressed");
        color.r = 1;
        color.g = 0;
        color.b = 0;
        color.a = 1;
        colors.push_back(color);
        time = 2.0;
      }
    }
    else
    {
      blinkFlag = false;
    }

    if(blinkFlag == true)
    {
      blinkGoal.colors = colors;
      blinkGoal.blink_duration = ros::Duration(time);
      blinkGoal.blink_rate_mean = 1.0;
      blinkGoal.blink_rate_sd = 0.1;
      my_actionClient.sendGoal(blinkGoal, this->doneBlinkCallback, this->activeBlinkCallback, this->feedbackBlinkCallback);
    }
    else
    {
      // Must be called to preempt the current action otherwise NAO stays in an infinite blinking state...
      my_actionClient.cancelAllGoals();
    }
  }

  // Called once when the blink goal completes
  static void doneBlinkCallback(const actionlib::SimpleClientGoalState& state, const tutorial_7::BlinkResultConstPtr& result)
  {
    ROS_INFO_STREAM("Finished in state: "<< state.toString().c_str());
  }

  // Called once when the blink goal becomes active
  static void activeBlinkCallback()
  {
    ROS_INFO("Blink goal just went active");
  }

  // Called every time feedback is received for the blink goal
  static void feedbackBlinkCallback(const tutorial_7::BlinkFeedbackConstPtr& feedback)
  {
    ROS_INFO_STREAM("Got the following Feedback: "<< feedback->last_color);
  }

  void tactileCallback(const naoqi_bridge_msgs::HeadTouch::ConstPtr& tactileState)
  {
    // recognize 
    naoqi_bridge_msgs::SetSpeechVocabularyActionGoal vocabulary;
    static long counterVocab = 0;
    std::stringstream vocabId;

    // say
    naoqi_bridge_msgs::SpeechWithFeedbackActionGoal vocabulary_talk;
    static long counterVocab_talk = 0;
    std::stringstream vocabId_talk;

    std::string wordsArr[] = {"Apple", "Banana", "Orange"};
    std::vector<std::string> wordsVec(wordsArr, wordsArr + 3);

    std_srvs::Empty recogArg;

    if (tactileState->button==tactileState->buttonFront)
    {
      if (tactileState->state == tactileState->statePressed){
        ROS_WARN_STREAM("Front button has been pressed");
        
        ++counterVocab;
        vocabId << "vocab" << counterVocab;
        vocabulary.goal_id.id = vocabId.str();
        vocabulary.goal.words = wordsVec;
        voc_params_pub.publish(vocabulary);

        recog_start_srv.call(recogArg);

      }
    }
    else if(tactileState->button==tactileState->buttonMiddle)
    {
      if (tactileState->state == tactileState->statePressed){
        ROS_WARN_STREAM("Middle button has been pressed");
        recog_stop_srv.call(recogArg);


        // talking stuff
        ++counterVocab_talk;
        vocabId_talk << "vocab" << counterVocab_talk;
        vocabulary_talk.goal_id.id = vocabId_talk.str();
        vocabulary_talk.goal.say = greeting;
        speech_pub.publish(vocabulary_talk);
        greeting = "";

      }
    }
    else if(tactileState->button==tactileState->buttonRear)
    {
      if (tactileState->state == tactileState->statePressed){
        ROS_WARN_STREAM("Rear button has been pressed");
        if (!walking) {
          walking = true;
          walker(0.5, 0, 0); // 1. Straight line
          sleep(5);
          walker(0, 0, 2.0); // 2. Turn left
          sleep(5);
          walker(0, 0, -2.0); // 2. Turn right
        } else {
          stopWalk();
          walking = false;

        }
      } 
    }

  }


  void main_loop(ros::Rate rate_sleep)
  {

    while(nh_.ok())
    {
      rate_sleep.sleep();
    }
  }

  void walker(double x, double y, double theta)
  {
    geometry_msgs::Pose2D position;
    position.x = x;
    position.y = y;
    position.theta = theta;

    walk_pub.publish(position);

  }

  void stopWalk()
  {
    std_srvs::Empty stopWalkArg;
    recog_stop_srv.call(stopWalkArg);

  }

};
int main(int argc, char** argv)
{
  ros::init(argc, argv, "control_tutorial_3");
  Nao_control TermiNAOtor;
  ros::Rate rate_sleep(10);

  ROS_INFO_STREAM("Started the Client");
  // Waiting for NAO blink action server to start (roslaunch nao_apps leds.launch):
  TermiNAOtor.my_actionClient.waitForServer();

  // Get into the main loop:
  TermiNAOtor.main_loop(rate_sleep);

  return 0;
}
