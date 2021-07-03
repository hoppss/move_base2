#include "move_base2/speaker_client.h"

namespace move_base
{
// c-style
static int gSoundAudioArrays[LAST_FLAG] = {
  [FAR_WAIT] = 620, [FAST_WAIT] = 621,     [FLOW_LOST] = 622,     [CHANGE_ROUTE] = 623,
  [OBSTACLE] = 624, [KEEP_DISTANCE] = 625, [EMERGENCY_STOP] = 626
};

SpeakerClient::SpeakerClient() : name_{ "speaker_client" }, logger_(rclcpp::get_logger("move_base_speaker"))
{
}

void SpeakerClient::initialize(const rclcpp_lifecycle::LifecycleNode::SharedPtr& parent)
{
  node_ = parent;
  //   clock_ = node_->get_clock();
  audio_client_ = node_->create_client<AudioPlayT>("audio_play");
}

SpeakerClient::~SpeakerClient()
{
  audio_client_.reset();
}

bool SpeakerClient::play(FollowType type)
{
  return sendGoal(gSoundAudioArrays[type]);
}

bool SpeakerClient::sendGoal(int audio_id)
{
  using namespace std::placeholders;

  // sound playing should not block service call, so we just wait for a short time.
  if (!audio_client_->wait_for_action_server(std::chrono::seconds(1)))
  {
    RCLCPP_ERROR(logger_, "Audio server not available after waiting");
    return false;
  }

  auto goal_msg = AudioPlayT::Goal();
  goal_msg.order.name.id = audio_id;
  goal_msg.order.user.id = interaction_msgs::msg::AudioUser::DEFAULT;

  RCLCPP_INFO(logger_, "Send audio play request");

  auto send_goal_options = rclcpp_action::Client<AudioPlayT>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(&SpeakerClient::goal_response_callback, this, _1);
  send_goal_options.feedback_callback = std::bind(&SpeakerClient::feedback_callback, this, _1, _2);
  send_goal_options.result_callback = std::bind(&SpeakerClient::result_callback, this, _1);
  audio_client_->async_send_goal(goal_msg, send_goal_options);

  return true;
}

void SpeakerClient::goal_response_callback(std::shared_future<GoalHandleAudio::SharedPtr> future)
{
  auto goal_handle = future.get();
  if (!goal_handle)
  {
    RCLCPP_ERROR(logger_, "Goal was rejected by server");
  }
  else
  {
    RCLCPP_INFO(logger_, "Goal accepted by server, waiting for result");
  }
}

void SpeakerClient::feedback_callback(GoalHandleAudio::SharedPtr,
                                      const std::shared_ptr<const AudioPlayT::Feedback> feedback)
{
  RCLCPP_INFO(logger_, "status: %u", feedback->feed.status);
}

void SpeakerClient::result_callback(const GoalHandleAudio::WrappedResult& result)
{
  RCLCPP_INFO(logger_, "result: %u", result.result->result.error);
}

}  // namespace move_base
