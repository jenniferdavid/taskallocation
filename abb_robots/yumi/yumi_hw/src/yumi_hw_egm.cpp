/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Francisco Vina, francisco.vinab@gmail.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <yumi_hw/yumi_hw_egm.h>
#include <abb_egm_interface/egm_common.h>
#include <ros/ros.h>
#include <curl/curl.h>


using namespace abb::egm_interface;
using namespace abb::rws_interface;


YumiEGMInterface::YumiEGMInterface() :
    has_params_(false), rws_connection_ready_(false)
{
    left_arm_feedback_.reset(new proto::Feedback());
    left_arm_status_.reset(new proto::RobotStatus());
    right_arm_feedback_.reset(new proto::Feedback());
    right_arm_status_.reset(new proto::RobotStatus());

    left_arm_joint_vel_targets_.reset(new proto::JointSpace());
    right_arm_joint_vel_targets_.reset(new proto::JointSpace());
    left_arm_joint_pos_targets_.reset(new proto::JointSpace());
    right_arm_joint_pos_targets_.reset(new proto::JointSpace());

    // preallocate memory for feedback/command messages
    initEGMJointSpaceMessage(left_arm_feedback_->mutable_joints());
    initEGMJointSpaceMessage(right_arm_feedback_->mutable_joints());

    initEGMJointStateMessage(left_arm_joint_vel_targets_->mutable_speed(), left_arm_joint_vel_targets_->mutable_external_speed());
    initEGMJointStateMessage(right_arm_joint_vel_targets_->mutable_speed(), right_arm_joint_vel_targets_->mutable_external_speed());

    initEGMJointStateMessage(left_arm_joint_pos_targets_->mutable_position(), left_arm_joint_pos_targets_->mutable_external_position());
    initEGMJointStateMessage(right_arm_joint_pos_targets_->mutable_position(), right_arm_joint_pos_targets_->mutable_external_position());

    getParams();

}


YumiEGMInterface::~YumiEGMInterface()
{

}

void YumiEGMInterface::getParams()
{
    ros::NodeHandle nh("~");

    nh.param("rws/delay_time", rws_delay_time_, 1.0);
    nh.param("rws/max_signal_retries", rws_max_signal_retries_, 5);

    // EGM parameters
    double timeout;
    nh.param("egm/comm_timeout", timeout, 30.0);
    egm_params_.setCommTimeout(timeout);


    std::string tool_name;
    nh.param("egm/tool_name", tool_name, std::string("tool0"));
    egm_params_.setToolName(tool_name);

    std::string wobj_name;
    nh.param("egm/wobj_name", wobj_name, std::string("wobj0"));
    egm_params_.setWobjName(wobj_name);

    double cond_min_max;
    nh.param("egm/cond_min_max", cond_min_max, 0.5);
    egm_params_.setCondMinMax(cond_min_max);

    double lp_filter;
    nh.param("egm/lp_filter", lp_filter, 0.0);
    egm_params_.setLpFilter(lp_filter);

    double max_speed_deviation;
    nh.param("egm/max_speed_deviation", max_speed_deviation, 400.0);
    egm_params_.setMaxSpeedDeviation(max_speed_deviation);

    double condition_time;
    nh.param("egm/condition_time", condition_time, 10.0);
    egm_params_.setCondTime(condition_time);

    double ramp_in_time;
    nh.param("egm/ramp_in_time", ramp_in_time, 0.1);
    egm_params_.setRampInTime(ramp_in_time);

    double pos_corr_gain;
    nh.param("egm/pos_corr_gain", pos_corr_gain, 0.0);
    egm_params_.setPosCorrGain(pos_corr_gain);

    nh.param("egm/use_position_mode", position_, false);

    has_params_ = true;
}


bool YumiEGMInterface::init(const std::string& ip, const std::string& port)
{
    if (!has_params_)
    {
        ROS_ERROR_STREAM(ros::this_node::getName() << ": missing EGM/RWS parameters.");
        return false;
    }

    rws_ip_ = ip;
    rws_port_ = port;

    CURLcode curl_code = curl_global_init(CURL_GLOBAL_DEFAULT);

    if(!curl_code == CURLE_OK)
    {
        return false;
    }

    if(!initRWS())
    {
        return false;
    }

    if (!initEGM(position_))
    {
        return false;
    }

    if(!startEGM())
    {
        return false;
    }

    return true;
}

bool YumiEGMInterface::stop()
{
    if(!stopEGM())
    {
        return false;
    }

    io_service_.stop();
    io_service_threads_.join_all();
    curl_global_cleanup();

    return true;
}

void YumiEGMInterface::getCurrentJointStates(float (&joint_pos)[N_YUMI_JOINTS], float (&joint_vel)[N_YUMI_JOINTS], float (&joint_acc)[N_YUMI_JOINTS])
{
    left_arm_egm_interface_->wait_for_data();
    left_arm_egm_interface_->read(left_arm_feedback_.get(), left_arm_status_.get());

    right_arm_egm_interface_->wait_for_data();
    right_arm_egm_interface_->read(right_arm_feedback_.get(), right_arm_status_.get());

    copyEGMJointSpaceToArray(left_arm_feedback_->joints(), joint_pos, joint_vel, joint_acc);
    copyEGMJointSpaceToArray(right_arm_feedback_->joints(), &joint_pos[7], &joint_vel[7], &joint_acc[7]);
}

void YumiEGMInterface::setJointVelTargets(float (&joint_vel_targets)[N_YUMI_JOINTS])
{
    copyArrayToEGMJointState(joint_vel_targets, left_arm_joint_vel_targets_->mutable_speed(), left_arm_joint_vel_targets_->mutable_external_speed());
    copyArrayToEGMJointState(&joint_vel_targets[7], right_arm_joint_vel_targets_->mutable_speed(), right_arm_joint_vel_targets_->mutable_external_speed());

    left_arm_egm_interface_->write(*left_arm_joint_vel_targets_);
    right_arm_egm_interface_->write(*right_arm_joint_vel_targets_);
}

void YumiEGMInterface::setJointPosTargets(float (&joint_pos_targets)[N_YUMI_JOINTS])
{
    copyArrayToEGMJointState(joint_pos_targets, left_arm_joint_pos_targets_->mutable_position(), left_arm_joint_pos_targets_->mutable_external_position());
    copyArrayToEGMJointState(&joint_pos_targets[7], right_arm_joint_pos_targets_->mutable_position(), right_arm_joint_pos_targets_->mutable_external_position());

    left_arm_egm_interface_->write(*left_arm_joint_pos_targets_);
    right_arm_egm_interface_->write(*right_arm_joint_pos_targets_);
}

void YumiEGMInterface::initEGMJointSpaceMessage(proto::JointSpace *joint_space_message)
{
    initEGMJointStateMessage(joint_space_message->mutable_position(), joint_space_message->mutable_external_position());
    initEGMJointStateMessage(joint_space_message->mutable_speed(), joint_space_message->mutable_external_speed());
    initEGMJointStateMessage(joint_space_message->mutable_acceleration(), joint_space_message->mutable_external_acceleration());
}

void YumiEGMInterface::initEGMJointStateMessage(google::protobuf::RepeatedField<double> *joint_states, google::protobuf::RepeatedField<double> *external_joint_states)
{
    joint_states->Clear();
    for (unsigned int i = 0; i < 6; ++i)
    {
        joint_states->Add(0.0);
    }

    external_joint_states->Clear();
    external_joint_states->Add(0.0);
}

void YumiEGMInterface::copyEGMJointStateToArray(const google::protobuf::RepeatedField<double> &joint_states,
                                                     const google::protobuf::RepeatedField<double> &external_joint_states,
                                                     float* joint_array) const
{
    joint_array[0] = (float)joint_states.Get(0)*M_PI/180.0;
    joint_array[1] = (float)joint_states.Get(1)*M_PI/180.0;
    joint_array[2] = (float)joint_states.Get(3)*M_PI/180.0;
    joint_array[3] = (float)joint_states.Get(4)*M_PI/180.0;
    joint_array[4] = (float)joint_states.Get(5)*M_PI/180.0;
    joint_array[5] = (float)external_joint_states.Get(0)*M_PI/180.0;
    joint_array[6] = (float)joint_states.Get(2)*M_PI/180.0;

    // std::cout << "joint 4 " << joint_array[4] << std::endl;
}

void YumiEGMInterface::copyEGMJointSpaceToArray(const proto::JointSpace &joint_space,
                                                     float* joint_pos, float* joint_vel, float* joint_acc ) const
{
    copyEGMJointStateToArray(joint_space.position(), joint_space.external_position(), joint_pos);
    copyEGMJointStateToArray(joint_space.speed(), joint_space.external_speed(), joint_vel);
    copyEGMJointStateToArray(joint_space.acceleration(), joint_space.external_acceleration(), joint_acc);
}


void YumiEGMInterface::copyArrayToEGMJointState(const float* joint_array,
                                                google::protobuf::RepeatedField<double>* joint_states,
                                                google::protobuf::RepeatedField<double>* external_joint_states) const
{
    joint_states->Set(0, (double) joint_array[0]*180.0/M_PI);
    joint_states->Set(1, (double) joint_array[1]*180.0/M_PI);
    joint_states->Set(2, (double) joint_array[6]*180.0/M_PI);
    joint_states->Set(3, (double) joint_array[2]*180.0/M_PI);
    joint_states->Set(4, (double) joint_array[3]*180.0/M_PI);
    joint_states->Set(5, (double) joint_array[4]*180.0/M_PI);
    external_joint_states->Set(0, (double) joint_array[5]*180.0/M_PI);
}

bool YumiEGMInterface::initRWS()
{
    ROS_INFO_STREAM(ros::this_node::getName() << ": starting RWS connection with IP & PORT: " << rws_ip_ << " / " << rws_port_);

    rws_interface_.reset(new RWSInterfaceYuMi(rws_ip_, rws_port_));
    ros::Duration(rws_delay_time_).sleep();

    // Check that RAPID is running on the robot and that robot is in AUTO mode
    if(!rws_interface_->isRAPIDRunning())
    {
        ROS_ERROR_STREAM(ros::this_node::getName() << ": robot unavailable, make sure that the RAPID program is running on the flexpendant.");
        return false;
    }

    ros::Duration(rws_delay_time_).sleep();

    if(!rws_interface_->isModeAuto())
    {
        ROS_ERROR_STREAM(ros::this_node::getName() << ": robot unavailable, make sure to set the robot to AUTO mode on the flexpendant.");
        return false;
    }

    ros::Duration(rws_delay_time_).sleep();

    if(!sendEGMParams())
    {
        return false;
    }

    rws_connection_ready_ = true;
    ros::Duration(rws_delay_time_).sleep();

    // if(!startEGM()) return false;

    // ros::NodeHandle nh;
    //rws_watchdog_timer_ = nh.createTimer(ros::Duration(rws_watchdog_period_), &YumiEGMInterface::rwsWatchdog, this);

    return true;
}

bool YumiEGMInterface::initEGM(bool position)
{
    left_arm_egm_interface_.reset(new EGMInterfaceDefault(io_service_, egm_common_values::communication::DEFAULT_PORT_NUMBER));
    right_arm_egm_interface_.reset(new EGMInterfaceDefault(io_service_, egm_common_values::communication::DEFAULT_PORT_NUMBER + 1));
    configureEGM(left_arm_egm_interface_, position);
    configureEGM(right_arm_egm_interface_, position);

    // create threads for EGM communication
    for(size_t i = 0; i < MAX_NUMBER_OF_EGM_CONNECTIONS; i++)
    {
        io_service_threads_.create_thread(boost::bind(&boost::asio::io_service::run, &io_service_));
    }

    return true;
}

bool YumiEGMInterface::sendEGMParams()
{
    DualEGMData dual_egm_data;

    if(!rws_interface_->getData(&dual_egm_data))
    {
        ROS_ERROR_STREAM(ros::this_node::getName() << ": robot unavailable, make sure to set the robot to AUTO mode on the flexpendant.");
        return false;
    }

    setEGMParams(&dual_egm_data.left);
    setEGMParams(&dual_egm_data.right);

    rws_interface_->setData(dual_egm_data);

    rws_interface_->getData(&dual_egm_data);

    return true;
}

void YumiEGMInterface::setEGMParams(EGMData* egm_data)
{
    egm_data->setCommTimeout(egm_params_.getCommTimeout());
    egm_data->setToolName(egm_params_.getToolName());
    egm_data->setWobjName(egm_params_.getWobjName());
    egm_data->setCondMinMax(egm_params_.getCondMinMax());
    egm_data->setLpFilter(egm_params_.getLpFilter());
    egm_data->setMaxSpeedDeviation(egm_params_.getMaxSpeedDeviation());
    egm_data->setCondTime(egm_params_.getCondTime());
    egm_data->setRampInTime(egm_params_.getRampInTime());
    egm_data->setPosCorrGain(egm_params_.getPosCorrGain());
}



void YumiEGMInterface::configureEGM(boost::shared_ptr<EGMInterfaceDefault> egm_interface, bool position)
{
    EGMInterfaceConfiguration configuration = egm_interface->getConfiguration();

    configuration.basic.use_conditions = false;
    configuration.basic.axes = EGMInterfaceConfiguration::Seven;
    configuration.basic.execution_mode = EGMInterfaceConfiguration::Direct;
    if (position)
    {
      configuration.communication.use_position = true;
    }
    else
    {
      configuration.communication.use_speed = true;
    }
    configuration.logging.use_logging = false;
    configuration.communication.use_position = position;

    egm_interface->setConfiguration(configuration);
}

bool YumiEGMInterface::startEGM()
{
    bool egm_started = false;

    if(rws_interface_ && rws_connection_ready_)
    {
        for(int i = 0; i < rws_max_signal_retries_ && !egm_started; ++i)
        {
            egm_started = rws_interface_->doEGMStartJoint();
            if(!egm_started)
            {
                ROS_ERROR_STREAM(ros::this_node::getName() << ": failed to send EGM start signal! [Attempt " << i+1 << "/" <<
                                rws_max_signal_retries_<< "]");
            }
        }
    }

    return egm_started;
}

bool YumiEGMInterface::stopEGM()
{
    bool egm_stopped = false;

    if(rws_interface_ && rws_connection_ready_)
    {
        for(int i = 0; i < rws_max_signal_retries_ && !egm_stopped; ++i)
        {
            egm_stopped = rws_interface_->doEGMStop();
            if(!egm_stopped)
            {
                ROS_ERROR_STREAM(ros::this_node::getName() << ": failed to send EGM stop signal! [Attempt " << i+1 << "/" <<
                                rws_max_signal_retries_<< "]");
            }
        }
    }

    return egm_stopped;
}

//void YumiEGMInterface::rwsWatchdog(const ros::TimerEvent &e)
//{
//    bool rc_auto = false;
//    bool rc_rapid_running = true;

//    if(rws_interface_)
//    {
//        rc_auto = rws_interface_->isModeAuto();
//        ros::Duration(rws_delay_time_).sleep();
//        rc_rapid_running = rws_interface_->isRAPIDRunning();

//        if(rc_auto && rc_rapid_running)
//        {
//            //if(!rws_connection_ready_)
//            //{
//                if (sendEGMParams()) rws_connection_ready_ = true;
//            //}
//        }
//        else
//        {
//            ROS_WARN("Robot controller is unavailable (it needs to be in auto mode and also have RAPID started)");
//            rws_connection_ready_ = false;
//        }
//    }
//}

YumiHWEGM::YumiHWEGM() : YumiHW(), is_initialized_(false)
{
  ros::NodeHandle nh("~");
  nh.param("egm/use_position_mode", position_, false);
}

YumiHWEGM::~YumiHWEGM()
{
    yumi_egm_interface_.stop();
}

void YumiHWEGM::setup(const std::string& ip, const std::string& port)
{
    ip_ = ip;
    port_ = port;
}

bool YumiHWEGM::init()
{
    if(is_initialized_)
    {
        return false;
    }
    //current_strategy_ = JOINT_VELOCITY;

    bool success = yumi_egm_interface_.init(ip_, port_);

    if(!success)
    {
        return false;
    }

    is_initialized_ = true;
    return true;
}


void YumiHWEGM::read(ros::Time time, ros::Duration period)
{
    if(!is_initialized_)
    {
        return;
    }

    data_buffer_mutex_.lock();

    yumi_egm_interface_.getCurrentJointStates(joint_pos_, joint_vel_, joint_acc_);

    for (int j = 0; j < n_joints_; j++)
    {
        joint_position_prev_[j] = joint_position_[j];
        joint_position_[j] = joint_pos_[j];

        // Estimation of joint velocity via finite differences method of first order
        // joint_velocity_[j] = (joint_position_[j] - joint_position_prev_[j]) / period.toSec();

        // Estimation of joint velocity via finite differences method of first order and exponential smoothing
        joint_velocity_[j] = filters::exponentialSmoothing((joint_position_[j]-joint_position_prev_[j])/period.toSec(), joint_velocity_[j], exponential_smoothing_alpha_);
    }

    data_buffer_mutex_.unlock();
}


void YumiHWEGM::write(ros::Time time, ros::Duration period)
{
    if(!is_initialized_)
    {
        return;
    }

    enforceLimits(period);

    data_buffer_mutex_.lock();

    for (int j = 0; j < n_joints_; j++)
    {
        joint_vel_targets_[j] = joint_velocity_command_[j];
        joint_pos_targets_[j] = joint_position_command_[j];
    }

    if (!position_)
    {
      yumi_egm_interface_.setJointVelTargets(joint_vel_targets_);
    }
    else
    {
      yumi_egm_interface_.setJointPosTargets(joint_pos_targets_);
    }

    data_buffer_mutex_.unlock();
}
