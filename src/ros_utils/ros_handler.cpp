// Copyright (c) 2013, Maria Teresa Lazaro Grañon
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice, this
//   list of conditions and the following disclaimer in the documentation and/or
//   other materials provided with the distribution.
//
//   Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
// ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "ros_handler.h"

RosHandler::RosHandler (int idRobot, int nRobots, TypeExperiment typeExperiment){

  _idRobot = idRobot;
  _nRobots = nRobots;
  _typeExperiment = typeExperiment;

  _gtPoses = new SE2[nRobots];
  _subgt = new ros::Subscriber[nRobots];
  _timeLastPing = new ros::Time[nRobots];
  
  _odomTopic = "odom";
  _scanTopic = "base_scan";
  _gpsTopic = "ublox_gps_rover/fix";
  _headingTopic = "ublox_gps_rover/navrelposned";

  _baseFrameId = "base_link";
  _trobotlaser = SE2(0,0,0);

  _useOdom = false;
  _useLaser = false;
  _useGPS = false;

  _invertedLaser = true;

  std::string fullns = ros::this_node::getNamespace();
  std::string delimiter = "_";
  _rootns = fullns.substr(0, fullns.find(delimiter));

  std::cerr << "NAMESPACE: " << fullns << std::endl;
  std::cerr << "ROOT NAMESPACE: " << _rootns << std::endl;
}

void RosHandler::pingCallback(const cg_mrslam::Ping::ConstPtr& msg){
  int robot = msg->robotFrom;
  std::cerr << "Received Ping from robot " << robot << std::endl;
  
  _timeLastPing[robot] = ros::Time::now();
}

void RosHandler::groundTruthCallback(const nav_msgs::Odometry::ConstPtr& msg, SE2 *gtpose)
{ 
  gtpose->setTranslation(Eigen::Vector2d(msg->pose.pose.position.x,  msg->pose.pose.position.y));
  gtpose->setRotation(Eigen::Rotation2Dd(tf::getYaw(msg->pose.pose.orientation)));
}

void RosHandler::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{ 
  _odom = *msg;
}

void RosHandler::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  _laserscan = *msg;
}

void RosHandler::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  _gps = *msg;
}
void RosHandler::headingCallback(const ublox_msgs::NavRELPOSNED::ConstPtr& msg)
{
  _heading = *msg;
}

SE2 RosHandler::getOdom(){
  SE2 odomSE2;
  odomSE2.setTranslation(Eigen::Vector2d(_odom.pose.pose.position.x, _odom.pose.pose.position.y));
  odomSE2.setRotation(Eigen::Rotation2Dd(tf::getYaw(_odom.pose.pose.orientation)));
  return odomSE2;
}

RobotLaser* RosHandler::getLaser(){

  LaserParameters lparams(0, _laserscan.ranges.size(), _laserscan.angle_min,  _laserscan.angle_increment, _laserscan.range_max, 0.1, 0);
  lparams.laserPose = _trobotlaser;

  RobotLaser* rlaser = new RobotLaser;
  rlaser->setLaserParams(lparams);
  rlaser->setOdomPose(getOdom());
  std::vector<double> ranges(_laserscan.ranges.size());
  for (size_t i =0; i < _laserscan.ranges.size(); i++){
    if(std::isinf(_laserscan.ranges[i])){
        ranges[i] = 0;  
      }
      else{
          ranges[i] = _laserscan.ranges[i];
      }
  }
  if (_invertedLaser)
    std::reverse(ranges.begin(), ranges.end());
  
  rlaser->setRanges(ranges);
  rlaser->setTimestamp(_laserscan.header.stamp.sec + _laserscan.header.stamp.nsec * pow(10, -9));
  rlaser->setLoggerTimestamp(rlaser->timestamp());
  rlaser->setHostname("hostname");

  return rlaser;
}
double RosHandler::Deg2Rad(double deg) {
    return M_PI * deg /180.;
}
Eigen::Vector2d RosHandler::GPS2Ecef(const double lat,const double lon,const double alt){

  // https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_geodetic_to_ECEF_coordinates
  constexpr double a = 6378137.;  // semi-major axis, equator to center.
  constexpr double f = 1. / 298.257223563;
  constexpr double b = a * (1. - f);  // semi-minor axis, pole to center.
  constexpr double a_squared = a * a;
  constexpr double b_squared = b * b;
  constexpr double e_squared = (a_squared - b_squared) / a_squared;

  const double sin_phi = std::sin(Deg2Rad(lat));
  const double cos_phi = std::cos(Deg2Rad(lat));
  const double sin_lambda = std::sin(Deg2Rad(lon));
  const double cos_lambda = std::cos(Deg2Rad(lon));
  const double N = a / std::sqrt(1 - e_squared * sin_phi * sin_phi);
  const double x = (N + alt) * cos_phi * cos_lambda;
  const double y = (N + alt) * cos_phi * sin_lambda;
  const double z = (b_squared / a_squared * N + alt) * sin_phi;

  return Eigen::Vector2d(-x,-y);
}
SE2 RosHandler::getGPS(){
      SE2 gpsSE2;
      gpsSE2.setTranslation(GPS2Ecef(_gps.latitude, _gps.longitude, _gps.altitude));
      gpsSE2.setRotation(Eigen::Rotation2Dd(Deg2Rad(-_heading.relPosHeading * 1E-5) + M_PI_2));
      return gpsSE2;
}
void RosHandler::init(){

  if (_useOdom){
    //Init Odom
    nav_msgs::Odometry::ConstPtr odommsg = ros::topic::waitForMessage<nav_msgs::Odometry>(_odomTopic);
    _odom = *odommsg;
  }
  
  if (_useLaser){
  //Init scan
    sensor_msgs::LaserScan::ConstPtr lasermsg = ros::topic::waitForMessage<sensor_msgs::LaserScan>(_scanTopic);
    _laserscan = *lasermsg;
    _laserMaxRange = _laserscan.range_max;

    tf::StampedTransform laserTransform;
    try {
      tf::TransformListener listener;
      listener.waitForTransform(_baseFrameId, lasermsg->header.frame_id,
				ros::Time::now(), ros::Duration(0.5));
      listener.lookupTransform(_baseFrameId, lasermsg->header.frame_id,
			       ros::Time::now(), laserTransform);
      _trobotlaser = SE2(laserTransform.getOrigin().x(),
			 laserTransform.getOrigin().y(),
			 tf::getYaw(laserTransform.getRotation()));
    }
    catch (tf::TransformException ex){
      ROS_ERROR("cg_mrslam: %s",ex.what());
      ros::Duration(1.0).sleep();
    }

    std::cerr << "Robot-laser transform: (" << _trobotlaser.translation().x() << ", " << _trobotlaser.translation().y() << ", " << _trobotlaser.rotation().angle() << ")" << std::endl;
  }
  if (_useGPS){
    sensor_msgs::NavSatFix::ConstPtr gpsmsg = ros::topic::waitForMessage<sensor_msgs::NavSatFix>(_gpsTopic);
    _gps = *gpsmsg;
    ublox_msgs::NavRELPOSNED::ConstPtr headingmsg = ros::topic::waitForMessage<ublox_msgs::NavRELPOSNED>(_headingTopic);
    _heading = *headingmsg;
  }
  if (_typeExperiment == SIM){
    //Init ground-truth
    for (int r = 0; r < _nRobots; r++){
      std::stringstream nametopic;
      nametopic << _rootns << "_" << r << "/base_pose_ground_truth";
      nav_msgs::Odometry::ConstPtr gtmsg = ros::topic::waitForMessage<nav_msgs::Odometry>(nametopic.str());
      _gtPoses[r] = SE2(gtmsg->pose.pose.position.x, gtmsg->pose.pose.position.y, tf::getYaw(gtmsg->pose.pose.orientation));
    }
  }

}

void RosHandler::run(){
  if (_useOdom) //Subscribe Odom
    _subOdom = _nh.subscribe<nav_msgs::Odometry>(_odomTopic, 1, &RosHandler::odomCallback, this);
    
  if (_useLaser) //Subscribe Laser
    _subScan = _nh.subscribe<sensor_msgs::LaserScan>(_scanTopic, 1,  &RosHandler::scanCallback, this);

  if(_useGPS){ //Subscribe GPS
    _subGPS = _nh.subscribe<sensor_msgs::NavSatFix>(_gpsTopic, 1, &RosHandler::gpsCallback, this);
    _subHeading =_nh.subscribe<ublox_msgs::NavRELPOSNED>(_headingTopic, 1, &RosHandler::headingCallback, this);
  }
  if (_typeExperiment == BAG){
    //subscribe pings
    _subPing = _nh.subscribe<cg_mrslam::Ping>("ping_msgs", 1, &RosHandler::pingCallback, this);
  } else if (_typeExperiment == SIM){
    //subscribe ground truth
    for (int r = 0; r < _nRobots; r++){
      std::stringstream nametopic;
      nametopic << _rootns << "_" << r << "/base_pose_ground_truth";
      _subgt[r] = _nh.subscribe<nav_msgs::Odometry>(nametopic.str(), 1, boost::bind(&RosHandler::groundTruthCallback, this, _1, &_gtPoses[r]));
    }
  } else if (_typeExperiment == REAL){
    //publish ping, sent and received messages
    _pubRecv = _nh.advertise<cg_mrslam::SLAM>("recv_msgs", 1);
    _pubSent = _nh.advertise<cg_mrslam::SLAM>("sent_msgs", 1);
    _pubPing = _nh.advertise<cg_mrslam::Ping>("ping_msgs", 1);
  }
}

void RosHandler::publishPing(int idRobotFrom){
  cg_mrslam::Ping rosmsg;

  rosmsg.header.stamp = ros::Time::now();

  rosmsg.robotFrom = idRobotFrom;
  rosmsg.robotTo   = _idRobot;
  
  _pubPing.publish(rosmsg);
}

void RosHandler::createComboMsg(ComboMessage* cmsg, cg_mrslam::SLAM& dslamMsg){
  dslamMsg.header.stamp = ros::Time::now();

  dslamMsg.robotId = cmsg->robotId();
  dslamMsg.type = cmsg->type();
  
  cg_mrslam::RobotLaser laser;
  laser.nodeId = cmsg->nodeId;
  laser.readings = cmsg->readings;
  laser.minAngle = cmsg->minangle;
  laser.angleInc = cmsg->angleincrement;
  laser.maxRange = cmsg->maxrange;
  laser.accuracy = cmsg->accuracy;
  dslamMsg.laser = laser;

  dslamMsg.vertices.resize(cmsg->vertexVector.size());
  for (size_t i = 0; i < cmsg->vertexVector.size(); i++){
    dslamMsg.vertices[i].id = cmsg->vertexVector[i].id;
    dslamMsg.vertices[i].estimate[0] = cmsg->vertexVector[i].estimate[0];
    dslamMsg.vertices[i].estimate[1] = cmsg->vertexVector[i].estimate[1];
    dslamMsg.vertices[i].estimate[2] = cmsg->vertexVector[i].estimate[2];   
  }
}

void RosHandler::createCondensedGraphMsg(CondensedGraphMessage* gmsg,  cg_mrslam::SLAM& dslamMsg){
  dslamMsg.header.stamp = ros::Time::now();

  dslamMsg.robotId = gmsg->robotId();
  dslamMsg.type = gmsg->type();
  
  dslamMsg.edges.resize(gmsg->edgeVector.size());
  for (size_t i = 0; i < gmsg->edgeVector.size(); i++){
    dslamMsg.edges[i].idFrom = gmsg->edgeVector[i].idfrom;
    dslamMsg.edges[i].idTo = gmsg->edgeVector[i].idto;
    dslamMsg.edges[i].estimate[0] = gmsg->edgeVector[i].estimate[0];
    dslamMsg.edges[i].estimate[1] = gmsg->edgeVector[i].estimate[1];
    dslamMsg.edges[i].estimate[2] = gmsg->edgeVector[i].estimate[2];
    dslamMsg.edges[i].information[0] = gmsg->edgeVector[i].information[0];
    dslamMsg.edges[i].information[1] = gmsg->edgeVector[i].information[1];
    dslamMsg.edges[i].information[2] = gmsg->edgeVector[i].information[2];
    dslamMsg.edges[i].information[3] = gmsg->edgeVector[i].information[3];
    dslamMsg.edges[i].information[4] = gmsg->edgeVector[i].information[4];
    dslamMsg.edges[i].information[5] = gmsg->edgeVector[i].information[5];
  }

  dslamMsg.closures = gmsg->closures;
}

void RosHandler::createDSlamMsg(RobotMessage* msg, cg_mrslam::SLAM& dslamMsg){
  ComboMessage* cmsg = dynamic_cast<ComboMessage*>(msg);
  if (cmsg)
    createComboMsg(cmsg, dslamMsg);
  else{
    CondensedGraphMessage* gmsg = dynamic_cast<CondensedGraphMessage*>(msg);
    if (gmsg)
      createCondensedGraphMsg(gmsg, dslamMsg);
  }
}

void RosHandler::publishSentMsg(RobotMessage* msg){
  cg_mrslam::SLAM dslamMsg;

  createDSlamMsg(msg, dslamMsg);
  _pubSent.publish(dslamMsg);
}

void RosHandler::publishReceivedMsg(RobotMessage* msg){
  cg_mrslam::SLAM dslamMsg;

  createDSlamMsg(msg, dslamMsg);
  _pubRecv.publish(dslamMsg);
}
