#include "FootControl.h"
#include "Utils.h"

FootControl* FootControl::me = NULL;

FootControl::FootControl(ros::NodeHandle &n, double frequency, std::string filename):
  _n(n),
  _loopRate(frequency),
  _dt(1.0f/frequency),
  _xCFilter(3,3,6,1.0f/frequency),
  _xDFilter(3,3,6,1.0f/frequency),
  _zDirFilter(3,3,6,1.0f/frequency),
  _filename(filename)
{
  me = this;

  _useLeftRobot = true;
  _useRightRobot = true;

  _gravity << 0.0f, 0.0f, -9.80665f;
  _toolComPositionFromSensor << 0.0f,0.0f,0.02f;
  _toolOffsetFromEE = 0.13f;
  _toolMass = 0.0f;

  _p << 0.0f,0.0f,-0.007f;
  // _taskAttractor << -0.65f, 0.05f, -0.007f;
  _taskAttractor << -0.6f, 0.1f, 0.4f;
  _planeNormal << 0.0f, 0.0f, 1.0f;

  for(int k = 0; k < NB_ROBOTS; k++)
  {
    _x[k].setConstant(0.0f);
    _x0[k].setConstant(0.0f);
    _q[k].setConstant(0.0f);
    _wrenchBias[k].setConstant(0.0f);
    _wrench[k].setConstant(0.0f);
    _filteredWrench[k].setConstant(0.0f);
    
    _xd[k].setConstant(0.0f);
    _fx[k].setConstant(0.0f);
    _vd[k].setConstant(0.0f);
    _qd[k] << 0.0f,0.0f,1.0f,0.0f;
    _omegad[k].setConstant(0.0f);
    _qd[k].setConstant(0.0f);
    _normalForce[k] = 0.0f;
    _normalDistance[k] = 0.0f;
    _Fd[k] = 0.0f;
    _e1[k] << 0.0f, 0.0f, -1.0f;

    _firstRobotPose[k] = false;
    _firstRobotTwist[k] = false;
    _firstWrenchReceived[k] = false;
    _firstDampingMatrix[k] = true;

    _wrenchCount[k] = 0;
    _wrenchBiasOK[k] = false;
    _d1[k] = 1.0f;

    _footPose[k].setConstant(0.0f);
    _footWrench[k].setConstant(0.0f);
    _footTwist[k].setConstant(0.0f);
    _footDesiredWrench[k].setConstant(0.0f);
    _footPosition[k].setConstant(0.0f);
    _vh[k].setConstant(0.0f);
    _footSensor[k].setConstant(0.0f);
    _footInterfaceSequenceID[k] = 0;    
    _firstFootInterfacePose[k] = false;
    _firstFootOutput[k] = false;
    _firstFootSensor[k] = false;
    _desiredFootWrench[k].setConstant(0.0f);
    _filteredFootSensor[k].setConstant(0.0f);

  }
  for(int k = 0; k < TOTAL_NB_MARKERS; k++)
  {
    _firstOptitrackPose[k] = true;
  }
  _optitrackOK = true;
  _stop = false;
  _firstObjectPose = true;
  _leftRobotOrigin << 0.066f, 0.9f, 0.0f;

  _objectDim << 0.41f, 0.22f, 0.22f;

  _markersPosition.setConstant(0.0f);
  _markersPosition0.setConstant(0.0f);
  _markersSequenceID.setConstant(0);
  _markersTracked.setConstant(0);

  _smax = 4.0f;
  _s = _smax;
  _dW = 0.0f;

  _sequenceID = 0;
  _averageCount = 0;

  _velocityLimit = 0.0f;

  _kxy = 0.0f;
  _dxy = 0.0f;
  _kphi = 0.0f;
  _dphi = 0.0f;

  _xAttractor[0] << -0.8, 0.4f,0.4f;
  _xAttractor[0] << -0.7, 0.0f,0.10f;
  _xAttractor[1] << -0.7, -0.5f,0.4f;
  _xAttractor[2] << -0.7, 0.0f,0.6f;
  _beliefs.setConstant(0.0f);
  _dbeliefs.setConstant(0.0f);
  for(int k = 0; k < NB_TASKS; k++)
  {
    _fxk[k].setConstant(0.0f);
  }
  _beliefs(2) = 1.0f;
  _adaptationRate = 100.0f;

  _msgMarker.header.frame_id = "world";
  _msgMarker.header.stamp = ros::Time();
  _msgMarker.ns = "object_shape";
  _msgMarker.id = 0;
  _msgMarker.type = visualization_msgs::Marker::CUBE;
  _msgMarker.action = visualization_msgs::Marker::ADD;
  _msgMarker.pose.position.x = _leftRobotOrigin(0)/2.0f;
  _msgMarker.pose.position.y = _leftRobotOrigin(1)/2.0f;
  _msgMarker.pose.position.z = _leftRobotOrigin(2)/2.0f;
  _msgMarker.pose.orientation.x = 0.0;
  _msgMarker.pose.orientation.y = 0.0;
  _msgMarker.pose.orientation.z = 0.0;
  _msgMarker.pose.orientation.w = 1.0;
  _msgMarker.scale.x = _objectDim(0);
  _msgMarker.scale.y = _objectDim(1);
  _msgMarker.scale.z = _objectDim(2);
  _msgMarker.color.a = 1.0;

  _msgMarker.color.r = 0.1f;
  _msgMarker.color.g = 0.3f;
  _msgMarker.color.b = 0.9f;
  _msgMarker.color.a = 1.0;

  _graspingForceThreshold = 4.0f;  // Grasping force threshold [N]
  _objectGrasped = false;
  _targetForce = 15.0f;

  _x0[LEFT](0) = _leftRobotOrigin(0)-0.60f;
  _x0[LEFT](1) = _leftRobotOrigin(1)-0.35f;
  _x0[LEFT](2) = _leftRobotOrigin(2)+0.34f;
  _x0[RIGHT](0) = -0.52f;
  _x0[RIGHT](1) = 0.4f;
  _x0[RIGHT](2) = 0.34f;

}


bool FootControl::init() 
{
  // Subscriber definitions
  _subRobotPose[RIGHT] = _n.subscribe<geometry_msgs::Pose>("/lwr/ee_pose", 1, boost::bind(&FootControl::updateRobotPose,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subRobotTwist[RIGHT] = _n.subscribe<geometry_msgs::Twist>("/lwr/joint_controllers/twist", 1, boost::bind(&FootControl::updateRobotTwist,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subDampingMatrix[RIGHT] = _n.subscribe<std_msgs::Float32MultiArray>("/lwr/joint_controllers/passive_ds_damping_matrix", 1, boost::bind(&FootControl::updateDampingMatrix,this,_1,RIGHT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subForceTorqueSensor[RIGHT] = _n.subscribe<geometry_msgs::WrenchStamped>("/ft_sensor_right/netft_data", 1, boost::bind(&FootControl::updateRobotWrench,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

  _subRobotPose[LEFT] = _n.subscribe<geometry_msgs::Pose>("/lwr2/ee_pose", 1, boost::bind(&FootControl::updateRobotPose,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subRobotTwist[LEFT] = _n.subscribe<geometry_msgs::Twist>("/lwr2/joint_controllers/twist", 1, boost::bind(&FootControl::updateRobotTwist,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subDampingMatrix[LEFT] = _n.subscribe<std_msgs::Float32MultiArray>("/lwr2/joint_controllers/passive_ds_damping_matrix", 1, boost::bind(&FootControl::updateDampingMatrix,this,_1,LEFT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subForceTorqueSensor[LEFT] = _n.subscribe<geometry_msgs::WrenchStamped>("/ft_sensor_left/netft_data", 1, boost::bind(&FootControl::updateRobotWrench,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

  _subOptitrackPose[ROBOT_BASIS_LEFT] = _n.subscribe<geometry_msgs::PoseStamped>("/optitrack/robot_left/pose", 1, boost::bind(&FootControl::updateOptitrackPose,this,_1,ROBOT_BASIS_LEFT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[ROBOT_BASIS_RIGHT] = _n.subscribe<geometry_msgs::PoseStamped>("/optitrack/robot_right/pose", 1, boost::bind(&FootControl::updateOptitrackPose,this,_1,ROBOT_BASIS_RIGHT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[P1] = _n.subscribe<geometry_msgs::PoseStamped>("/optitrack/p1/pose", 1, boost::bind(&FootControl::updateOptitrackPose,this,_1,P1),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[P2] = _n.subscribe<geometry_msgs::PoseStamped>("/optitrack/p2/pose", 1, boost::bind(&FootControl::updateOptitrackPose,this,_1,P2),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[P3] = _n.subscribe<geometry_msgs::PoseStamped>("/optitrack/p3/pose", 1, boost::bind(&FootControl::updateOptitrackPose,this,_1,P3),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[P4] = _n.subscribe<geometry_msgs::PoseStamped>("/optitrack/p4/pose", 1, boost::bind(&FootControl::updateOptitrackPose,this,_1,P4),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  
  _subFootOutput[RIGHT] = _n.subscribe<custom_msgs::FootOutputMsg>("/FI_Output/Right",1, boost::bind(&FootControl::updateFootOutput,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subFootOutput[LEFT] = _n.subscribe<custom_msgs::FootOutputMsg>("/FI_Output/Left",1, boost::bind(&FootControl::updateFootOutput,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

  _subFootSensor[RIGHT] = _n.subscribe<geometry_msgs::Wrench>("/FI_FFeedback/1",1, boost::bind(&FootControl::updateFootSensor,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subFootSensor[LEFT] = _n.subscribe<geometry_msgs::Wrench>("/FI_FFeedback/2",1, boost::bind(&FootControl::updateFootSensor,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
 
  // Publisher definitions
  _pubDesiredTwist[RIGHT] = _n.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 1);
  _pubDesiredOrientation[RIGHT] = _n.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);
  _pubFilteredWrench[RIGHT] = _n.advertise<geometry_msgs::WrenchStamped>("FootControl/filteredWrenchRight", 1);
  _pubNormalForce[RIGHT] = _n.advertise<std_msgs::Float32>("FootControl/normalForceRight", 1);

  _pubDesiredTwist[LEFT] = _n.advertise<geometry_msgs::Twist>("/lwr2/joint_controllers/passive_ds_command_vel", 1);
  _pubDesiredOrientation[LEFT] = _n.advertise<geometry_msgs::Quaternion>("/lwr2/joint_controllers/passive_ds_command_orient", 1);
  _pubFilteredWrench[LEFT] = _n.advertise<geometry_msgs::WrenchStamped>("FootControl/filteredWrenchLeft", 1);
  _pubNormalForce[LEFT] = _n.advertise<std_msgs::Float32>("FootControl/normalForceLeft", 1);

  _pubTaskAttractor = _n.advertise<geometry_msgs::PointStamped>("FootControl/taskAttractor", 1);  

  _pubMarker = _n.advertise<visualization_msgs::Marker>("FootControl/cube", 1);

  _pubFootInput[RIGHT] = _n.advertise<custom_msgs::FootInputMsg>("/FI_Input/Right", 1);
  _pubFootInput[LEFT] = _n.advertise<custom_msgs::FootInputMsg>("/FI_Input/Left", 1);

  _pubFootSensor[RIGHT] = _n.advertise<geometry_msgs::WrenchStamped>("/FootControl/filteredFootSensorRight", 1);
  _pubFootSensor[LEFT] = _n.advertise<geometry_msgs::WrenchStamped>("/FootControl/filteredFootSensorLeft", 1);


  // Dynamic reconfigure definition
  _dynRecCallback = boost::bind(&FootControl::dynamicReconfigureCallback, this, _1, _2);
  _dynRecServer.setCallback(_dynRecCallback);

  signal(SIGINT,FootControl::stopNode);

  _outputFile.open(ros::package::getPath(std::string("robot_shared_control"))+"/data_foot/"+_filename+".txt");

  if(!_n.getParamCached("/lwr/ds_param/damping_eigval0",_d1[RIGHT]) && _useRightRobot)
  {
    ROS_ERROR("[FootControl]: Cannot read first eigen value of passive ds controller for right robot");
    return false;
  }

  if(!_n.getParamCached("/lwr2/ds_param/damping_eigval0",_d1[LEFT]) && _useLeftRobot)
  {
    ROS_ERROR("[FootControl]: Cannot read first eigen value of passive ds controller for left robot");
    return false;
  }

  if (_n.ok()) 
  { 
    // Wait for poses being published
    ros::spinOnce();
    ROS_INFO("[FootControl]: The modulated ds node is ready.");
    return true;
  }
  else 
  {
    ROS_ERROR("[FootControl]: The ros node has a problem.");
    return false;
  }
}


void FootControl::run()
{
  _timeInit = ros::Time::now().toSec();

  while (!_stop) 
  {
    if(allDataReceived())
    {
      _mutex.lock();

      // Check for update of the DS-impedance controller gain
      ros::param::getCached("/lwr/ds_param/damping_eigval0",_d1[RIGHT]);
      ros::param::getCached("/lwr2/ds_param/damping_eigval0",_d1[LEFT]);

      // Initialize optitrack
      if(!_optitrackOK)
      {
        // optitrackInitialization();
      }
      else
      {
        // Compute object pose
        // computeObjectPose();

        if(_firstObjectPose)
        {
          // Compute control command
          computeCommand();

          // Publish data to topics
          publishData();
        }

        // Log data
        logData();
      }

      _mutex.unlock();

    }

    ros::spinOnce();

    _loopRate.sleep();
  }

  // Send zero velocity command to stop the robot
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    _vd[k].setConstant(0.0f);
    _omegad[k].setConstant(0.0f);
    _qd[k] = _q[k];  
    _desiredFootWrench[k].setConstant(0.0f);  
  }

  publishData();
  ros::spinOnce();
  _loopRate.sleep();

  _outputFile.close();
  ros::shutdown();
}

bool FootControl::allDataReceived()
{
  if(_useLeftRobot && _useRightRobot)
  {
    return (_firstRobotPose[RIGHT] && _firstRobotTwist[RIGHT] && _wrenchBiasOK[RIGHT] && _firstDampingMatrix[RIGHT] && _firstFootOutput[RIGHT] &&
            _firstRobotPose[LEFT] && _firstRobotTwist[LEFT] && _wrenchBiasOK[LEFT] && _firstDampingMatrix[LEFT] && _firstFootOutput[LEFT]);
  }
  else if(_useLeftRobot)
  {
    return (_firstRobotPose[LEFT] && _firstRobotTwist[LEFT] && _wrenchBiasOK[LEFT] && _firstDampingMatrix[LEFT] && _firstFootOutput[LEFT]);
  }
  else if(_useRightRobot)
  {
    return (_firstRobotPose[RIGHT] && _firstRobotTwist[RIGHT] && _wrenchBiasOK[RIGHT] && _firstDampingMatrix[RIGHT] && _firstFootOutput[RIGHT]);
  }
  else
  {
    return false;
  }
}

void FootControl::stopNode(int sig)
{
  me->_stop = true;
}


void FootControl::computeCommand()
{
  footDataTransformation();

  positionPositionMapping();

  positionVelocityMapping();

  // updateIndividualTasks();

  // taskAdaptation();

  Eigen::MatrixXf::Index indexMax;
  if(fabs(1.0f-_beliefs.array().maxCoeff(&indexMax))<FLT_EPSILON)
  {
    _Fd[RIGHT] = _Fdk[indexMax];
    //  if(_buttonsFalcon == (int) CENTER)
    //  {
    //   _sigmaH += 5.0f*_dt*_vM.dot(_e1);
    //   if(_sigmaH>1)
    //   {
    //     _sigmaH = 1.0f;
    //   }
    //   else if(_sigmaH<0.0f)
    //   {
    //     _sigmaH = 0.0f;
    //   }
    // }
    // _Fd*=(1+_sigmaH);
  }
  else
  {
    _Fd[RIGHT] = 0.0f;
    // _sigmaH = 0.0f;
  }

  for(int k = 0; k < NB_ROBOTS; k++)
  {
    _vd[k] = _x0[k]+_xh[k]-_x[k];

    if(_vd[k].norm()>0.3f)
    {
      _vd[k] *= 0.3f/_vd[k].norm();
    }    
  }

  // computeModulatedDS();

  computeDesiredOrientation();

  computeDesiredFootWrench();
}

void FootControl::updateIndividualTasks()
{
  for(int k = 0; k < NB_TASKS; k++)
  {
    _fxk[k] = _xAttractor[k]-_x[RIGHT];
    _Fdk[k] = 0.0f;
  }

  Eigen::Vector3f F = _filteredWrench[RIGHT].segment(0,3);
  _normalForce[RIGHT] = _e1[RIGHT].dot(-_wRb[RIGHT]*F);
    // Compute desired force profile
  if(_normalForce[RIGHT]<3.0f)
  {
    _Fdk[0] = 5.0f;
  }
  else
  {
    _Fdk[0] = _targetForce;
  }

}

void FootControl::taskAdaptation()
{
  _fx[RIGHT].setConstant(0.0f);
  for(int k = 0; k < NB_TASKS; k++)
  {
    _fx[RIGHT]+=_beliefs[k]*_fxk[k];
  }

  for(int k = 0; k < NB_TASKS; k++)
  {
    _dbeliefs[k] = _adaptationRate*((_vh[RIGHT]-_fx[RIGHT]).dot(_fxk[k])+(_beliefs[k]-0.5f)*_fxk[k].squaredNorm());
    std::cerr << k << " " << (_vh[RIGHT]-_fx[RIGHT]).dot(_fxk[k]) << " " << (_beliefs[k]-0.5f)*_fxk[k].squaredNorm() <<std::endl;
  }

  Eigen::MatrixXf::Index indexMax;
  float bmax = _dbeliefs.array().maxCoeff(&indexMax);
  if(fabs(1.0f-bmax)< FLT_EPSILON)
  {
    _dbeliefs(indexMax) = 0.0f;
  }

  Eigen::Matrix<float,NB_TASKS-1,1> temp;
  int m = 0;
  for(int k = 0; k < NB_TASKS; k++)
  {
    if(k!=indexMax)
    {
      temp[m] = _dbeliefs[k];
      m++;
    }
  }
  float b2max = temp.array().maxCoeff();
  float z = (bmax+b2max)/2.0f;
  _dbeliefs.array() -= z;

  float S = 0.0f;
  for(int k = 0; k < NB_TASKS; k++)
  {
    if(fabs(_beliefs[k])>FLT_EPSILON || _dbeliefs[k] > 0)
    {
      S+=_dbeliefs[k];
    }
  }
  _dbeliefs[indexMax]-=S;
  _beliefs+=_dt*_dbeliefs;
  for(int k = 0; k < NB_TASKS; k++)
  {
    if(_beliefs[k]< 0.0f)
    {
      _beliefs[k] = 0.0f;
    }
    else if(_beliefs[k] > 1.0f)
    {
      _beliefs[k] = 1.0f;
    }
  }

  std::cerr << "dBeliefs: " << _dbeliefs.transpose() << std::endl;
  std::cerr << "beliefs: " << _beliefs.transpose() << std::endl;
}



void FootControl::footDataTransformation()
{
  _footPosition[RIGHT](0) = _footPose[RIGHT](1);
  _footPosition[RIGHT](1) = -_footPose[RIGHT](0);
  _footPosition[RIGHT](2) = _footPose[RIGHT](3);
  _footPosition[LEFT](0) = _footPose[LEFT](1);
  _footPosition[LEFT](1) = -_footPose[LEFT](0);
  _footPosition[LEFT](2) = _footPose[LEFT](3);

  // std::cerr << "Before transformation: " <<_footPose.segment(0,4).transpose() << std::endl;
  // std::cerr << "After transformation: " << _footPosition.transpose() << std::endl;
}


void FootControl::positionPositionMapping()
{

  Eigen::Vector3f gains[NB_ROBOTS];
  gains[RIGHT] << 2*_xyPositionMapping/FOOT_INTERFACE_Y_RANGE_RIGHT, 2*_xyPositionMapping/FOOT_INTERFACE_X_RANGE_RIGHT, 2*_zPositionMapping/FOOT_INTERFACE_PHI_RANGE_RIGHT;
  gains[LEFT] << 2*_xyPositionMapping/FOOT_INTERFACE_Y_RANGE_LEFT, 2*_xyPositionMapping/FOOT_INTERFACE_X_RANGE_LEFT, 2*_zPositionMapping/FOOT_INTERFACE_PHI_RANGE_LEFT;
  
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    _xh[k] = gains[k].cwiseProduct(_footPosition[k]);
    std::cerr << "Master position " << k << " : " <<_xh[k].transpose() << std::endl;
  }
}

void FootControl::positionVelocityMapping()
{

  _velocityLimit = 0.3f;
  Eigen::Vector3f gains[NB_ROBOTS];
  gains[RIGHT] << 2.0f*_velocityLimit/FOOT_INTERFACE_Y_RANGE_RIGHT, 2.0f*_velocityLimit/FOOT_INTERFACE_X_RANGE_RIGHT, 2.0f*_velocityLimit/FOOT_INTERFACE_PHI_RANGE_RIGHT;
  gains[LEFT] << 2.0f*_velocityLimit/FOOT_INTERFACE_Y_RANGE_LEFT, 2.0f*_velocityLimit/FOOT_INTERFACE_X_RANGE_LEFT, 2.0f*_velocityLimit/FOOT_INTERFACE_PHI_RANGE_LEFT;
  
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    _vh[k] = gains[k].cwiseProduct(_footPosition[k]);
    if(_vh[k].norm()>_velocityLimit)
    {
      _vh[k] *= _velocityLimit/_vh[k].norm();
    }    
    // std::cerr << "Master velocity " << k << " : " <<_vh[k].transpose() << std::endl;
  }
}


void FootControl::computeDesiredFootWrench()
{
  Eigen::Vector3f temp;
  temp = _wRb[RIGHT]*_filteredWrench[RIGHT].segment(0,3);

  // temp.setConstant(0.0f);
  _desiredFootWrench[RIGHT](1) = temp(0);
  _desiredFootWrench[RIGHT](0) = -temp(1);
  _desiredFootWrench[RIGHT](3) = temp(2)*0.205/5;
  // _desiredFootWrench[RIGHT](0) += -_kxy*_footPose[RIGHT](0)-_dxy*_footTwist[RIGHT](0);
  // _desiredFootWrench[RIGHT](1) += -_kxy*_footPose[RIGHT](1)-_dxy*_footTwist[RIGHT](1);
  // _desiredFootWrench[RIGHT](3) += -_kphi*_footPose[RIGHT](3)-_dphi*_footTwist[RIGHT](3);

  temp = _wRb[LEFT]*_filteredWrench[LEFT].segment(0,3);
  _desiredFootWrench[LEFT](1) = temp(0);
  _desiredFootWrench[LEFT](0) = -temp(1);
  _desiredFootWrench[LEFT](3) = temp(2)*0.205/5;

  for(int k = 0 ; k < 3; k++)
  {
    if(_desiredFootWrench[RIGHT](k)>25.0f)
    {
      _desiredFootWrench[RIGHT](k) = 25.0f;
    }
    else if(_desiredFootWrench[RIGHT](k)<-25.0f)
    {
      _desiredFootWrench[RIGHT](k) = -25.0f;
    }

    if(_desiredFootWrench[LEFT](k)>25.0f)
    {
      _desiredFootWrench[LEFT](k) = 25.0f;
    }
    else if(_desiredFootWrench[LEFT](k)<-25.0f)
    {
      _desiredFootWrench[LEFT](k) = -25.0f;
    }
  }

  for(int k = 0 ; k < 3; k++)
  {
    if(_desiredFootWrench[RIGHT](k+3)>0.187f*40/9.15)
    {
      _desiredFootWrench[RIGHT](k+3) = 0.187f*40/9.15;
    }
    else if(_desiredFootWrench[RIGHT](k+3)<-0.187f*40/9.15)
    {
      _desiredFootWrench[RIGHT](k+3) = -0.187f*40/9.15;
    }

    if(_desiredFootWrench[LEFT](k+3)>0.212f*40/9.15)
    {
      _desiredFootWrench[LEFT](k+3) = 0.212f*40/9.15;
    }
    else if(_desiredFootWrench[LEFT](k+3)<-0.212f*40/9.15)
    {
      _desiredFootWrench[LEFT](k+3) = -0.212f*40/9.15;
    }

  }
}


void FootControl::computeModulatedDS()
{
  for(int k = 0; k < NB_ROBOTS; k++)
  {

    // Compute modulation gain
    if(_d1[k]<1.0f)
    {
      _d1[k] = 1.0f;
    }

    float delta = std::pow(2.0f*_e1[k].dot(_fx[k])*(_Fd[k]/_d1[k]),2.0f)+4.0f*std::pow(_fx[k].norm(),4.0f); 
    float la;

    if(fabs(_fx[k].norm())<FLT_EPSILON)
    {
      la = 0.0f;
    }
    else if(_e1[k].dot(_fx[k])<0.0f)
    {
      la = 1.0f;
    }
    else
    {
      la = (-2.0f*_e1[k].dot(_fx[k])*(_Fd[k]/_d1[k])+sqrt(delta))/(2.0f*std::pow(_fx[k].norm(),2.0f));
    }

    // // Update tank dynamics
    // _pd[k] = _v[k].transpose()*_D[k]*_v[k]; 
    // float ds = _dt*(_alpha[k]*_pd[k]-_beta[k]*(la-1.0f)*_pn[k]-_gamma[k]*_pf[k]);

    // if(_s[k]+ds>=_smax)
    // {
    //   _s[k] = _smax;
    // }
    // else if(_s[k]+ds<=0.0f)
    // {
    //   _s[k] = 0.0f;
    // }
    // else
    // {
    //   _s[k]+=ds;
    // }

    // // Update robot's power flow
    // _dW[k] = (la-1.0f)*(1-_beta[k])*_pn[k]+(_gammap[k]-_gamma[k])*_pf[k]-(1-_alpha[k])*_pd[k];

    // Comput modulated DS
    _vd[k] = la*_fx[k]+_Fd[k]*_e1[k]/_d1[k];

    std::cerr << "Robot " << k << ": Fd: " << _Fd[k] << ": Fm: " << _normalForce[k] << " delta: " << delta << " la: " << la << " vdr.dot(e1) " << _e1[k].dot(_fx[k]) << std::endl;

    // Bound desired velocity for safety
    if(_vd[k].norm()>0.3f)
    {
      _vd[k] *= 0.3f/_vd[k].norm();
    }   
  }
}


void FootControl::computeDesiredOrientation()
{
 for(int k = 0; k < NB_ROBOTS; k++)
  {
    Eigen::Matrix3f Rd;
    if(k == RIGHT)
    {
      Rd << -1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f,
            0.0f, 1.0f, 0.0f;
    }
    else
    {
      Rd << -1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, -1.0f,
            0.0f, -1.0f, 0.0f;
    }

    _qd[k] = Utils::rotationMatrixToQuaternion(Rd);

    if(_q[k].dot(_qd[k])<0)
    {
      _qd[k] *=-1.0f;
    }

    //   Rd << -1.0f, 0.0f, 0.0f,
    //         0.0f, 1.0f, 0.0f,
    //         0.0f, 0.0f, -1.0f;
    // Eigen::Matrix3f Rx;
    // float angle = _footPose[RIGHT](4)*M_PI/180.0f;
    // Rx << 1.0f, 0.0f, 0.0f,
    //       0.0f, cos(angle), -sin(angle),
    //       0.0f, sin(angle), cos(angle);

    // Rd = Rx*Rd;
    // std::cerr << Rd << std::endl;

    // _qd[k] = Utils::rotationMatrixToQuaternion(Rd);

    // if(_q[k].dot(_qd[k])<0)
    // {
    //   _qd[k] *=-1.0f;
    // }


     // _qd[k] << 0.0f,0.0f,1.0f,0.0f;

    // Compute needed angular velocity to perform the desired quaternion
    Eigen::Vector4f qcurI, wq;
    qcurI(0) = _q[k](0);
    qcurI.segment(1,3) = -_q[k].segment(1,3);
    wq = 5.0f*Utils::quaternionProduct(qcurI,_qd[k]-_q[k]);
    Eigen::Vector3f omegaTemp = _wRb[k]*wq.segment(1,3);
    _omegad[k] = omegaTemp; 
  }
}


void FootControl::logData()
{
  _outputFile << ros::Time::now() << " "
              << _x[LEFT].transpose() << " "
              << _vd[LEFT].transpose() << " "
              << (_wRb[LEFT]*_filteredWrench[LEFT].segment(0,3)).transpose() << " "
              << (_wRb[LEFT]*_filteredWrench[LEFT].segment(3,3)).transpose() << " "
              << _x[RIGHT].transpose() << " "
              << _vd[RIGHT].transpose() << " "
              << (_wRb[RIGHT]*_filteredWrench[RIGHT].segment(0,3)).transpose() << " "
              << (_wRb[RIGHT]*_filteredWrench[RIGHT].segment(3,3)).transpose() << " "
              << _footPose[LEFT](0) << " "
              << _footPose[LEFT](1) << " "
              << _footPose[LEFT](3) << " "
              << _footWrench[LEFT](0) << " "
              << _footWrench[LEFT](1) << " "
              << _footWrench[LEFT](3) << " "
              << _desiredFootWrench[LEFT](0) << " "
              << _desiredFootWrench[LEFT](1) << " "
              << _desiredFootWrench[LEFT](3) << " "
              << _footState[LEFT] << " "
              << _footPose[RIGHT](0) << " "
              << _footPose[RIGHT](1) << " "
              << _footPose[RIGHT](3) << " "
              << _footWrench[RIGHT](0) << " "
              << _footWrench[RIGHT](1) << " "
              << _footWrench[RIGHT](3) << " "
              << _desiredFootWrench[RIGHT](0) << " "
              << _desiredFootWrench[RIGHT](1) << " "
              << _desiredFootWrench[RIGHT](3) << " "
              << _footState[RIGHT] << " "
              << _footSensor[RIGHT].transpose() << " "
              << _filteredFootSensor[RIGHT].transpose() << std::endl;
}


void FootControl::publishData()
{
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    // Publish desired twist (passive ds controller)
    _msgDesiredTwist.linear.x  = _vd[k](0);
    _msgDesiredTwist.linear.y  = _vd[k](1);
    _msgDesiredTwist.linear.z  = _vd[k](2);

    // Convert desired end effector frame angular velocity to world frame
    _msgDesiredTwist.angular.x = _omegad[k](0);
    _msgDesiredTwist.angular.y = _omegad[k](1);
    _msgDesiredTwist.angular.z = _omegad[k](2);

    _pubDesiredTwist[k].publish(_msgDesiredTwist);

    // Publish desired orientation
    _msgDesiredOrientation.w = _qd[k](0);
    _msgDesiredOrientation.x = _qd[k](1);
    _msgDesiredOrientation.y = _qd[k](2);
    _msgDesiredOrientation.z = _qd[k](3);

    _pubDesiredOrientation[k].publish(_msgDesiredOrientation);

    _msgFilteredWrench.header.frame_id = "world";
    _msgFilteredWrench.header.stamp = ros::Time::now();
    _msgFilteredWrench.wrench.force.x = _filteredWrench[k](0);
    _msgFilteredWrench.wrench.force.y = _filteredWrench[k](1);
    _msgFilteredWrench.wrench.force.z = _filteredWrench[k](2);
    _msgFilteredWrench.wrench.torque.x = _filteredWrench[k](3);
    _msgFilteredWrench.wrench.torque.y = _filteredWrench[k](4);
    _msgFilteredWrench.wrench.torque.z = _filteredWrench[k](5);
    _pubFilteredWrench[k].publish(_msgFilteredWrench);

    std_msgs::Float32 msg;
    msg.data = _normalForce[k];
    _pubNormalForce[k].publish(msg); 

    _msgFootSensor.header.frame_id = "world";
    _msgFootSensor.header.stamp = ros::Time::now();
    _msgFootSensor.wrench.force.x = _filteredFootSensor[k](0);
    _msgFootSensor.wrench.force.y = _filteredFootSensor[k](1);
    _msgFootSensor.wrench.force.z = _filteredFootSensor[k](2);
    _msgFootSensor.wrench.torque.x = _filteredFootSensor[k](3);
    _msgFootSensor.wrench.torque.y = _filteredFootSensor[k](4);
    _msgFootSensor.wrench.torque.z = _filteredFootSensor[k](5);
    _pubFootSensor[k].publish(_msgFootSensor);
    
    _msgFootInput.FxDes = _desiredFootWrench[k](0);
    _msgFootInput.FyDes = _desiredFootWrench[k](1);
    _msgFootInput.TphiDes = _desiredFootWrench[k](3);
    _msgFootInput.TthetaDes = _desiredFootWrench[k](4);
    _msgFootInput.TpsiDes = _desiredFootWrench[k](5);
    _msgFootInput.stateDes = 2;
    _pubFootInput[k].publish(_msgFootInput);

  }

  _msgMarker.header.frame_id = "world";
  _msgMarker.header.stamp = ros::Time();
  _pubMarker.publish(_msgMarker);

  _msgTaskAttractor.header.frame_id = "world";
  _msgTaskAttractor.header.stamp = ros::Time::now();
  _msgTaskAttractor.point.x = _taskAttractor(0);
  _msgTaskAttractor.point.y = _taskAttractor(1);
  _msgTaskAttractor.point.z = _taskAttractor(2);
  _pubTaskAttractor.publish(_msgTaskAttractor);

}


void FootControl::updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg, int k)
{

  Eigen::Vector3f temp = _x[k];

  // Update end effecotr pose (position+orientation)
  _x[k] << msg->position.x, msg->position.y, msg->position.z;
  _q[k] << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
  _wRb[k] = Utils::quaternionToRotationMatrix(_q[k]);
  _x[k] = _x[k]+_toolOffsetFromEE*_wRb[k].col(2);

  if(k==(int)LEFT)
  {
    _x[k] += _leftRobotOrigin;
  }

  if(!_firstRobotPose[k])
  {
    _firstRobotPose[k] = true;
    _xd[k] = _x[k];
    // _x0[k] = _x[k];
    _qd[k] = _q[k];
    _vd[k].setConstant(0.0f);
  }
}


void FootControl::updateRobotTwist(const geometry_msgs::Twist::ConstPtr& msg, int k)
{
  _v[k] << msg->linear.x, msg->linear.y, msg->linear.z;
  _w[k] << msg->angular.x, msg->angular.y, msg->angular.z;

  if(!_firstRobotTwist[k])
  {
    _firstRobotTwist[k] = true;
  }
}
 

void FootControl::updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg, int k)
{
  Eigen::Matrix<float,6,1> raw;
  raw(0) = msg->wrench.force.x;
  raw(1) = msg->wrench.force.y;
  raw(2) = msg->wrench.force.z;
  raw(3) = msg->wrench.torque.x;
  raw(4) = msg->wrench.torque.y;
  raw(5) = msg->wrench.torque.z;

  if(!_wrenchBiasOK[k] && _firstRobotPose[k])
  {
    Eigen::Vector3f loadForce = _wRb[k].transpose()*_toolMass*_gravity;
    _wrenchBias[k].segment(0,3) -= loadForce;
    _wrenchBias[k].segment(3,3) -= _toolComPositionFromSensor.cross(loadForce);
    _wrenchBias[k] += raw; 
    _wrenchCount[k]++;
    if(_wrenchCount[k]==NB_SAMPLES)
    {
      _wrenchBias[k] /= NB_SAMPLES;
      _wrenchBiasOK[k] = true;
      std::cerr << "[FootControl]: Bias " << k << ": " <<_wrenchBias[k].transpose() << std::endl;
    }
  }

  if(_wrenchBiasOK[k] && _firstRobotPose[k])
  {
    _wrench[k] = raw-_wrenchBias[k];
    Eigen::Vector3f loadForce = _wRb[k].transpose()*_toolMass*_gravity;
    _wrench[k].segment(0,3) -= loadForce;
    _wrench[k].segment(3,3) -= _toolComPositionFromSensor.cross(loadForce);
    _filteredWrench[k] = _filteredForceGain*_filteredWrench[k]+(1.0f-_filteredForceGain)*_wrench[k];
  }
}


void FootControl::updateDampingMatrix(const std_msgs::Float32MultiArray::ConstPtr& msg, int k) 
{
  if(!_firstDampingMatrix[k])
  {
    _firstDampingMatrix[k] = true;
  }

  _D[k] << msg->data[0],msg->data[1],msg->data[2],
           msg->data[3],msg->data[4],msg->data[5],
           msg->data[6],msg->data[7],msg->data[8];
}

void FootControl::optitrackInitialization()
{
  if(_averageCount< AVERAGE_COUNT)
  {
    if(_markersTracked(ROBOT_BASIS_RIGHT) && _markersTracked(ROBOT_BASIS_LEFT))
    {
      _markersPosition0 = (_averageCount*_markersPosition0+_markersPosition)/(_averageCount+1);
      _averageCount++;
    }
    std::cerr << "[FootControl]: Optitrack Initialization count: " << _averageCount << std::endl;
    if(_averageCount == 1)
    {
      ROS_INFO("[FootControl]: Optitrack Initialization starting ...");
    }
    else if(_averageCount == AVERAGE_COUNT)
    {
      ROS_INFO("[FootControl]: Optitrack Initialization done !");
    }
  }
  else
  {
    _optitrackOK = true;
  }
}

void FootControl::updateOptitrackPose(const geometry_msgs::PoseStamped::ConstPtr& msg, int k) 
{
  if(!_firstOptitrackPose[k])
  {
    _firstOptitrackPose[k] = true;
  }

  _markersSequenceID(k) = msg->header.seq;
  _markersTracked(k) = checkTrackedMarker(_markersPosition.col(k)(0),msg->pose.position.x);
  _markersPosition.col(k) << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  if(k == (int) ROBOT_BASIS_RIGHT || k == (int) ROBOT_BASIS_LEFT)
  {
    _markersPosition.col(k)(2) -= 0.03f;
  }
}


void FootControl::updateFootInterfacePose(const geometry_msgs::PoseStamped::ConstPtr& msg, int k)
{

  Eigen::Matrix<float,6,1> temp;
  temp = _footPose[k];
  _footPose[k] << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;

  if(_footInterfaceSequenceID[k]!=msg->header.seq)
  {
    _footInterfaceSequenceID[k] = msg->header.seq;
  }

  if(!_firstFootInterfacePose[k])
  {
    _firstFootInterfacePose[k] = true;
  }
}


void FootControl::updateFootInterfaceWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg, int k)
{

  Eigen::Matrix<float,6,1> temp;
  temp = _footPose[k];
  _footWrench[k] << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z, msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;

  if(!_firstFootInterfaceWrench[k])
  {
    _firstFootInterfaceWrench[k] = true;
  }
}


void FootControl::updateFootOutput(const custom_msgs::FootOutputMsg::ConstPtr& msg, int k)
{

  _footPose[k] << msg->x, msg->y,0.0f, msg->phi, msg->theta, msg->psi;
  _footWrench[k] << msg->Fx, msg->Fy,0.0f, msg->Tphi, msg->Ttheta, msg->Tpsi;
  _footTwist[k] << msg->vx, msg->vy, 0.0f, msg->wphi, msg->wtheta, msg->wpsi;
  _footState[k] = msg->state;

  if(!_firstFootOutput[k])
  {
    _firstFootOutput[k] = true;
  }
}

void FootControl::updateFootSensor(const geometry_msgs::Wrench::ConstPtr& msg, int k)
{

  _footSensor[k] << msg->force.x, msg->force.y ,msg->force.z, msg->torque.x, msg->torque.y, msg->torque.z;

  if(!_firstFootSensor[k])
  {
    _firstFootSensor[k] = true;
  }

  _filteredFootSensor[k] = _filteredForceGain*_filteredFootSensor[k]+(1.0f-_filteredForceGain)*_footSensor[k];

}


uint16_t FootControl::checkTrackedMarker(float a, float b)
{
  if(fabs(a-b)< FLT_EPSILON)
  {
    return 0;
  }
  else
  {
    return 1;
  }
}


void FootControl::dynamicReconfigureCallback(robot_shared_control::footControl_paramsConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure request. Updatig the parameters ...");

  _filteredForceGain = config.filteredForceGain;
  _velocityLimit = config.velocityLimit;
  _duration = config.duration;
  _kxy = config.kxy;
  _dxy = config.dxy;
  _kphi = config.kphi;
  _dphi = config.dphi;
  _xyPositionMapping = config.xyPositionMapping;
  _zPositionMapping = config.zPositionMapping;
}

// void FootControl::computeObjectPose()
// {
//   // Check if all markers on the object are tracked
//   // The four markers are positioned on the corner of the upper face:
//   // P2 ----- P3
//   // |        |
//   // |        |
//   // P1 ----- P4
//   if(_markersTracked.segment(NB_ROBOTS,TOTAL_NB_MARKERS-NB_ROBOTS).sum() == TOTAL_NB_MARKERS-NB_ROBOTS)
//   {

//     if(!_firstObjectPose)
//     {
//       _firstObjectPose = true;
//     }

//     // Compute markers position in the right robot frame
//     _p1 = _markersPosition.col(P1)-_markersPosition0.col(ROBOT_BASIS_RIGHT);
//     _p2 = _markersPosition.col(P2)-_markersPosition0.col(ROBOT_BASIS_RIGHT);
//     _p3 = _markersPosition.col(P3)-_markersPosition0.col(ROBOT_BASIS_RIGHT);
//     _p4 = _markersPosition.col(P4)-_markersPosition0.col(ROBOT_BASIS_RIGHT);

//     // Compute object center position
//     _xoC = (_p1+_p2+_p3+_p4)/4.0f;
//     // Compute object dimension vector
//     // The dimension obtained from the markers is adjusted to match the real
//     // object dimension
//     _xoD = (_p3+_p4-_p1-_p2)/2.0f;
//     _xoD = 0.20f*_xoD.normalized(); 

//     // Filter center position and dimension vector of the object
//     SGF::Vec temp(3);
//     _xCFilter.AddData(_xoC);
//     _xCFilter.GetOutput(0,temp);
//     _xoC = temp;
//     Eigen::Vector3f xDir = _p2-_p1;
//     xDir.normalize();
//     Eigen::Vector3f yDir = _p1-_p4;
//     yDir.normalize();
//     Eigen::Vector3f zDir = xDir.cross(yDir);
//     zDir.normalize();
//     _zDirFilter.AddData(xDir.cross(yDir));
//     _zDirFilter.GetOutput(0,temp);
//     zDir = temp;
//     zDir.normalize();   
//     _xoC -= 1.0f*(_objectDim(2)/2.0f)*zDir;
      
//     // Filter object direction
//     _xDFilter.AddData(_xoD);
//     _xDFilter.GetOutput(0,temp);
//     _xoD = 0.20f*temp.normalized();

//     // std::cerr <<"real" << _xdD.norm() << " " <<_xdD.transpose() << std::endl;
//     // std::cerr << "filter" <<  _xoD.norm() << " " <<_xoD.transpose() << std::endl;

//     _msgMarker.pose.position.x = _xoC(0);
//     _msgMarker.pose.position.y = _xoC(1);
//     _msgMarker.pose.position.z = _xoC(2);
//     Eigen::Matrix3f R;
//     R.col(0) = xDir;
//     R.col(1) = yDir;
//     R.col(2) = zDir;
//     Eigen::Vector4f q = Utils::rotationMatrixToQuaternion(R);
//     _msgMarker.pose.orientation.x = q(1);
//     _msgMarker.pose.orientation.y = q(2);
//     _msgMarker.pose.orientation.z = q(3);
//     _msgMarker.pose.orientation.w = q(0);

//   }

// }