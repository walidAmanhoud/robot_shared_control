#include "FeetTelemanipulation.h"
#include "Utils.h"

FeetTelemanipulation* FeetTelemanipulation::me = NULL;

FeetTelemanipulation::FeetTelemanipulation(ros::NodeHandle &n, double frequency, std::string filename):
  _n(n),
  _loopRate(frequency),
  _dt(1.0f/frequency),
  _filename(filename)
{
  me = this;

  _useLeftRobot = true;
  _useRightRobot = true;

  _gravity << 0.0f, 0.0f, -9.80665f;
  _toolComPositionFromSensor << 0.0f,0.0f,0.02f;
  _toolOffsetFromEE = 0.13f;
  _toolMass = 0.2f;

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
    _Fd[k] = 0.0f;

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
    _footPosition[k].setConstant(0.0f);
    _xdFoot[k].setConstant(0.0f);
    _vdFoot[k].setConstant(0.0f);
    _footInterfaceSequenceID[k] = 0;    
    _firstFootInterfacePose[k] = false;
    _firstFootOutput[k] = false;
    _desiredFootWrench[k].setConstant(0.0f);
    _FdFoot[k].setConstant(0.0f);
    _normalForceAverage[k] = 0.0f;
  }

  _stop = false;
  _leftRobotOrigin << 0.066f, 0.9f, 0.0f;
  _x0[LEFT](0) = _leftRobotOrigin(0)-0.60f;
  _x0[LEFT](1) = _leftRobotOrigin(1)-0.35f;
  _x0[LEFT](2) = _leftRobotOrigin(2)+0.45f;
  _x0[RIGHT](0) = -0.52f;
  _x0[RIGHT](1) = 0.4f;
  _x0[RIGHT](2) = 0.45f;
  _graspingForceThreshold = 4.0f;  // Grasping force threshold [N]
  _objectGrasped = false;
  _targetForce = 15.0f;
  
  _velocityLimit = 0.4f;
  _kxy = 0.0f;
  _dxy = 0.0f;
  _kphi = 0.0f;
  _dphi = 0.0f;

  // _strategy = AUTONOMOUS_LOAD_SUPPORT;
  _strategy = PURE_TELEMANIPULATION;
}


bool FeetTelemanipulation::init() 
{
  // Subscriber definitions
  _subRobotPose[LEFT] = _n.subscribe<geometry_msgs::Pose>("/lwr2/ee_pose", 1, boost::bind(&FeetTelemanipulation::updateRobotPose,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subRobotTwist[LEFT] = _n.subscribe<geometry_msgs::Twist>("/lwr2/joint_controllers/twist", 1, boost::bind(&FeetTelemanipulation::updateRobotTwist,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subDampingMatrix[LEFT] = _n.subscribe<std_msgs::Float32MultiArray>("/lwr2/joint_controllers/passive_ds_damping_matrix", 1, boost::bind(&FeetTelemanipulation::updateDampingMatrix,this,_1,LEFT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subForceTorqueSensor[LEFT] = _n.subscribe<geometry_msgs::WrenchStamped>("/ft_sensor_left/netft_data", 1, boost::bind(&FeetTelemanipulation::updateRobotWrench,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subFootOutput[LEFT] = _n.subscribe<custom_msgs::FootOutputMsg>("/FI_Output/Left",1, boost::bind(&FeetTelemanipulation::updateFootOutput,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

  _subRobotPose[RIGHT] = _n.subscribe<geometry_msgs::Pose>("/lwr/ee_pose", 1, boost::bind(&FeetTelemanipulation::updateRobotPose,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subRobotTwist[RIGHT] = _n.subscribe<geometry_msgs::Twist>("/lwr/joint_controllers/twist", 1, boost::bind(&FeetTelemanipulation::updateRobotTwist,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subDampingMatrix[RIGHT] = _n.subscribe<std_msgs::Float32MultiArray>("/lwr/joint_controllers/passive_ds_damping_matrix", 1, boost::bind(&FeetTelemanipulation::updateDampingMatrix,this,_1,RIGHT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subForceTorqueSensor[RIGHT] = _n.subscribe<geometry_msgs::WrenchStamped>("/ft_sensor_right/netft_data", 1, boost::bind(&FeetTelemanipulation::updateRobotWrench,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subFootOutput[RIGHT] = _n.subscribe<custom_msgs::FootOutputMsg>("/FI_Output/Right",1, boost::bind(&FeetTelemanipulation::updateFootOutput,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
 
  // Publisher definitions
  _pubDesiredTwist[LEFT] = _n.advertise<geometry_msgs::Twist>("/lwr2/joint_controllers/passive_ds_command_vel", 1);
  _pubDesiredOrientation[LEFT] = _n.advertise<geometry_msgs::Quaternion>("/lwr2/joint_controllers/passive_ds_command_orient", 1);
  _pubFilteredWrench[LEFT] = _n.advertise<geometry_msgs::WrenchStamped>("FeetTelemanipulation/filteredWrenchLeft", 1);
  _pubNormalForce[LEFT] = _n.advertise<std_msgs::Float32>("FeetTelemanipulation/normalForceLeft", 1);
  _pubFootInput[LEFT] = _n.advertise<custom_msgs::FootInputMsg>("/FI_Input/Left", 1);

  _pubDesiredTwist[RIGHT] = _n.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 1);
  _pubDesiredOrientation[RIGHT] = _n.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);
  _pubFilteredWrench[RIGHT] = _n.advertise<geometry_msgs::WrenchStamped>("FeetTelemanipulation/filteredWrenchRight", 1);
  _pubNormalForce[RIGHT] = _n.advertise<std_msgs::Float32>("FeetTelemanipulation/normalForceRight", 1);
  _pubFootInput[RIGHT] = _n.advertise<custom_msgs::FootInputMsg>("/FI_Input/Right", 1);

  // Dynamic reconfigure definition
  _dynRecCallback = boost::bind(&FeetTelemanipulation::dynamicReconfigureCallback, this, _1, _2);
  _dynRecServer.setCallback(_dynRecCallback);

  signal(SIGINT,FeetTelemanipulation::stopNode);

  _outputFile.open(ros::package::getPath(std::string("robot_shared_control"))+"/data_foot/"+_filename+".txt");

  if(!_n.getParamCached("/lwr/ds_param/damping_eigval0",_d1[RIGHT]) && _useRightRobot)
  {
    ROS_ERROR("[FeetTelemanipulation]: Cannot read first eigen value of passive ds controller for right robot");
    return false;
  }

  if(!_n.getParamCached("/lwr2/ds_param/damping_eigval0",_d1[LEFT]) && _useLeftRobot)
  {
    ROS_ERROR("[FeetTelemanipulation]: Cannot read first eigen value of passive ds controller for left robot");
    return false;
  }

  if (_n.ok()) 
  { 
    // Wait for poses being published
    ros::spinOnce();
    ROS_INFO("[FeetTelemanipulation]: The FeetTelemanipulation node is ready.");
    return true;
  }
  else 
  {
    ROS_ERROR("[FeetTelemanipulation]: The ros node has a problem.");
    return false;
  }
}


void FeetTelemanipulation::run()
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
          
      // Compute control command
      computeCommand();

      // Publish data to topics
      publishData();

      // Log data
      logData();

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


void FeetTelemanipulation::stopNode(int sig)
{
  me->_stop = true;
}


bool FeetTelemanipulation::allDataReceived()
{
  if(_useLeftRobot && _useRightRobot)
  {
    // std::cerr <<(int) _firstRobotPose[RIGHT] << (int) _firstRobotTwist[RIGHT] << (int) _wrenchBiasOK[RIGHT] << (int) _firstDampingMatrix[RIGHT] << (int) _firstFootOutput[RIGHT] << (int)
    //         _firstRobotPose[LEFT] << (int) _firstRobotTwist[LEFT] << (int) _wrenchBiasOK[LEFT] << (int) _firstDampingMatrix[LEFT] << (int) _firstFootOutput[LEFT] << std::endl;
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


void FeetTelemanipulation::computeCommand()
{
  footDataTransformation();

  positionPositionMapping();

  positionVelocityMapping();

  updateObjectGraspingState();

  switch(_strategy)
  {
    case PURE_TELEMANIPULATION:
    {
      pureTelemanipulation();
      break;
    }
    case AUTONOMOUS_LOAD_SUPPORT:
    {
      autonomousLoadSupport();
      break;
    }
    default:
    {
      _vd[LEFT].setConstant(0.0f);  
      _vd[RIGHT].setConstant(0.0f);  

      break;
    }
  }

  computeDesiredOrientation();

  computeDesiredFootWrench();
}


void FeetTelemanipulation::footDataTransformation()
{
  _footPosition[RIGHT](0) = _footPose[RIGHT](1);
  _footPosition[RIGHT](1) = -_footPose[RIGHT](0);
  _footPosition[RIGHT](2) = _footPose[RIGHT](3);
  _footPosition[LEFT](0) = _footPose[LEFT](1);
  _footPosition[LEFT](1) = -_footPose[LEFT](0);
  _footPosition[LEFT](2) = _footPose[LEFT](3);
}

void FeetTelemanipulation::positionPositionMapping()
{
  Eigen::Vector3f gains[NB_ROBOTS];
  gains[RIGHT] << 2*_xyPositionMapping/FOOT_INTERFACE_Y_RANGE_RIGHT, 2*_xyPositionMapping/FOOT_INTERFACE_X_RANGE_RIGHT, 2*_zPositionMapping/FOOT_INTERFACE_PHI_RANGE_RIGHT;
  gains[LEFT] << 2*_xyPositionMapping/FOOT_INTERFACE_Y_RANGE_LEFT, 2*_xyPositionMapping/FOOT_INTERFACE_X_RANGE_LEFT, 2*_zPositionMapping/FOOT_INTERFACE_PHI_RANGE_LEFT;
  
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    _xdFoot[k] = gains[k].cwiseProduct(_footPosition[k]);
    // std::cerr << "Master position " << k << " : " <<_xdFoot[k].transpose() << std::endl;
  }
}


void FeetTelemanipulation::positionVelocityMapping()
{

  _velocityLimit = 0.3f;
  Eigen::Vector3f gains[NB_ROBOTS];
  gains[RIGHT] << 2.0f*_velocityLimit/FOOT_INTERFACE_Y_RANGE_RIGHT, 2.0f*_velocityLimit/FOOT_INTERFACE_X_RANGE_RIGHT, 2.0f*_velocityLimit/FOOT_INTERFACE_PHI_RANGE_RIGHT;
  gains[LEFT] << 2.0f*_velocityLimit/FOOT_INTERFACE_Y_RANGE_LEFT, 2.0f*_velocityLimit/FOOT_INTERFACE_X_RANGE_LEFT, 2.0f*_velocityLimit/FOOT_INTERFACE_PHI_RANGE_LEFT;
  
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    _vdFoot[k] = gains[k].cwiseProduct(_footPosition[k]);
    if(_vdFoot[k].norm()>_velocityLimit)
    {
      _vdFoot[k] *= _velocityLimit/_vdFoot[k].norm();
    }    
    // std::cerr << "Master velocity " << k << " : " <<_vdFoot[k].transpose() << std::endl;
  }
}


void FeetTelemanipulation::updateObjectGraspingState()
{
  Eigen::Vector3f temp;
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    _normalForce[k] = (_wRb[k]*_filteredWrench[k].segment(0,3)).dot(-_wRb[k].col(2));
    if(_normalForceWindow[k].size()<WINDOW_SIZE)
    {
      _normalForceWindow[k].push_back(_normalForce[k]);
      _normalForceAverage[k] = 0.0f;
    }
    else
    {
      _normalForceWindow[k].pop_front();
      _normalForceWindow[k].push_back(_normalForce[k]);
      _normalForceAverage[k] = 0.0f;
      for(int m = 0; m < WINDOW_SIZE; m++)
      {
        _normalForceAverage[k]+=_normalForceWindow[k][m];
      }
      _normalForceAverage[k] /= WINDOW_SIZE;
    }
  }

  if(_normalForceAverage[LEFT] > _graspingForceThreshold && _normalForceAverage[RIGHT] > _graspingForceThreshold)
  {
    _objectGrasped = true;
  }
  else
  {
    _objectGrasped = false;
  }
  std::cerr << "Object grapsed: " <<(int) _objectGrasped << std::endl;
}


void FeetTelemanipulation::pureTelemanipulation()
{
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    _xd[k] = _xdFoot[k];
    _vd[k] = _x0[k]+_xd[k]-_x[k];

    if(_vd[k].norm()>0.3f)
    {
      _vd[k] *= 0.3f/_vd[k].norm();
    }  
    // std::cerr << k << "a: " << (_x0[k]+_xd[k]).transpose() << std::endl;
    // std::cerr << k << "v: " << _vd[k].transpose() << std::endl;
    _FdFoot[k] =  _wRb[k]*_filteredWrench[k].segment(0,3);
  }
}


void FeetTelemanipulation::autonomousLoadSupport()
{
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    if(_objectGrasped)
    {
      _Fd[k] = _targetForce;
    }
    else
    {
      _Fd[k] = 0.0f;
    }

    _xd[k] = _xdFoot[k];
    _vd[k] = _x0[k]+_xd[k]-_x[k]+(_Fd[k]/_d1[k])*_wRb[k].col(2);

    if(_vd[k].norm()>0.3f)
    {
      _vd[k] *= 0.3f/_vd[k].norm();
    }  
    _FdFoot[k] = _wRb[k]*_filteredWrench[k].segment(0,3)+_Fd[k]*_wRb[k].col(2);
    // _FdFoot[k] = _wRb[k]*_filteredWrench[k].segment(0,3);
    // _FdFoot[k].setConstant(0.0f);
    // _FdFoot[k] = _wRb[k]*_filteredWrench[k].segment(0,3)+_Fd[k]*_wRb[k].col(2);
    std::cerr << "Fd foot: " << k << " :" << _FdFoot[k].transpose() << std::endl;
  }
}


void FeetTelemanipulation::computeDesiredFootWrench()
{
  // temp.setConstant(0.0f);
  _desiredFootWrench[RIGHT](1) = _FdFoot[RIGHT](0);
  _desiredFootWrench[RIGHT](0) = -_FdFoot[RIGHT](1);
  _desiredFootWrench[RIGHT](3) = _FdFoot[RIGHT](2)*0.205/5;
  // _desiredFootWrench[RIGHT](0) += -_kxy*_footPose[RIGHT](0)-_dxy*_footTwist[RIGHT](0);
  // _desiredFootWrench[RIGHT](1) += -_kxy*_footPose[RIGHT](1)-_dxy*_footTwist[RIGHT](1);
  // _desiredFootWrench[RIGHT](3) += -_kphi*_footPose[RIGHT](3)-_dphi*_footTwist[RIGHT](3);

  _desiredFootWrench[LEFT](1) = _FdFoot[LEFT](0);
  _desiredFootWrench[LEFT](0) = -_FdFoot[LEFT](1);
  _desiredFootWrench[LEFT](3) = _FdFoot[LEFT](2)*0.205/5;

    for(int k = 0; k < NB_ROBOTS; k++)
  { 
    _desiredFootWrench[k](0) += -_kxy*_footPose[k](0)-_dxy*_footTwist[k](0);
    _desiredFootWrench[k](1) += -_kxy*_footPose[k](1)-_dxy*_footTwist[k](1);
    _desiredFootWrench[k](3) += -_kphi*_footPose[k](3)-_dphi*_footTwist[k](3);
  }

  for(int k = 0; k < 3; k++)
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


void FeetTelemanipulation::computeDesiredOrientation()
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

    _qd[k] = Utils<float>::rotationMatrixToQuaternion(Rd);

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

    // _qd[k] = Utils<float>::rotationMatrixToQuaternion(Rd);

    // if(_q[k].dot(_qd[k])<0)
    // {
    //   _qd[k] *=-1.0f;
    // }


     // _qd[k] << 0.0f,0.0f,1.0f,0.0f;

    // Compute needed angular velocity to perform the desired quaternion
    Eigen::Vector4f qcurI, wq;
    qcurI(0) = _q[k](0);
    qcurI.segment(1,3) = -_q[k].segment(1,3);
    wq = 5.0f*Utils<float>::quaternionProduct(qcurI,_qd[k]-_q[k]);
    Eigen::Vector3f omegaTemp = _wRb[k]*wq.segment(1,3);
    _omegad[k] = omegaTemp; 
  }
}


void FeetTelemanipulation::logData()
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
              << _footState[RIGHT] << std::endl;


}


void FeetTelemanipulation::publishData()
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

    _msgFootInput.FxDes = _desiredFootWrench[k](0);
    _msgFootInput.FyDes = _desiredFootWrench[k](1);
    _msgFootInput.TphiDes = _desiredFootWrench[k](3);
    _msgFootInput.TthetaDes = _desiredFootWrench[k](4);
    _msgFootInput.TpsiDes = _desiredFootWrench[k](5);
    _msgFootInput.stateDes = 2;
    _pubFootInput[k].publish(_msgFootInput);
  }
}


void FeetTelemanipulation::updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg, int k)
{

  Eigen::Vector3f temp = _x[k];

  // Update end effecotr pose (position+orientation)
  _x[k] << msg->position.x, msg->position.y, msg->position.z;
  _q[k] << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
  _wRb[k] = Utils<float>::quaternionToRotationMatrix(_q[k]);
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


void FeetTelemanipulation::updateRobotTwist(const geometry_msgs::Twist::ConstPtr& msg, int k)
{
  _v[k] << msg->linear.x, msg->linear.y, msg->linear.z;
  _w[k] << msg->angular.x, msg->angular.y, msg->angular.z;

  if(!_firstRobotTwist[k])
  {
    _firstRobotTwist[k] = true;
  }
}
 

void FeetTelemanipulation::updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg, int k)
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
      std::cerr << "[FeetTelemanipulation]: Bias " << k << ": " <<_wrenchBias[k].transpose() << std::endl;
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


void FeetTelemanipulation::updateDampingMatrix(const std_msgs::Float32MultiArray::ConstPtr& msg, int k) 
{
  if(!_firstDampingMatrix[k])
  {
    _firstDampingMatrix[k] = true;
  }

  _D[k] << msg->data[0],msg->data[1],msg->data[2],
           msg->data[3],msg->data[4],msg->data[5],
           msg->data[6],msg->data[7],msg->data[8];
}


void FeetTelemanipulation::updateFootOutput(const custom_msgs::FootOutputMsg::ConstPtr& msg, int k)
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


void FeetTelemanipulation::dynamicReconfigureCallback(robot_shared_control::feetTelemanipulation_paramsConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure request. Updatig the parameters ...");

  _filteredForceGain = config.filteredForceGain;
  _velocityLimit = config.velocityLimit;
  _kxy = config.kxy;
  _dxy = config.dxy;
  _kphi = config.kphi;
  _dphi = config.dphi;
  _xyPositionMapping = config.xyPositionMapping;
  _zPositionMapping = config.zPositionMapping;
  _useSharedControl = config.useSharedControl;
  if(_useSharedControl)
  {
    _strategy = AUTONOMOUS_LOAD_SUPPORT;
  }
  else
  {
    _strategy = PURE_TELEMANIPULATION;
  }
}