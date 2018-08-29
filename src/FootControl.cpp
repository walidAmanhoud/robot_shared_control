#include "FootControl.h"
#include "Utils.h"

FootControl* FootControl::me = NULL;

FootControl::FootControl(ros::NodeHandle &n, double frequency):
  _n(n),
  _loopRate(frequency),
  _dt(1.0f/frequency),
  _xCFilter(3,3,6,1.0f/frequency),
  _xDFilter(3,3,6,1.0f/frequency),
  _zDirFilter(3,3,6,1.0f/frequency)
{
  me = this;

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

    _firstRobotPose[k] = false;
    _firstRobotTwist[k] = false;
    _firstWrenchReceived[k] = false;
    _firstDampingMatrix[k] = true;

    _wrenchCount[k] = 0;
    _wrenchBiasOK[k] = false;
    _d1[k] = 1.0f;

    _footData[k].setConstant(0.0f);
    _footPosition[k].setConstant(0.0f);
    _vm[k].setConstant(0.0f);
    _footInterfaceSequenceID[k] = 0;    
    _firstFootInterfaceData[k] = false;

  }
  for(int k = 0; k < TOTAL_NB_MARKERS; k++)
  {
    _firstOptitrackPose[k] = false;
  }
  _optitrackOK = false;
  _stop = false;
  _firstObjectPose = false;
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
  
  _subFootInterfaceData[RIGHT] = _n.subscribe<geometry_msgs::PoseStamped>("/FI_Pose",1, boost::bind(&FootControl::updateFootInterfaceData,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subFootInterfaceData[LEFT] = _n.subscribe<geometry_msgs::PoseStamped>("/FI_Pose/2",1, boost::bind(&FootControl::updateFootInterfaceData,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

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


  // Dynamic reconfigure definition
  _dynRecCallback = boost::bind(&FootControl::dynamicReconfigureCallback, this, _1, _2);
  _dynRecServer.setCallback(_dynRecCallback);

  signal(SIGINT,FootControl::stopNode);

  if(!_n.getParamCached("/lwr/ds_param/damping_eigval0",_d1[RIGHT]))
  {
    ROS_ERROR("[FootControl]: Cannot read first eigen value of passive ds controller for right robot");
    return false;
  }

  if(!_n.getParamCached("/lwr2/ds_param/damping_eigval0",_d1[LEFT]))
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
    // std::cerr << (int) _firstRobotPose << " " << (int) _firstRobotTwist << " " << (int) _firstFootInterfaceData << std::endl;
    // std::cerr << _firstOptitrackPose[ROBOT_BASIS_LEFT] << _firstOptitrackPose[ROBOT_BASIS_RIGHT] << _firstOptitrackPose[P1] <<
       // _firstOptitrackPose[P2] << _firstOptitrackPose[P3] << _firstOptitrackPose[P4] << _firstDampingMatrix << _firstFootInterfaceData<<std::endl;
    if(_firstRobotPose[RIGHT] && _firstRobotPose[LEFT] && _firstRobotTwist[RIGHT] && _firstRobotTwist[LEFT] &&
       _wrenchBiasOK[RIGHT] && _wrenchBiasOK[LEFT] && _firstDampingMatrix[RIGHT] && _firstDampingMatrix[LEFT] &&
       _firstOptitrackPose[ROBOT_BASIS_LEFT] && _firstOptitrackPose[ROBOT_BASIS_RIGHT] && _firstOptitrackPose[P1] &&
       _firstOptitrackPose[P2] && _firstOptitrackPose[P3] && _firstOptitrackPose[P4] &&
       _firstFootInterfaceData[LEFT] && _firstFootInterfaceData[RIGHT])
    {
      _mutex.lock();

      // Check for update of the DS-impedance controller gain
      ros::param::getCached("/lwr/ds_param/damping_eigval0",_d1[RIGHT]);
      ros::param::getCached("/lwr2/ds_param/damping_eigval0",_d1[LEFT]);

      // Initialize optitrack
      if(!_optitrackOK)
      {
        optitrackInitialization();
      }
      else
      {

        // Compute object pose
        computeObjectPose();

        if(_firstObjectPose)
        {
          // Compute control command
          computeCommand();

          // Publish data to topics
          publishData();
        }

        // Log data
        // logData();
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
  }

  publishData();
  ros::spinOnce();
  _loopRate.sleep();

  ros::shutdown();
}


void FootControl::stopNode(int sig)
{
  me->_stop = true;
}


void FootControl::computeObjectPose()
{
  // Check if all markers on the object are tracked
  // The four markers are positioned on the corner of the upper face:
  // P2 ----- P3
  // |        |
  // |        |
  // P1 ----- P4
  if(_markersTracked.segment(NB_ROBOTS,TOTAL_NB_MARKERS-NB_ROBOTS).sum() == TOTAL_NB_MARKERS-NB_ROBOTS)
  {

    if(!_firstObjectPose)
    {
      _firstObjectPose = true;
    }

    // Compute markers position in the right robot frame
    _p1 = _markersPosition.col(P1)-_markersPosition0.col(ROBOT_BASIS_RIGHT);
    _p2 = _markersPosition.col(P2)-_markersPosition0.col(ROBOT_BASIS_RIGHT);
    _p3 = _markersPosition.col(P3)-_markersPosition0.col(ROBOT_BASIS_RIGHT);
    _p4 = _markersPosition.col(P4)-_markersPosition0.col(ROBOT_BASIS_RIGHT);

    // Compute object center position
    _xoC = (_p1+_p2+_p3+_p4)/4.0f;
    // Compute object dimension vector
    // The dimension obtained from the markers is adjusted to match the real
    // object dimension
    _xoD = (_p3+_p4-_p1-_p2)/2.0f;
    _xoD = 0.20f*_xoD.normalized(); 

    // Filter center position and dimension vector of the object
    SGF::Vec temp(3);
    _xCFilter.AddData(_xoC);
    _xCFilter.GetOutput(0,temp);
    _xoC = temp;
    Eigen::Vector3f xDir = _p2-_p1;
    xDir.normalize();
    Eigen::Vector3f yDir = _p1-_p4;
    yDir.normalize();
    Eigen::Vector3f zDir = xDir.cross(yDir);
    zDir.normalize();
    _zDirFilter.AddData(xDir.cross(yDir));
    _zDirFilter.GetOutput(0,temp);
    zDir = temp;
    zDir.normalize();   
    _xoC -= 1.0f*(_objectDim(2)/2.0f)*zDir;
      
    // Filter object direction
    _xDFilter.AddData(_xoD);
    _xDFilter.GetOutput(0,temp);
    _xoD = 0.20f*temp.normalized();

    // std::cerr <<"real" << _xdD.norm() << " " <<_xdD.transpose() << std::endl;
    // std::cerr << "filter" <<  _xoD.norm() << " " <<_xoD.transpose() << std::endl;

    _msgMarker.pose.position.x = _xoC(0);
    _msgMarker.pose.position.y = _xoC(1);
    _msgMarker.pose.position.z = _xoC(2);
    Eigen::Matrix3f R;
    R.col(0) = xDir;
    R.col(1) = yDir;
    R.col(2) = zDir;
    Eigen::Vector4f q = Utils::rotationMatrixToQuaternion(R);
    _msgMarker.pose.orientation.x = q(1);
    _msgMarker.pose.orientation.y = q(2);
    _msgMarker.pose.orientation.z = q(3);
    _msgMarker.pose.orientation.w = q(0);

  }

}


void FootControl::computeCommand()
{

  // Compute robots center + distance vector;
  _xC = (_x[LEFT]+_x[RIGHT])/2.0f;
  _xD = (_x[RIGHT]-_x[LEFT]);

  // Compute errors to object center position and dimension vector 
  _eoD = (_xD-_xoD).dot(_xoD.normalized());
  _eoC = (_xoC-_xC).norm();

  float alpha = Utils::smoothFall(_eoD,0.02f,0.1f)*Utils::smoothFall(_eoC,0.1f,0.2f); 
  if(_normalForce[LEFT]*alpha>_graspingForceThreshold && _normalForce[RIGHT]*alpha>_graspingForceThreshold)
  {
    _objectGrasped = true;
  }
  else
  {
    _objectGrasped = false;
  }

  if(_objectGrasped) // Object reachable and grasped
  {
    _xdD << 0.0f,-1.0f,0.0f;
    _xdD *= 0.20f;
  }
  else // Object reachable but not grasped
  {
  }

    _xdD = _xoD;
  std::cerr << "Object grasped: " << (int) _objectGrasped << " alpha: " << alpha <<std::endl;

  _e1[LEFT] = _xdD.normalized();
  _e1[RIGHT] = -_xdD.normalized();

  footDataTransformation();

  positionPositionMapping();

  positionVelocityMapping();

  for(int k = 0; k < NB_ROBOTS; k++)
  {
    _normalForce[k] = fabs((_wRb[k]*_filteredWrench[k].segment(0,3)).dot(_e1[k]));

    if(_d1[k]<1.0f)
    {
      _d1[k] = 1.0f;
    }

    if(_objectGrasped)
    {
      _Fd[k] = _targetForce;
    }
    else
    {
      _Fd[k] = _targetForce*alpha;    
    }

    if(_e1[LEFT].dot(_vm[LEFT])<0 && _e1[RIGHT].dot(_vm[RIGHT])<0)
    {
      _Fd[k] = 0.0f;
    }

    // _fx[k] = _vm[k];
    // _vd[k] = _vm[k];
    // _vd[k] = _vm[k]+(_Fd[k]/_d1[k])*_e1[k];
    // _vd[k].setConstant(0.0f);
    _vd[k] = _x0[k]+_xh[k]-_x[k];

    if(_vd[k].norm()>0.3f)
    {
      _vd[k] *= 0.3f/_vd[k].norm();
    }    
    // std::cerr << "vd " << k << " : " <<_vd[k].transpose() << " " << _vd[k].norm() << std::endl;
  }


  // // Update surface info
  // updateSurfaceInformation();

  // // Compute nominal DS
  // computeNominalDS();

  // // Compute modulated DS
  // computeModulatedDS();
    
  computeDesiredOrientation();
  // _qd[0] << 0.0f,0.0f,1.0f,0.0f;
  // _qd[1] << 0.0f,0.0f,1.0f,0.0f;
  // // Compute desired orientation
}


void FootControl::footDataTransformation()
{
  _footPosition[RIGHT](0) = FOOT_INTERFACE_Y_RANGE_RIGHT/2.0f-_footData[RIGHT](1);
  _footPosition[RIGHT](1) = -(FOOT_INTERFACE_X_RANGE_RIGHT/2.0f-_footData[RIGHT](0));
  _footPosition[RIGHT](2) = -_footData[RIGHT](3);
  _footPosition[LEFT](0) = FOOT_INTERFACE_Y_RANGE_LEFT/2.0f+_footData[LEFT](1);
  _footPosition[LEFT](1) = (FOOT_INTERFACE_X_RANGE_LEFT/2.0f+_footData[LEFT](0));
  _footPosition[LEFT](2) = _footData[LEFT](3);

  // std::cerr << "Before transformation: " <<_footData.segment(0,4).transpose() << std::endl;
  // std::cerr << "After transformation: " << _footPosition.transpose() << std::endl;
}

void FootControl::positionPositionMapping()
{

  Eigen::Vector3f gains[NB_ROBOTS];
  gains[RIGHT] << 2*0.7/FOOT_INTERFACE_Y_RANGE_RIGHT, 2*0.7/FOOT_INTERFACE_X_RANGE_RIGHT, 2*0.5/FOOT_INTERFACE_PHI_RANGE_RIGHT;
  gains[LEFT] << 2*0.7f/FOOT_INTERFACE_Y_RANGE_LEFT, 2*0.7/FOOT_INTERFACE_X_RANGE_LEFT, 2*0.5/FOOT_INTERFACE_PHI_RANGE_LEFT;
  
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    _xh[k] = gains[k].cwiseProduct(_footPosition[k]);
    std::cerr << "Master position " << k << " : " <<_xh[k].transpose() << std::endl;
  }
}

void FootControl::positionVelocityMapping()
{

  _velocityLimit = 0.25f;
  Eigen::Vector3f gains[NB_ROBOTS];
  gains[RIGHT] << 2.0f*_velocityLimit/FOOT_INTERFACE_Y_RANGE_RIGHT, 2.0f*_velocityLimit/FOOT_INTERFACE_X_RANGE_RIGHT, 2.0f*_velocityLimit/FOOT_INTERFACE_PHI_RANGE_RIGHT;
  gains[LEFT] << 2.0f*_velocityLimit/FOOT_INTERFACE_Y_RANGE_LEFT, 2.0f*_velocityLimit/FOOT_INTERFACE_X_RANGE_LEFT, 2.0f*_velocityLimit/FOOT_INTERFACE_PHI_RANGE_LEFT;
  
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    _vm[k] = gains[k].cwiseProduct(_footPosition[k]);
   if(_vm[k].norm()>_velocityLimit)
    {
      _vm[k] *= _velocityLimit/_vm[k].norm();
    }    
    std::cerr << "Master velocity " << k << " : " <<_vm[k].transpose() << std::endl;
  }
}
// void FootControl::updateSurfaceInformation()
// {
//   switch(_surfaceType)
//   {
//     case PLANAR:
//     {
//       // Compute markers position in the robot frame
//       // The three markers are positioned on the surface to form an angle of 90 deg:
//       // P1 ----- P2
//       // |       
//       // |
//       // P3
//       _p1 = _markersPosition.col(P1)-_markersPosition0.col(ROBOT_BASIS);
//       _p2 = _markersPosition.col(P2)-_markersPosition0.col(ROBOT_BASIS);
//       _p3 = _markersPosition.col(P3)-_markersPosition0.col(ROBOT_BASIS);
//       Eigen::Vector3f p13,p12;

//       // Compute main directions between the markers  
//       p13 = _p3-_p1;
//       p12 = _p2-_p1;
//       p13 /= p13.norm();
//       p12 /= p12.norm();

//       // Compute normal vector 
//       _planeNormal = p13.cross(p12);
//       _planeNormal /= _planeNormal.norm();
  
//       // Compute vertical projection onto the surface
//       _xProj = _x;
//       _xProj(2) = (-_planeNormal(0)*(_xProj(0)-_p3(0))-_planeNormal(1)*(_xProj(1)-_p3(1))+_planeNormal(2)*_p3(2))/_planeNormal(2);
      
//       // Compute _e1 = normal vector pointing towards the surface
//       _e1 = -_planeNormal;
      
//       // Compute signed normal distance to the plane
//       _normalDistance = (_xProj-_x).dot(_e1);

//       break;
//     }
//     case NON_FLAT:
//     {
//       _svm.preComputeKernel(true);

//       // The surface is learned with respect to a frame defined by the marker P1
//       // We get the robot position in the surface frame
//       Eigen::Vector3f x;
//       _p1 = _markersPosition.col(P1)-_markersPosition0.col(ROBOT_BASIS);
//       _p2 = _markersPosition.col(P2)-_markersPosition0.col(ROBOT_BASIS);
//       _p3 = _markersPosition.col(P3)-_markersPosition0.col(ROBOT_BASIS);

//       // Compute surface frame, wRs is the rotation matrix for the surface frame to the world frame  
//       _wRs.col(0) = (_p1-_p3).normalized();
//       _wRs.col(1) = (_p1-_p2).normalized();
//       _wRs.col(2) = ((_wRs.col(0)).cross(_wRs.col(1))).normalized();

//       // Compute robot postion in surface frame
//       x = _wRs.transpose()*(_x-_p1);

//       // We compute the normal distance by evlauating the SVM model
//       _normalDistance = _svm.calculateGamma(x.cast<double>());

//       // We get the normal vector by evaluating the gradient of the model
//       _planeNormal = _svm.calculateGammaDerivative(x.cast<double>()).cast<float>();
//       _planeNormal = _wRs*_planeNormal;
//       _planeNormal.normalize();
//       _e1 = -_planeNormal;
//       std::cerr << "[FootControl]: Normal distance: " << _normalDistance << " Normal vector: " << _e1.transpose() << std::endl;    

//       break;
//     }
//     default:
//     {
//       break;
//     }
//   }

//   if(_normalDistance < 0.0f)
//   {
//     _normalDistance = 0.0f;
//   }

//   // Compute normal force
//   Eigen::Vector3f F = _filteredWrench.segment(0,3);
//   _normalForce = _e1.dot(-_wRb*F);

// }


// void FootControl::computeNominalDS()
// {
//   // Compute fixed attractor on plane
//   if(_surfaceType == PLANAR)
//   {
//     _xAttractor = _p1+0.5f*(_p2-_p1)+0.3f*(_p3-_p1);
//     _xAttractor(2) = (-_planeNormal(0)*(_xAttractor(0)-_p1(0))-_planeNormal(1)*(_xAttractor(1)-_p1(1))+_planeNormal(2)*_p1(2))/_planeNormal(2);
//   }
//   else 
//   {
//     _xAttractor = _p1+0.45f*(_p2-_p1)+0.5f*(_p3-_p1);
//     // _xAttractor += _offset;

//     // Compute normal distance and vector at the attractor location in the surface frame
//     Eigen::Vector3f x, attractorNormal; 
//     x = _wRs.transpose()*(_xAttractor-_p1);

//     float attractorNormalDistance = _svm.calculateGamma(x.cast<double>());
//     attractorNormal = _svm.calculateGammaDerivative(x.cast<double>()).cast<float>();
//     attractorNormal = _wRs*attractorNormal;
//     attractorNormal.normalize();

//     // Compute attractor normal projection on the surface int the world frame
//     _xAttractor -= attractorNormalDistance*attractorNormal;
//   }

//   // The reaching velocity direction is aligned with the normal vector to the surface
//   Eigen::Vector3f v0 = _targetVelocity*_e1;
 
//   // Compute normalized circular dynamics projected onto the surface
//   Eigen::Vector3f vdContact;
//   vdContact = (Eigen::Matrix3f::Identity()-_e1*_e1.transpose())*getCircularMotionVelocity(_x,_xAttractor);
//   vdContact.normalize();

//   // Compute rotation angle + axis between reaching velocity vector and circular dynamics
//   float angle = std::acos(v0.normalized().dot(vdContact));
//   float theta = (1.0f-std::tanh(10*_normalDistance))*angle;
//   Eigen::Vector3f u = (v0.normalized()).cross(vdContact);

//   // Get corresponding rotation matrix
//   Eigen::Matrix3f K,R;
//   if(u.norm() < FLT_EPSILON)
//   {
//     R.setIdentity();
//   }
//   else
//   {
//     u/=u.norm();
//     K = Utils::getSkewSymmetricMatrix(u);
//     R = Eigen::Matrix3f::Identity()+std::sin(theta)*K+(1.0f-std::cos(theta))*K*K;
//   }

//   // Compute nominal DS
//   _fx = R*v0;
      
//   // Bound nominal DS for safety
//   if(_fx.norm()>_velocityLimit)
//   {
//     _fx *= _velocityLimit/_fx.norm();
//   }
// }


// Eigen::Vector3f FootControl::getCircularMotionVelocity(Eigen::Vector3f position, Eigen::Vector3f attractor)
// {
//   Eigen::Vector3f velocity;

//   position = position-attractor;

//   velocity(2) = -position(2);

//   float R = sqrt(position(0) * position(0) + position(1) * position(1));
//   float T = atan2(position(1), position(0));

//   float r = 0.05f;
//   float omega = M_PI;

//   velocity(0) = -(R-r) * cos(T) - R * omega * sin(T);
//   velocity(1) = -(R-r) * sin(T) + R * omega * cos(T);

//   return velocity;
// }


// void FootControl::updateTankScalars()
// {
//   if(_s>_smax)
//   {
//     _alpha = 0.0f;
//   }
//   else
//   {
//     _alpha = 1.0f;
//   }
//   _alpha = Utils::smoothFall(_s,_smax-0.1f*_smax,_smax);

//   _pn = _d1*_v.dot(_fx);

//   if(_s < 0.0f && _pn < 0.0f)
//   {
//     _beta = 0.0f;
//   }
//   else if(_s > _smax && _pn > FLT_EPSILON)
//   {
//     _beta = 0.0f;
//   }
//   else
//   {
//     _beta = 1.0f;
//   }
  
//   _pf = _Fd*_v.dot(_e1);
  
//   if(_s < FLT_EPSILON && _pf > FLT_EPSILON)
//   {
//     _gamma = 0.0f;
//   }
//   else if(_s > _smax && _pf < FLT_EPSILON)
//   {
//     _gamma = 0.0f;
//   }
//   else
//   {
//     _gamma = 1.0f;
//   }

//   if(_pf<FLT_EPSILON)
//   {
//     _gammap = 1.0f;
//   }
//   else
//   {
//     _gammap = _gamma;
//   }
// }


void FootControl::computeModulatedDS()
{
  for(int k = 0; k < NB_ROBOTS; k++)
  {

    // Compute modulation gain
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
    // Compute rotation error between current orientation and plane orientation using Rodrigues' law
    Eigen::Vector3f ref;
    if(k == (int) RIGHT)
    {
      ref = -_xdD.normalized();
    }
    else
    {
      ref = _xdD.normalized();
    }
      
    ref.normalize();

    // Compute rotation error between current orientation and plane orientation using Rodrigues' law
    Eigen::Vector3f u;
    u = (_wRb[k].col(2)).cross(ref);
    float c = (_wRb[k].col(2)).transpose()*ref;  
    float s = u.norm();
    u /= s;

    Eigen::Matrix3f K;
    K << Utils::getSkewSymmetricMatrix(u);

    Eigen::Matrix3f Re;
    if(fabs(s)< FLT_EPSILON)
    {
      Re = Eigen::Matrix3f::Identity();
    }
    else
    {
      Re = Eigen::Matrix3f::Identity()+s*K+(1-c)*K*K;
    }
    
    // Convert rotation error into axis angle representation
    Eigen::Vector3f omega;
    float angle;
    Eigen::Vector4f qtemp = Utils::rotationMatrixToQuaternion(Re);
    Utils::quaternionToAxisAngle(qtemp,omega,angle);

    // Compute final quaternion on plane
    Eigen::Vector4f qf = Utils::quaternionProduct(qtemp,_q[k]);

    // Perform quaternion slerp interpolation to progressively orient the end effector while approaching the surface
    if(k == (int)RIGHT)
    {
      _normalDistance[k] = (_x[k]-(_xoC+_xoD/2.0f)).dot(_xoD.normalized());
    }
    else
    {
      _normalDistance[k] = (_x[k]-(_xoC-_xoD/2.0f)).dot(-_xoD.normalized());
    }

    if(_normalDistance[k]<0.0f)
    {
      _normalDistance[k] = 0.0f;
    }
    std::cerr << "Quaternion " << k  << " " << _normalDistance[k] << std::endl;
    // std::cerr << 1.0f-std::tanh(3.0f*_normalDistance) << std::endl;
    Eigen::Vector4f q0; 
    q0 << 0.0f,0.0f,1.0f,0.0f;

    // _qd = Utils::slerpQuaternion(q0,qf,1.0f-std::tanh(3.0f*_normalDistance));
    // _qd[k] = Utils::slerpQuaternion(q0,qf,Utils::smoothFall(_normalDistance[k],0.1f,0.6f));
    _qd[k] = Utils::slerpQuaternion(_q[k],qf,Utils::smoothFall(_normalDistance[k],0.1f,0.6f));

    // Compute needed angular velocity to perform the desired quaternion
    Eigen::Vector4f qcurI, wq;
    qcurI(0) = _q[k](0);
    qcurI.segment(1,3) = -_q[k].segment(1,3);
    wq = 5.0f*Utils::quaternionProduct(qcurI,_qd[k]-_q[k]);
    Eigen::Vector3f omegaTemp = _wRb[k]*wq.segment(1,3);
    _omegad[k] = omegaTemp; 
  }
}


// void FootControl::logData()
// {
//   _outputFile << ros::Time::now() << " "
//               << _x.transpose() << " "
//               << _v.transpose() << " "
//               << _fx.transpose() << " "
//               << _vd.transpose() << " "
//               << _e1.transpose() << " "
//               << _wRb.col(2).transpose() << " "
//               << (_markersPosition.col(P1)-_markersPosition.col(ROBOT_BASIS)).transpose() << " "
//               << _normalDistance << " "
//               << _normalForce << " "
//               << _Fd << " "
//               << _sequenceID << " "
//               << _s << " " 
//               << _alpha << " "
//               << _beta << " "
//               << _gamma << " "
//               << _gammap << " "
//               << _dW << " " << std::endl;
// }


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
    _x0[k] = _x[k];
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


void FootControl::updateFootInterfaceData(const geometry_msgs::PoseStamped::ConstPtr& msg, int k)
{

  Eigen::Matrix<float,6,1> temp;
  temp = _footData[k];
  _footData[k] << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;

  if(_footInterfaceSequenceID[k]!=msg->header.seq)
  {
    _footInterfaceSequenceID[k] = msg->header.seq;
  }

  if(!_firstFootInterfaceData[k])
  {
    _firstFootInterfaceData[k] = true;
  }
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
  _offset(0) = config.xOffset;
  _offset(1) = config.yOffset;
  _offset(2) = config.zOffset;
}

