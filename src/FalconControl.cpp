#include "FalconControl.h"
#include "Utils.h"

FalconControl* FalconControl::me = NULL;

FalconControl::FalconControl(ros::NodeHandle &n, double frequency):
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

  _x.setConstant(0.0f);
  _q.setConstant(0.0f);
  _wrenchBiasOK = false;
  _wrenchBias.setConstant(0.0f);
  _wrench.setConstant(0.0f);
  _filteredWrench.setConstant(0.0f);
  _normalDistance = 0.0f;
  _normalForce = 0.0f;


  _xd.setConstant(0.0f);
  _fx.setConstant(0.0f);
  _vd.setConstant(0.0f);
  _Fd = 0.0f;
  _omegad.setConstant(0.0f);
  _qd.setConstant(0.0f);

  _p << 0.0f,0.0f,-0.007f;
  // _taskAttractor << -0.65f, 0.05f, -0.007f;
  _taskAttractor << -0.6f, 0.1f, 0.4f;
  _xAttractor[0] << -0.8, 0.4f,0.4f;
  _xAttractor[0] << -0.7, 0.0f,0.14f;
  _xAttractor[1] << -0.7, -0.5f,0.4f;
  _xAttractor[2] << -0.7, 0.0f,0.7f;
  _planeNormal << 0.0f, 0.0f, 1.0f;

  _xFalcon.setConstant(0.0f);
  _vFalcon.setConstant(0.0f);
  _xM.setConstant(0.0f);
  _vM.setConstant(0.0f);
  _vh.setConstant(0.0f);
  _kFalcon = 0.0f;
  _dFalcon = 0.0f;
  _buttonsFalcon = 0;

  _firstRobotPose = false;
  _firstRobotTwist = false;
  _firstWrenchReceived = false;
  for(int k = 0; k < TOTAL_NB_MARKERS; k++)
  {
    _firstOptitrackPose[k] = true;
  }
  _firstDampingMatrix = true;
  _optitrackOK = true;
  _wrenchBiasOK = false;
  _stop = false;
  _firstFalconPosition = false;
  _firstFalconVelocity = false;
  _firstFalconButtons = false;
  _firstObjectPose = true;

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
  _wrenchCount = 0;
  _d1 = 0.0f;
  _D.setConstant(0.0f);

  _velocityLimit = 0.0f;

  _beliefs.setConstant(0.0f);
  _dbeliefs.setConstant(0.0f);
  for(int k = 0; k < NB_TASKS; k++)
  {
    _fxk[k].setConstant(0.0f);
  }
  _fx.setConstant(0.0f);
  _beliefs(2) = 1.0f;
  _adaptationRate = 100.0f;

  _e1 << 0.0f, 0.0f, -1.0f;

  _targetForce = 10.0f;
  _sigmaH =0.0f;


}


bool FalconControl::init() 
{
  // Subscriber definitions
  _subRobotPose = _n.subscribe("/lwr/ee_pose", 1, &FalconControl::updateRobotPose, this, ros::TransportHints().reliable().tcpNoDelay());
  _subRobotTwist = _n.subscribe("/lwr/joint_controllers/twist", 1, &FalconControl::updateRobotTwist, this, ros::TransportHints().reliable().tcpNoDelay());
  _subForceTorqueSensor = _n.subscribe("/ft_sensor/netft_data", 1, &FalconControl::updateRobotWrench, this, ros::TransportHints().reliable().tcpNoDelay());
   _subOptitrackPose[ROBOT_BASIS_LEFT] = _n.subscribe<geometry_msgs::PoseStamped>("/optitrack/robot_left/pose", 1, boost::bind(&FalconControl::updateOptitrackPose,this,_1,ROBOT_BASIS_LEFT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[ROBOT_BASIS_RIGHT] = _n.subscribe<geometry_msgs::PoseStamped>("/optitrack/robot_right/pose", 1, boost::bind(&FalconControl::updateOptitrackPose,this,_1,ROBOT_BASIS_RIGHT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[P1] = _n.subscribe<geometry_msgs::PoseStamped>("/optitrack/p1/pose", 1, boost::bind(&FalconControl::updateOptitrackPose,this,_1,P1),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[P2] = _n.subscribe<geometry_msgs::PoseStamped>("/optitrack/p2/pose", 1, boost::bind(&FalconControl::updateOptitrackPose,this,_1,P2),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[P3] = _n.subscribe<geometry_msgs::PoseStamped>("/optitrack/p3/pose", 1, boost::bind(&FalconControl::updateOptitrackPose,this,_1,P3),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[P4] = _n.subscribe<geometry_msgs::PoseStamped>("/optitrack/p4/pose", 1, boost::bind(&FalconControl::updateOptitrackPose,this,_1,P4),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subDampingMatrix = _n.subscribe("/lwr/joint_controllers/passive_ds_damping_matrix", 1, &FalconControl::updateDampingMatrix,this,ros::TransportHints().reliable().tcpNoDelay());
  _subFalconPosition = _n.subscribe<geometry_msgs::PoseStamped>("/falcon/position",1,&FalconControl::updateFalconPosition, this, ros::TransportHints().reliable().tcpNoDelay());
  _subFalconVelocity = _n.subscribe<geometry_msgs::TwistStamped>("/falcon/velocity",1,&FalconControl::updateFalconVelocity, this, ros::TransportHints().reliable().tcpNoDelay());
  _subFalconButtons = _n.subscribe<std_msgs::UInt32>("/falcon/buttons",1,&FalconControl::updateFalconButtons, this, ros::TransportHints().reliable().tcpNoDelay());

  // Publisher definitions
  _pubDesiredTwist = _n.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 1);
  _pubDesiredOrientation = _n.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);
  _pubFilteredWrench = _n.advertise<geometry_msgs::WrenchStamped>("FalconControl/filteredWrench", 1);
  _pubTaskAttractor = _n.advertise<geometry_msgs::PointStamped>("FalconControl/taskAttractor", 1);  
  _pubNormalForce = _n.advertise<std_msgs::Float32>("FalconControl/normalForce", 1);
  _pubFalconForce = _n.advertise<geometry_msgs::WrenchStamped>("/falcon/force_desired", 1);

  // Dynamic reconfigure definition
  _dynRecCallback = boost::bind(&FalconControl::dynamicReconfigureCallback, this, _1, _2);
  _dynRecServer.setCallback(_dynRecCallback);

  signal(SIGINT,FalconControl::stopNode);

  if(!_n.getParamCached("/lwr/ds_param/damping_eigval0",_d1))
  {
    ROS_ERROR("[FalconControl]: Cannot read first eigen value of passive ds controller");
    return false;
  }

  if (_n.ok()) 
  { 
    // Wait for poses being published
    ros::spinOnce();
    ROS_INFO("[FalconControl]: The modulated ds node is ready.");
    return true;
  }
  else 
  {
    ROS_ERROR("[FalconControl]: The ros node has a problem.");
    return false;
  }
}


void FalconControl::run()
{
  _timeInit = ros::Time::now().toSec();

  while (!_stop) 
  {
    // std::cerr << (int) _firstRobotPose << " " << (int) _firstRobotTwist << " " << (int) _firstFootInterfaceData << std::endl;
    // std::cerr << _firstOptitrackPose[ROBOT_BASIS_LEFT] << _firstOptitrackPose[ROBOT_BASIS_RIGHT] << _firstOptitrackPose[P1] <<
       // _firstOptitrackPose[P2] << _firstOptitrackPose[P3] << _firstOptitrackPose[P4] << _firstDampingMatrix << _firstFootInterfaceData<<std::endl;
    if(_firstRobotPose && _firstRobotTwist && _wrenchBiasOK &&
       _firstOptitrackPose[ROBOT_BASIS_LEFT] && _firstOptitrackPose[ROBOT_BASIS_RIGHT] && _firstOptitrackPose[P1] &&
       _firstOptitrackPose[P2] && _firstOptitrackPose[P3] && _firstOptitrackPose[P4] && _firstDampingMatrix &&
       _firstFalconPosition && _firstFalconVelocity && _firstFalconButtons)
    {
      _mutex.lock();

      // Check for update of the DS-impedance controller gain
      ros::param::getCached("/lwr/ds_param/damping_eigval0",_d1);

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

          computeFalconForce();

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
  _vd.setConstant(0.0f);
  _omegad.setConstant(0.0f);
  _qd = _q;

  publishData();
  ros::spinOnce();
  _loopRate.sleep();

  ros::shutdown();
}


void FalconControl::stopNode(int sig)
{
  me->_stop = true;
}


void FalconControl::computeObjectPose()
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
    std::cerr << _xoC.transpose() << std::endl; 
      
    // Filter object direction
    _xDFilter.AddData(_xoD);
    _xDFilter.GetOutput(0,temp);
    _xoD = 0.20f*temp.normalized();

    // std::cerr <<"real" << _xdD.norm() << " " <<_xdD.transpose() << std::endl;
    // std::cerr << "filter" <<  _xoD.norm() << " " <<_xoD.transpose() << std::endl;

  }

}


void FalconControl::computeCommand()
{
  falconDataTransformation();

  rateControllerStep();

  updateIndividualTasks();

  if(_buttonsFalcon != (int) CENTER)
  {
    taskAdaptation();
  }

  _fx.setConstant(0.0f);
  for(int k = 0; k < NB_TASKS; k++)
  {
    _fx+=_beliefs[k]*_fxk[k];
  }

  Eigen::MatrixXf::Index indexMax;
  if(fabs(1.0f-_beliefs.array().maxCoeff(&indexMax))<FLT_EPSILON)
  {
    _Fd = _Fdk[indexMax];
     if(_buttonsFalcon == (int) CENTER)
     {
      _sigmaH += 5.0f*_dt*_vM.dot(_e1);
      if(_sigmaH>1)
      {
        _sigmaH = 1.0f;
      }
      else if(_sigmaH<0.0f)
      {
        _sigmaH = 0.0f;
      }
    }
    _Fd*=(1+_sigmaH);
  }
  else
  {
    _Fd = 0.0f;
    _sigmaH = 0.0f;
  }
  std::cerr << "dBeliefs: " << _dbeliefs.transpose() << std::endl;
  std::cerr << "vM: " << _vM.norm() << std::endl;
  std::cerr << "beliefs: " << _beliefs.transpose() << std::endl;
  std::cerr << "f: " << _fx.transpose() << " Fd: " << _Fd << " sigmaH: "<< _sigmaH << std::endl;

  // _vd = _vh;
  // _vd.setConstant(0.0f);
  // _vd = _fx;
  //   if(_vd.norm()>_velocityLimit)
  //   {
  //     _vd *= _velocityLimit/_vd.norm();
  //   }

  // std::cerr << "Falcon button: " << (int) _buttonsFalcon << std::endl;
  // std::cerr << "Robot desired velocity: " <<_vd.transpose() << " " << _vd.norm() << std::endl;

  // // Update surface info
  // updateSurfaceInformation();

  // // Compute nominal DS
  // computeNominalDS();

  // // Compute modulated DS
  computeModulatedDS();
    
  // // Compute desired orientation
  // computeDesiredOrientation();
  _qd << 0.0f,0.0f,1.0f,0.0f;
  Eigen::Vector4f qcurI, wq;
  qcurI(0) = _q(0);
  qcurI.segment(1,3) = -_q.segment(1,3);
  wq = 5.0f*Utils::quaternionProduct(qcurI,_qd-_q);
  Eigen::Vector3f omegaTemp = _wRb*wq.segment(1,3);
  _omegad = omegaTemp; 
}

void FalconControl::updateIndividualTasks()
{
  for(int k = 0; k < NB_TASKS; k++)
  {
    _fxk[k] = _xAttractor[k]-_x;
    _Fdk[k] = 0.0f;
  }

  Eigen::Vector3f F = _filteredWrench.segment(0,3);
  _normalForce = _e1.dot(-_wRb*F);
    // Compute desired force profile
  if(_normalForce<3.0f)
  {
    _Fdk[0] = 5.0f;
  }
  else
  {
    _Fdk[0] = _targetForce;
  }

}

void FalconControl::taskAdaptation()
{
  _fx.setConstant(0.0f);
  for(int k = 0; k < NB_TASKS; k++)
  {
    _fx+=_beliefs[k]*_fxk[k];
  }

  for(int k = 0; k < NB_TASKS; k++)
  {
    _dbeliefs[k] = _adaptationRate*((_vh-_fx).dot(_fxk[k])+(_beliefs[k]-0.5f)*_fxk[k].squaredNorm());
    // std::cerr << k << " " << (_vh-_fx).dot(_fxk[k]) << std::endl;
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

void FalconControl::falconDataTransformation()
{
  _xM(0) = -_xFalcon(0);
  _xM(1) = -_xFalcon(1);
  _xM(2) = _xFalcon(2);
  _vM(0) = -_vFalcon(0);
  _vM(1) = -_vFalcon(1);
  _vM(2) = _vFalcon(2);
  // std::cerr << "Before transformation: " <<_footData.segment(0,4).transpose() << std::endl;
  // std::cerr << "After transformation: " << _xM.transpose() << std::endl;
}


void FalconControl::rateControllerStep()
{
  Eigen::Vector3f gains;
  gains << 2.0f*_velocityLimit/FALCON_INTERFACE_X_RANGE, 2.0f*_velocityLimit/FALCON_INTERFACE_Y_RANGE, 2.0f*_velocityLimit/FALCON_INTERFACE_Z_RANGE;
  

  // if(_vFalcon.norm() > FLT_EPSILON)
  {
    _vh = gains.cwiseProduct(_xM);
  }
  // else
  // {
  //   _vh.setConstant(0.0f);
  // }
  // else
  // {
  //   _vh.setConstant(0.0f);
  // }

  for(int k = 0; k < 3; k++)
  {
    if(fabs(_vh(k))>_velocityLimit)
    {
      _vh(k) *= _velocityLimit/fabs(_vh(k));
    }
  }
  // std::cerr << "Master velocity: " << _vh.transpose() << std::endl;
}



void FalconControl::computeFalconForce()
{
  Eigen::Matrix3f K, D;
  K = _kFalcon*Eigen::Matrix3f::Identity();
  D = _dFalcon*Eigen::Matrix3f::Identity();

  // if(fabs(1.0f-_beliefs.array().maxCoeff())<FLT_EPSILON)
  {
    _FdFalcon = -K*_xFalcon-D*_vFalcon;
     if(_buttonsFalcon == (int) CENTER && _vh(2)<0.0)
     {
      _FdFalcon(2)+=_normalForce;
     }
  }
  // else
  // {
  //   _FdFalcon = -D*_vFalcon;
  // }

  // std::cerr << "Falcon desired force: " << _FdFalcon.transpose() << " " << std::endl;

}
// void FalconControl::updateSurfaceInformation()
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
//       std::cerr << "[FalconControl]: Normal distance: " << _normalDistance << " Normal vector: " << _e1.transpose() << std::endl;    

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


// void FalconControl::computeNominalDS()
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


// Eigen::Vector3f FalconControl::getCircularMotionVelocity(Eigen::Vector3f position, Eigen::Vector3f attractor)
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


// void FalconControl::updateTankScalars()
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


void FalconControl::computeModulatedDS()
{
  // Check the first impedance gain of the DS-impedance controller
  if(_d1<1.0f)
  {
    _d1 = 1.0f;
  }

  // Update tank scalar variables
  // updateTankScalars();
  _gammap = 1.0f;

  // Compute modulation gain
  float delta = std::pow(2.0f*_e1.dot(_fx)*_gammap*_Fd/_d1,2.0f)+4.0f*std::pow(_fx.norm(),4.0f); 
  float la;
  if(fabs(_fx.norm())<FLT_EPSILON)
  {
    la = 0.0f;
  }
  else
  {
    la = (-2.0f*_e1.dot(_fx)*_gammap*_Fd/_d1+sqrt(delta))/(2.0f*std::pow(_fx.norm(),2.0f));
  }

  // if(_s < 0.0f && _pn < 0.0f)
  // {
  //   la = 1.0f;
  // }

  // // Update tank dynamics
  // _pd = _v.transpose()*_D*_v;
  // float  ds = _dt*(_alpha*_pd-_beta*(la-1.0f)*_pn-_gamma*_pf);

  // if(_s+ds>=_smax)
  // {
  //   _s = _smax;
  // }
  // else if(_s+ds<=0.0f)
  // {
  //   _s = 0.0f;
  // }
  // else
  // {
  //   _s+=ds;
  // }

  // // Update robot's power flow
  // _dW = (la-1.0f)*(1-_beta)*_pn+(_gammap-_gamma)*_pf-(1-_alpha)*_pd;

  // Compute modulated DS
  _vd = la*_fx+_gammap*_Fd*_e1/_d1;

  std::cerr << "[FalconControl]: F: " << _normalForce << " Fd:  " << _gammap*_Fd << " ||fx||: " << _fx.norm() << std::endl;
  std::cerr << "[FalconControl]: la: " << la << " vd: " << _vd.norm() << std::endl;
  // std::cerr << "[FalconControl]: Tank: " << _s  <<" dW: " << _dW <<std::endl;


  // Bound modulated DS for safety 
  if(_vd.norm()>_velocityLimit)
  {
    _vd *= _velocityLimit/_vd.norm();
  }

  // std::cerr << "[FalconControl]: vd after scaling: " << _vd.norm() << " distance: " << _normalDistance << " v: " << _v.segment(0,3).norm() <<std::endl;
}


void FalconControl::computeDesiredOrientation()
{
  // Compute rotation error between current orientation and plane orientation using Rodrigues' law
  Eigen::Vector3f ref;
  ref = -_xoD.normalized();  
  ref.normalize();

  // Compute rotation error between current orientation and plane orientation using Rodrigues' law
  Eigen::Vector3f u;
  u = (_wRb.col(2)).cross(ref);
  float c = (_wRb.col(2)).transpose()*ref;  
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
  Eigen::Vector4f qf = Utils::quaternionProduct(qtemp,_q);

  // Perform quaternion slerp interpolation to progressively orient the end effector while approaching the surface
  _normalDistance = (_x-(_xoC+_xoD/2.0f)).dot(_xoD.normalized());
  if(_normalDistance<0.0f)
  {
    _normalDistance = 0.0f;
  }
  std::cerr << _normalDistance << std::endl;
  // std::cerr << 1.0f-std::tanh(3.0f*_normalDistance) << std::endl;
  Eigen::Vector4f q0; 
  q0 << 0.0f,0.0f,1.0f,0.0f;

  // _qd = Utils::slerpQuaternion(q0,qf,1.0f-std::tanh(3.0f*_normalDistance));
  _qd = Utils::slerpQuaternion(q0,qf,Utils::smoothFall(_normalDistance,0.1f,0.6f));

  // Compute needed angular velocity to perform the desired quaternion
  Eigen::Vector4f qcurI, wq;
  qcurI(0) = _q(0);
  qcurI.segment(1,3) = -_q.segment(1,3);
  wq = 5.0f*Utils::quaternionProduct(qcurI,_qd-_q);
  Eigen::Vector3f omegaTemp = _wRb*wq.segment(1,3);
  _omegad = omegaTemp; 
}


// void FalconControl::logData()
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


void FalconControl::publishData()
{
  // Publish desired twist (passive ds controller)
  _msgDesiredTwist.linear.x  = _vd(0);
  _msgDesiredTwist.linear.y  = _vd(1);
  _msgDesiredTwist.linear.z  = _vd(2);

  // Convert desired end effector frame angular velocity to world frame
  _msgDesiredTwist.angular.x = _omegad(0);
  _msgDesiredTwist.angular.y = _omegad(1);
  _msgDesiredTwist.angular.z = _omegad(2);

  _pubDesiredTwist.publish(_msgDesiredTwist);

  // Publish desired orientation
  _msgDesiredOrientation.w = _qd(0);
  _msgDesiredOrientation.x = _qd(1);
  _msgDesiredOrientation.y = _qd(2);
  _msgDesiredOrientation.z = _qd(3);

  _pubDesiredOrientation.publish(_msgDesiredOrientation);

  _msgTaskAttractor.header.frame_id = "world";
  _msgTaskAttractor.header.stamp = ros::Time::now();
  _msgTaskAttractor.point.x = _taskAttractor(0);
  _msgTaskAttractor.point.y = _taskAttractor(1);
  _msgTaskAttractor.point.z = _taskAttractor(2);
  _pubTaskAttractor.publish(_msgTaskAttractor);

  _msgFilteredWrench.header.frame_id = "world";
  _msgFilteredWrench.header.stamp = ros::Time::now();
  _msgFilteredWrench.wrench.force.x = _filteredWrench(0);
  _msgFilteredWrench.wrench.force.y = _filteredWrench(1);
  _msgFilteredWrench.wrench.force.z = _filteredWrench(2);
  _msgFilteredWrench.wrench.torque.x = _filteredWrench(3);
  _msgFilteredWrench.wrench.torque.y = _filteredWrench(4);
  _msgFilteredWrench.wrench.torque.z = _filteredWrench(5);
  _pubFilteredWrench.publish(_msgFilteredWrench);

  _msgFalconForce.header.frame_id = "world";
  _msgFalconForce.header.stamp = ros::Time::now();
  _msgFalconForce.wrench.force.x = _FdFalcon(0);
  _msgFalconForce.wrench.force.y = _FdFalcon(1);
  _msgFalconForce.wrench.force.z = _FdFalcon(2);
  _msgFalconForce.wrench.torque.x = 0.0f;
  _msgFalconForce.wrench.torque.y = 0.0f;
  _msgFalconForce.wrench.torque.z = 0.0f;
  _pubFalconForce.publish(_msgFalconForce);

  std_msgs::Float32 msg;
  msg.data = _normalForce;
  _pubNormalForce.publish(msg);
}


void FalconControl::updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg)
{
  _msgRealPose = *msg;

  Eigen::Vector3f temp = _x;

  // Update end effecotr pose (position+orientation)
  _x << _msgRealPose.position.x, _msgRealPose.position.y, _msgRealPose.position.z;
  _q << _msgRealPose.orientation.w, _msgRealPose.orientation.x, _msgRealPose.orientation.y, _msgRealPose.orientation.z;
  _wRb = Utils::quaternionToRotationMatrix(_q);
  _x = _x+_toolOffsetFromEE*_wRb.col(2);

  if((_x-temp).norm()>FLT_EPSILON)
  {
    _sequenceID++;
  }

  if(!_firstRobotPose)
  {
    _firstRobotPose = true;
    _xd = _x;
    _qd = _q;
    _vd.setConstant(0.0f);
  }
}


void FalconControl::updateRobotTwist(const geometry_msgs::Twist::ConstPtr& msg)
{
  _v(0) = msg->linear.x;
  _v(1) = msg->linear.y;
  _v(2) = msg->linear.z;
  _w(0) = msg->angular.x;
  _w(1) = msg->angular.y;
  _w(2) = msg->angular.z;

  if(!_firstRobotTwist)
  {
    _firstRobotTwist = true;
  }
}


void FalconControl::updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  Eigen::Matrix<float,6,1> raw;
  raw(0) = msg->wrench.force.x;
  raw(1) = msg->wrench.force.y;
  raw(2) = msg->wrench.force.z;
  raw(3) = msg->wrench.torque.x;
  raw(4) = msg->wrench.torque.y;
  raw(5) = msg->wrench.torque.z;

  if(!_wrenchBiasOK && _firstRobotPose)
  {
    Eigen::Vector3f loadForce = _wRb.transpose()*_toolMass*_gravity;
    _wrenchBias.segment(0,3) -= loadForce;
    _wrenchBias.segment(3,3) -= _toolComPositionFromSensor.cross(loadForce);
    _wrenchBias += raw; 
    _wrenchCount++;
    if(_wrenchCount==NB_SAMPLES)
    {
      _wrenchBias /= NB_SAMPLES;
      _wrenchBiasOK = true;
      std::cerr << "[FalconControl]: Bias: " << _wrenchBias.transpose() << std::endl;
    }
  }

  if(_wrenchBiasOK && _firstRobotPose)
  {
    _wrench = raw-_wrenchBias;
    Eigen::Vector3f loadForce = _wRb.transpose()*_toolMass*_gravity;
    _wrench.segment(0,3) -= loadForce;
    _wrench.segment(3,3) -= _toolComPositionFromSensor.cross(loadForce);
    _filteredWrench = _filteredForceGain*_filteredWrench+(1.0f-_filteredForceGain)*_wrench;
  }
}


void FalconControl::optitrackInitialization()
{
  if(_averageCount< AVERAGE_COUNT)
  {
    if(_markersTracked(ROBOT_BASIS_RIGHT) && _markersTracked(ROBOT_BASIS_LEFT))
    {
      _markersPosition0 = (_averageCount*_markersPosition0+_markersPosition)/(_averageCount+1);
      _averageCount++;
    }
    std::cerr << "[FalconControl]: Optitrack Initialization count: " << _averageCount << std::endl;
    if(_averageCount == 1)
    {
      ROS_INFO("[FalconControl]: Optitrack Initialization starting ...");
    }
    else if(_averageCount == AVERAGE_COUNT)
    {
      ROS_INFO("[FalconControl]: Optitrack Initialization done !");
    }
  }
  else
  {
    _optitrackOK = true;
  }
}

void FalconControl::updateOptitrackPose(const geometry_msgs::PoseStamped::ConstPtr& msg, int k) 
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


void FalconControl::updateDampingMatrix(const std_msgs::Float32MultiArray::ConstPtr& msg) 
{
  if(!_firstDampingMatrix)
  {
    _firstDampingMatrix = true;
  }

  _D << msg->data[0],msg->data[1],msg->data[2],
        msg->data[3],msg->data[4],msg->data[5],
        msg->data[6],msg->data[7],msg->data[8];
}


void FalconControl::updateFalconPosition(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  _xFalcon << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  if(!_firstFalconPosition)
  {
    _firstFalconPosition = true;
  }
}


void FalconControl::updateFalconVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  _vFalcon << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z;

  if(!_firstFalconVelocity)
  {
    _firstFalconVelocity = true;
  }
}

void FalconControl::updateFalconButtons(const std_msgs::UInt32::ConstPtr& msg)
{
  _buttonsFalcon = msg->data;

  if(!_firstFalconButtons)
  {
    _firstFalconButtons = true;
  }
}


uint16_t FalconControl::checkTrackedMarker(float a, float b)
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


void FalconControl::dynamicReconfigureCallback(robot_shared_control::falconControl_paramsConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure request. Updatig the parameters ...");

  _filteredForceGain = config.filteredForceGain;
  _velocityLimit = config.velocityLimit;
  _duration = config.duration;
  _kFalcon = config.kFalcon;
  _dFalcon = config.dFalcon;
}
