#ifndef __FEET_TELEMANIPULATION_H__
#define __FEET_TELEMANIPULATION_H__

#include <signal.h>
#include <mutex>
#include <fstream>
#include <pthread.h>
#include <vector>
#include <deque>
#include "ros/ros.h"
#include <ros/package.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32.h"
#include "visualization_msgs/Marker.h"
#include <dynamic_reconfigure/server.h>
#include "robot_shared_control/feetTelemanipulation_paramsConfig.h"
#include "custom_msgs/FootInputMsg.h"
#include "custom_msgs/FootOutputMsg.h"
#include "Eigen/Eigen"

#define NB_ROBOTS 2
#define NB_SAMPLES 50
#define FOOT_INTERFACE_X_RANGE_RIGHT 0.350
#define FOOT_INTERFACE_Y_RANGE_RIGHT 0.293
#define FOOT_INTERFACE_PHI_RANGE_RIGHT 35
#define FOOT_INTERFACE_X_RANGE_LEFT 0.350
#define FOOT_INTERFACE_Y_RANGE_LEFT 0.293
#define FOOT_INTERFACE_PHI_RANGE_LEFT 35
#define WINDOW_SIZE 10


class FeetTelemanipulation 
{
	public:
    // Robot ID, left or right
		enum ROBOT {LEFT = 0, RIGHT = 1};

		enum Strategy {PURE_TELEMANIPULATION = 0, AUTONOMOUS_LOAD_SUPPORT = 1};

	private:
		// ROS variables
		ros::NodeHandle _n;
		ros::Rate _loopRate;
		float _dt;

		// Subscribers declarations
		ros::Subscriber _subRobotPose[NB_ROBOTS];							// robot pose
		ros::Subscriber _subRobotTwist[NB_ROBOTS];						// robot twist
		ros::Subscriber _subForceTorqueSensor[NB_ROBOTS];			// force torque sensor
		ros::Subscriber _subDampingMatrix[NB_ROBOTS];					// Damping matrix of DS-impedance controller
		ros::Subscriber _subFootInterfacePose[NB_ROBOTS];
		ros::Subscriber _subFootInterfaceWrench[NB_ROBOTS];
		ros::Subscriber _subFootOutput[NB_ROBOTS];

		// Publisher declaration
		ros::Publisher _pubDesiredTwist[NB_ROBOTS];						// Desired twist to DS-impdedance controller
		ros::Publisher _pubCommand;							   						// Desired twist to DS-impdedance controller
		ros::Publisher _pubDesiredOrientation[NB_ROBOTS];  		// Desired orientation to DS-impedance controller
		ros::Publisher _pubFilteredWrench[NB_ROBOTS];					// Filtered measured wrench
		ros::Publisher _pubNormalForce[NB_ROBOTS];							// Measured normal force
    ros::Publisher _pubDesiredFootWrench[NB_ROBOTS];                          // Marker (RVIZ) 
    ros::Publisher _pubFootInput[NB_ROBOTS];                          // Marker (RVIZ) 
		
		// Messages declaration
		geometry_msgs::Pose _msgRealPose;
		geometry_msgs::Pose _msgDesiredPose;
		geometry_msgs::Quaternion _msgDesiredOrientation;
		geometry_msgs::Twist _msgDesiredTwist;
		geometry_msgs::WrenchStamped _msgFilteredWrench;
		geometry_msgs::Wrench _msgDesiredFootWrench;
		custom_msgs::FootInputMsg _msgFootInput;
		
		// Tool characteristics
		float _toolMass;														// Tool mass [kg]
		float _toolOffsetFromEE;										// Tool offset along z axis of end effector [m]							
		Eigen::Vector3f _toolComPositionFromSensor;   // Offset of the tool [m]	(3x1)
		Eigen::Vector3f _gravity;										// Gravity vector [m/s^2] (3x1)

		// Tool state variables
		Eigen::Vector3f _x[NB_ROBOTS];													// Position [m] (3x1)
		Eigen::Vector3f _x0[NB_ROBOTS];													// Position [m] (3x1)
		Eigen::Vector4f _q[NB_ROBOTS];													// Quaternion (4x1)
		Eigen::Matrix3f _wRb[NB_ROBOTS];												// Orientation matrix (3x1) (form end effector to world frame)
		Eigen::Vector3f _v[NB_ROBOTS];													// Velocity [m/s] (3x1)
		Eigen::Vector3f _w[NB_ROBOTS];													// Angular velocity [rad/s] (3x1)
		Eigen::Matrix<float,6,1> _wrench[NB_ROBOTS];						// Wrench [N and Nm] (6x1)
		Eigen::Matrix<float,6,1> _wrenchBias[NB_ROBOTS];				// Wrench bias [N and Nm] (6x1)
		Eigen::Matrix<float,6,1> _filteredWrench[NB_ROBOTS];		// Filtered wrench [N and Nm] (6x1)
    float _normalForce[NB_ROBOTS];													// Normal force to the surface [N]
    Eigen::Vector3f _leftRobotOrigin;

		// Tool control variables
		Eigen::Vector3f _xd[NB_ROBOTS];				// Desired position [m] (3x1)
		Eigen::Vector4f _qd[NB_ROBOTS];				// Desired quaternion (4x1)
		Eigen::Vector3f _omegad[NB_ROBOTS];		// Desired angular velocity [rad/s] (3x1)
		Eigen::Vector3f _fx[NB_ROBOTS];				// Desired Nominal velociy [m/s] (3x1)
		Eigen::Vector3f _vd[NB_ROBOTS];				// Desired modulated velocity [m/s] (3x1)
		float _targetForce;										// Target force in contact [N]
		float _Fd[NB_ROBOTS];									// Desired force profile
		float _graspingForceThreshold;

    // Foot interface variables
    Eigen::Matrix<float,6,1> _footPose[NB_ROBOTS];
    Eigen::Vector3f _footPosition[NB_ROBOTS];
    Eigen::Matrix<float,6,1> _footWrench[NB_ROBOTS];
    Eigen::Matrix<float,6,1> _footTwist[NB_ROBOTS];
		Eigen::Matrix<float,6,1> _desiredFootWrench[NB_ROBOTS];		// Filtered wrench [N and Nm] (6x1)
    uint32_t _footInterfaceSequenceID[NB_ROBOTS];
    int _footState[NB_ROBOTS];
    float _xyPositionMapping;
    float _zPositionMapping;
    Eigen::Vector3f _vdFoot[NB_ROBOTS];
    Eigen::Vector3f _xdFoot[NB_ROBOTS];
    Eigen::Vector3f _FdFoot[NB_ROBOTS];

    // Booleans
    bool _useLeftRobot;
    bool _useRightRobot;
		bool _firstRobotPose[NB_ROBOTS];						// Monitor the first robot pose update
		bool _firstRobotTwist[NB_ROBOTS];						// Monitor the first robot twist update
		bool _firstWrenchReceived[NB_ROBOTS];				// Monitor first force/torque data update
		bool _firstDampingMatrix[NB_ROBOTS];				// Monitor first damping matrix update
		bool _wrenchBiasOK[NB_ROBOTS];							// Check if computation of force/torque sensor bias is OK
		bool _stop;																	// Check for CTRL+C
		bool _objectGrasped;												
		bool _firstFootInterfacePose[NB_ROBOTS];
		bool _firstFootInterfaceWrench[NB_ROBOTS];
		bool _firstFootOutput[NB_ROBOTS];
		
		// User variables
		float _velocityLimit;				// Velocity limit [m/s]
		float _filteredForceGain;		// Filtering gain for force/torque sensor
		float _kxy;		
		float _dxy;		
		float _kphi;		
		float _dphi;		
		bool _useSharedControl;
		bool _useIIWA;
		Eigen::Matrix3f _Riiwa;
		std_msgs::Float64MultiArray _msgCommand;


		// Other variables
    double _timeInit;
		int _wrenchCount[NB_ROBOTS];
		Eigen::Matrix3f _D[NB_ROBOTS];
		float _d1[NB_ROBOTS];
		uint32_t _sequenceID;
		std::string _filename;
		std::ifstream _inputFile;
		std::ofstream _outputFile;
		std::mutex _mutex;
		Strategy _strategy;
    float _normalForceAverage[NB_ROBOTS];
		std::deque<float> _normalForceWindow[NB_ROBOTS];
		static FeetTelemanipulation* me;

		// Dynamic reconfigure (server+callback)
		dynamic_reconfigure::Server<robot_shared_control::feetTelemanipulation_paramsConfig> _dynRecServer;
		dynamic_reconfigure::Server<robot_shared_control::feetTelemanipulation_paramsConfig>::CallbackType _dynRecCallback;

	public:

		// Class constructor
		FeetTelemanipulation(ros::NodeHandle &n, double frequency, std::string filename);

		// Initialize node
		bool init();

		// Run node
		void run();

	private:
		
		// Callback called when CTRL is detected to stop the node
		static void stopNode(int sig);

		bool allDataReceived();
		
		// Compute command to be sent to the DS-impedance controller
    void computeCommand();
    
    void updateObjectGraspingState();

    void pureTelemanipulation();

    void autonomousLoadSupport();
        
    void footDataTransformation();

    void positionVelocityMapping();

    void positionPositionMapping();

  	void computeDesiredFootWrench();

		// Compute desired orientation
		void computeDesiredOrientation();
    
  	// Log data to text file
    void logData();

    // Publish data to topics
    void publishData();
    
    // Callback to update the robot pose
    void updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg, int k);

    // Callback to update the robot twist
    void updateRobotTwist(const geometry_msgs::Twist::ConstPtr& msg, int k);

    // Callback to update the robot wrench (force/torque sensor data)
    void updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg, int k);

		// Callback to update damping matrix form the DS-impedance controller
    void updateDampingMatrix(const std_msgs::Float32MultiArray::ConstPtr& msg, int k); 

    // Callback to update data from foot interface
		void updateFootOutput(const custom_msgs::FootOutputMsg::ConstPtr& msg, int k); 

    // Callback for dynamic reconfigure
    void dynamicReconfigureCallback(robot_shared_control::feetTelemanipulation_paramsConfig &config, uint32_t level);
};


#endif
