#ifndef __FOOT_CONTROL_H__
#define __FOOT_CONTROL_H__

#include <signal.h>
#include <mutex>
#include <fstream>
#include <pthread.h>
#include <vector>
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
#include "std_msgs/Float32.h"
#include "visualization_msgs/Marker.h"
#include <dynamic_reconfigure/server.h>
#include "robot_shared_control/footControl_paramsConfig.h"
#include "custom_msgs/FootInputMsg.h"
#include "custom_msgs/FootOutputMsg.h"
#include "Eigen/Eigen"
#include "svm_grad.h"
#include "sg_filter.h"

#define NB_SAMPLES 50
#define AVERAGE_COUNT 100
#define TOTAL_NB_MARKERS 6
#define NB_ROBOTS 2
#define FOOT_INTERFACE_X_RANGE_RIGHT 0.350
#define FOOT_INTERFACE_Y_RANGE_RIGHT 0.293
#define FOOT_INTERFACE_PHI_RANGE_RIGHT 35
#define FOOT_INTERFACE_X_RANGE_LEFT 0.350
#define FOOT_INTERFACE_Y_RANGE_LEFT 0.293
#define FOOT_INTERFACE_PHI_RANGE_LEFT 35
#define NB_TASKS 3


class FootControl 
{
	public:
		// Optitrack makers ID
  	enum MarkersID {ROBOT_BASIS_LEFT = 0, ROBOT_BASIS_RIGHT = 1, P1 = 2, P2 = 3, P3 = 4, P4 = 5};

    // Robot ID, left or right
		enum ROBOT {LEFT = 0, RIGHT = 1};


	private:
		// ROS variables
		ros::NodeHandle _n;
		ros::Rate _loopRate;
		float _dt;

		// Subscribers declarations
		ros::Subscriber _subRobotPose[NB_ROBOTS];												// robot pose
		ros::Subscriber _subRobotTwist[NB_ROBOTS];												// robot twist
		ros::Subscriber _subForceTorqueSensor[NB_ROBOTS];								// force torque sensor
		ros::Subscriber _subOptitrackPose[TOTAL_NB_MARKERS];	// optitrack markers pose
		ros::Subscriber _subDampingMatrix[NB_ROBOTS];										// Damping matrix of DS-impedance controller
		ros::Subscriber _subFootInterfacePose[NB_ROBOTS];
		ros::Subscriber _subFootInterfaceWrench[NB_ROBOTS];
		ros::Subscriber _subFootOutput[NB_ROBOTS];
		ros::Subscriber _subFootSensor[NB_ROBOTS];

		// Publisher declaration
		ros::Publisher _pubDesiredTwist[NB_ROBOTS];						// Desired twist to DS-impdedance controller
		ros::Publisher _pubDesiredOrientation[NB_ROBOTS];  		// Desired orientation to DS-impedance controller
		ros::Publisher _pubFilteredWrench[NB_ROBOTS];					// Filtered measured wrench
		ros::Publisher _pubTaskAttractor;						// Attractor on surface (RVIZ)
		ros::Publisher _pubNormalForce[NB_ROBOTS];							// Measured normal force to the surface
    ros::Publisher _pubMarker;                          // Marker (RVIZ) 
    ros::Publisher _pubDesiredFootWrench[NB_ROBOTS];                          // Marker (RVIZ) 
    ros::Publisher _pubFootInput[NB_ROBOTS];                          // Marker (RVIZ) 
    ros::Publisher _pubFootSensor[NB_ROBOTS];                          // Marker (RVIZ) 

		
		// Messages declaration
		geometry_msgs::Pose _msgRealPose;
		geometry_msgs::Pose _msgDesiredPose;
		geometry_msgs::Quaternion _msgDesiredOrientation;
		geometry_msgs::Twist _msgDesiredTwist;
		geometry_msgs::PointStamped _msgTaskAttractor;
		geometry_msgs::WrenchStamped _msgFilteredWrench;
		geometry_msgs::Wrench _msgDesiredFootWrench;
		geometry_msgs::WrenchStamped _msgFootSensor;
		custom_msgs::FootInputMsg _msgFootInput;
    visualization_msgs::Marker _msgMarker;

		
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
		Eigen::Matrix<float,6,1> _desiredFootWrench[NB_ROBOTS];		// Filtered wrench [N and Nm] (6x1)
    float _normalDistance[NB_ROBOTS];											// Normal distance to the surface [m]
    float _normalForce[NB_ROBOTS];													// Normal force to the surface [N]

		// Tool control variables
		Eigen::Vector3f _xd[NB_ROBOTS];				// Desired position [m] (3x1)
		Eigen::Vector4f _qd[NB_ROBOTS];				// Desired quaternion (4x1)
		Eigen::Vector3f _omegad[NB_ROBOTS];		// Desired angular velocity [rad/s] (3x1)
		Eigen::Vector3f _fx[NB_ROBOTS];				// Desired Nominal velociy [m/s] (3x1)
		Eigen::Vector3f _vd[NB_ROBOTS];				// Desired modulated velocity [m/s] (3x1)
		float _targetForce;					// Target force in contact [N]
		float _targetVelocity;			// Velocity norm of the nominal DS [m/s]
		float _Fd[NB_ROBOTS];									// Desired force profile

		// Task variables
    Eigen::Vector3f _taskAttractor;				// Attractor position [m] (3x1)
    Eigen::Vector3f _planeNormal;					// Normal vector to the surface (pointing outside the surface) (3x1)
    Eigen::Vector3f _e1[NB_ROBOTS];									// Normal vector to the surface (pointing towards the surface) (3x1)
    Eigen::Vector3f _p;										// Point on the surface [m] (3x1)
    Eigen::Vector3f _xProj;								// Vertical projection on the surface [m] (3x1)
    Eigen::Matrix3f _wRs;									// Orientation matrix from surface frame to world frame

    // Foot interface variables
    Eigen::Matrix<float,6,1> _footPose[NB_ROBOTS];
    Eigen::Matrix<float,6,1> _footSensor[NB_ROBOTS];
    Eigen::Matrix<float,6,1> _footWrench[NB_ROBOTS];
    Eigen::Matrix<float,6,1> _footTwist[NB_ROBOTS];
    Eigen::Matrix<float,6,1> _footDesiredWrench[NB_ROBOTS];
    Eigen::Vector3f _footPosition[NB_ROBOTS];
    uint32_t _footInterfaceSequenceID[NB_ROBOTS];
    Eigen::Vector3f _vh[NB_ROBOTS];
    Eigen::Vector3f _xh[NB_ROBOTS];
    float _xyPositionMapping;
    float _zPositionMapping;
    int _footState[NB_ROBOTS];
		Eigen::Matrix<float,6,1> _filteredFootSensor[NB_ROBOTS];		// Filtered wrench [N and Nm] (6x1)


    // Object variables
    Eigen::Vector3f _objectDim;       // Object dimensions [m] (3x1)
    Eigen::Vector3f _xoC;             // Measured object center position [m] (3x1)
    Eigen::Vector3f _xoD;             // Measured object dimension vector [m] (3x1)
    Eigen::Vector3f _xdD;             // Desired distance vector [m] (3x1)

    SGF::SavitzkyGolayFilter _xCFilter;
    SGF::SavitzkyGolayFilter _xDFilter;
    SGF::SavitzkyGolayFilter _zDirFilter;
    Eigen::Vector3f _xC;              // Center position between the two robots [m] (3x1)
    Eigen::Vector3f _xD;              // Distance vector between the two robots [m] (3x1)
    float _eoD;                       // Error to object dimension vector [m]                       
    float _eoC;                       // Error to object center position [m]  
    float _graspingForceThreshold;  // Grasping force threshold [N]
    bool _objectGrasped;                                // Check if the object is grasped


    // Booleans
    bool _useLeftRobot;
    bool _useRightRobot;
		bool _firstRobotPose[NB_ROBOTS];																// Monitor the first robot pose update
		bool _firstRobotTwist[NB_ROBOTS];															// Monitor the first robot twist update
		bool _firstWrenchReceived[NB_ROBOTS];													// Monitor first force/torque data update
    bool _firstOptitrackPose[TOTAL_NB_MARKERS];					// Monitor first optitrack markers update
		bool _firstDampingMatrix[NB_ROBOTS];														// Monitor first damping matrix update
		bool _optitrackOK;																	// Check if all markers position is received
		bool _wrenchBiasOK[NB_ROBOTS];																	// Check if computation of force/torque sensor bias is OK
		bool _stop;																					// Check for CTRL+C
		bool _firstFootInterfacePose[NB_ROBOTS];
		bool _firstFootInterfaceWrench[NB_ROBOTS];
		bool _firstFootOutput[NB_ROBOTS];
		bool _firstFootSensor[NB_ROBOTS];
		bool _firstObjectPose;

    // Optitrack variables
    Eigen::Matrix<float,3,TOTAL_NB_MARKERS> _markersPosition;       // Markers position in optitrack frame
    Eigen::Matrix<float,3,TOTAL_NB_MARKERS> _markersPosition0;			// Initial markers position in opittrack frame
    Eigen::Matrix<uint32_t,TOTAL_NB_MARKERS,1> _markersSequenceID;	// Markers sequence ID
    Eigen::Matrix<uint16_t,TOTAL_NB_MARKERS,1> _markersTracked;			// Markers tracked state
		Eigen::Vector3f _p1;																						// First marker position in the robot frame
		Eigen::Vector3f _p2;																						// Second marker position in the robot frame
		Eigen::Vector3f _p3;																						// Third marker position in the robot frame
		Eigen::Vector3f _p4;																						// Fourth marker position in the robot frame
    Eigen::Vector3f _leftRobotOrigin;                               // Left robot basis position in the right robot frame

    Eigen::Vector3f _xAttractor[NB_TASKS];  				
   	Eigen::Matrix<float,NB_TASKS,1> _beliefs;
   	Eigen::Matrix<float,NB_TASKS,1> _dbeliefs;   	
   	Eigen::Vector3f _fxk[NB_TASKS];
   	float _Fdk[NB_TASKS];
   	float _adaptationRate;

		// Tank parameters
		float _s;					// Current tank level
		float _smax;			// Max tank level
		float _alpha;			// Scalar variable controlling the flow of the dissipated energy
		float _beta;			// Scalar variable controlling the flow of the energy due to the nominal DS
		float _gamma;     // Scalar variable controlling the flow of the energy due to the contact force
		float _gammap;    // Scalar variable adapting the control low to ensure passivity
		float _pn;				// Power due to the nominal DS
		float _pf;				// Power due to the contact force
		float _pd;				// Dissipated power
		float _dW;				// Robot's power flow
		
		// User variables
		float _velocityLimit;				// Velocity limit [m/s]
		float _filteredForceGain;		// Filtering gain for force/torque sensor
    double _duration;						// Duration of an experiment [s]
		float _kxy;		
		float _dxy;		
		float _kphi;		
		float _dphi;		
		// Other variables
    double _timeInit;
		uint32_t _averageCount = 0;
		int _wrenchCount[NB_ROBOTS];
		Eigen::Matrix3f _D[NB_ROBOTS];
		float _d1[NB_ROBOTS];
		uint32_t _sequenceID;
		std::string _filename;
		std::ifstream _inputFile;
		std::ofstream _outputFile;
		SVMGrad _svm;
		std::mutex _mutex;
		static FootControl* me;

		// Dynamic reconfigure (server+callback)
		dynamic_reconfigure::Server<robot_shared_control::footControl_paramsConfig> _dynRecServer;
		dynamic_reconfigure::Server<robot_shared_control::footControl_paramsConfig>::CallbackType _dynRecCallback;

	public:

		// Class constructor
		FootControl(ros::NodeHandle &n, double frequency, std::string filename);

		// Initialize node
		bool init();

		// Run node
		void run();

	private:
		
		// Callback called when CTRL is detected to stop the node
		static void stopNode(int sig);

		bool allDataReceived();

		void computeObjectPose();
		
		// Compute command to be sent to the DS-impedance controller
    void computeCommand();

    void updateIndividualTasks();
    
    void taskAdaptation();
    

    void footDataTransformation();

    void positionVelocityMapping();

    void positionPositionMapping();

  	void computeDesiredFootWrench();

    // Update surface info (normal vector and distance)
		// void updateSurfaceInformation();

		// Compute nominal DS
		// void computeNominalDS();

		// Generate circular motion dynamics
		// Eigen::Vector3f getCircularMotionVelocity(Eigen::Vector3f position, Eigen::Vector3f attractor);

		// Update scalar variables controlling the tank dynamics
		// void updateTankScalars();

		// Compute modulated DS
		void computeModulatedDS();

		// Compute desired orientation
		void computeDesiredOrientation();
    
  	// Log data to text file
    void logData();

    // Publish data to topics
    void publishData();
    
    // Compute inital markers positon
    void optitrackInitialization();

    // Callback to update the robot pose
    void updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg, int k);

    // Callback to update the robot twist
    void updateRobotTwist(const geometry_msgs::Twist::ConstPtr& msg, int k);

    // Callback to update the robot wrench (force/torque sensor data)
    void updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg, int k);

		// Callback to update damping matrix form the DS-impedance controller
    void updateDampingMatrix(const std_msgs::Float32MultiArray::ConstPtr& msg, int k); 

    // Callback to update markers pose from Optitrack
		void updateOptitrackPose(const geometry_msgs::PoseStamped::ConstPtr& msg, int k); 

    // Callback to update data from foot interface
		void updateFootInterfacePose(const geometry_msgs::PoseStamped::ConstPtr& msg, int k); 

    // Callback to update data from foot interface
		void updateFootInterfaceWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg, int k); 

		void updateFootOutput(const custom_msgs::FootOutputMsg::ConstPtr& msg, int k); 

		void updateFootSensor(const geometry_msgs::Wrench::ConstPtr& msg, int k);

		// Check if the marker is tracked
		uint16_t checkTrackedMarker(float a, float b);


    // Callback for dynamic reconfigure
    void dynamicReconfigureCallback(robot_shared_control::footControl_paramsConfig &config, uint32_t level);
};


#endif
