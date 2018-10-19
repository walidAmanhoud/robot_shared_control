#ifndef __FALCON_CONTROL_H__
#define __FALCON_CONTROL_H__

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
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt32.h"
#include "visualization_msgs/Marker.h"
#include <dynamic_reconfigure/server.h>
#include "robot_shared_control/falconControl_paramsConfig.h"
#include "Eigen/Eigen"

#define NB_SAMPLES 50
#define AVERAGE_COUNT 100
#define TOTAL_NB_MARKERS 6
#define NB_ROBOTS 2
#define FALCON_INTERFACE_X_RANGE 0.1
#define FALCON_INTERFACE_Y_RANGE 0.1
#define FALCON_INTERFACE_Z_RANGE 0.1
#define NB_TASKS 4

class FalconControl 
{
	public:
		// Optitrack makers ID
  	enum MarkersID {ROBOT_BASIS_LEFT = 0, ROBOT_BASIS_RIGHT = 1, P1 = 2, P2 = 3, P3 = 4, P4 = 5};

  	enum Buttons {CENTER = 1, LEFT = 2, TOP = 4, RIGHT = 8};

	private:
		// ROS variables
		ros::NodeHandle _nh;
		ros::Rate _loopRate;
		float _dt;

		// Subscribers declarations
		ros::Subscriber _subRobotPose;												// robot pose
		ros::Subscriber _subRobotTwist;												// robot twist
		ros::Subscriber _subForceTorqueSensor;								// force torque sensor
		ros::Subscriber _subOptitrackPose[TOTAL_NB_MARKERS];	// optitrack markers pose
		ros::Subscriber _subDampingMatrix;										// Damping matrix of DS-impedance controller
		ros::Subscriber _subFalconPosition;
		ros::Subscriber _subFalconVelocity;
		ros::Subscriber _subFalconButtons;

		// Publisher declaration
		ros::Publisher _pubDesiredTwist;						// Desired twist to DS-impdedance controller
		ros::Publisher _pubDesiredOrientation;  		// Desired orientation to DS-impedance controller
		ros::Publisher _pubFilteredWrench;					// Filtered measured wrench
		ros::Publisher _pubTaskAttractor;						// Attractor on surface (RVIZ)
		ros::Publisher _pubNormalForce;							// Measured normal force to the surface
		ros::Publisher _pubFalconForce;							// Measured normal force to the surface
		
		// Messages declaration
		geometry_msgs::Pose _msgRealPose;
		geometry_msgs::Pose _msgDesiredPose;
		geometry_msgs::Quaternion _msgDesiredOrientation;
		geometry_msgs::Twist _msgDesiredTwist;
		geometry_msgs::PointStamped _msgTaskAttractor;
		geometry_msgs::WrenchStamped _msgFilteredWrench;
		geometry_msgs::WrenchStamped _msgFalconForce;
		
		// Tool characteristics
		float _toolMass;														// Tool mass [kg]
		float _toolOffsetFromEE;										// Tool offset along z axis of end effector [m]							
		Eigen::Vector3f _toolComPositionFromSensor;   // Offset of the tool [m]	(3x1)
		Eigen::Vector3f _gravity;										// Gravity vector [m/s^2] (3x1)

		// Tool state variables
		Eigen::Vector3f _x;													// Position [m] (3x1)
		Eigen::Vector4f _q;													// Quaternion (4x1)
		Eigen::Matrix3f _wRb;												// Orientation matrix (3x1) (form end effector to world frame)
		Eigen::Vector3f _v;													// Velocity [m/s] (3x1)
		Eigen::Vector3f _w;													// Angular velocity [rad/s] (3x1)
		Eigen::Matrix<float,6,1> _wrench;						// Wrench [N and Nm] (6x1)
		Eigen::Matrix<float,6,1> _wrenchBias;				// Wrench bias [N and Nm] (6x1)
		Eigen::Matrix<float,6,1> _filteredWrench;		// Filtered wrench [N and Nm] (6x1)
    float _normalDistance;											// Normal distance to the surface [m]
    float _normalForce;													// Normal force to the surface [N]

		// Tool control variables
		Eigen::Vector3f _xd;				// Desired position [m] (3x1)
		Eigen::Vector4f _qd;				// Desired quaternion (4x1)
		Eigen::Vector3f _omegad;		// Desired angular velocity [rad/s] (3x1)
		Eigen::Vector3f _fx;				// Desired Nominal velociy [m/s] (3x1)
		Eigen::Vector3f _vd;				// Desired modulated velocity [m/s] (3x1)
		float _targetForce;					// Target force in contact [N]
		float _targetVelocity;			// Velocity norm of the nominal DS [m/s]
		float _Fd;									// Desired force profile

		// Task variables
    Eigen::Vector3f _taskAttractor;				// Attractor position [m] (3x1)
    Eigen::Vector3f _planeNormal;					// Normal vector to the surface (pointing outside the surface) (3x1)
    Eigen::Vector3f _n;									// Normal vector to the surface (pointing towards the surface) (3x1)
    Eigen::Vector3f _p;										// Point on the surface [m] (3x1)
    Eigen::Vector3f _xProj;								// Vertical projection on the surface [m] (3x1)
    Eigen::Matrix3f _wRs;									// Orientation matrix from surface frame to world frame
    Eigen::Vector3f _xAttractor[NB_TASKS];  				

    // Falcon interface variables
    Eigen::Vector3f _xFalcon;
    Eigen::Vector3f _vFalcon;
    Eigen::Vector3f _xM;
    Eigen::Vector3f _vM;
    Eigen::Vector3f _vh;
    Eigen::Vector3f _FdFalcon;
   	float _kFalcon;
   	float _dFalcon; 
   	uint32_t _buttonsFalcon;
    float _sigmaH;
    float _bs = 0.0f;


   	// Task adaptation parameters
   	Eigen::Matrix<float,NB_TASKS,1> _beliefs;
   	Eigen::Matrix<float,NB_TASKS,1> _dbeliefs;   	
   	Eigen::Vector3f _fxk[NB_TASKS];
   	Eigen::Vector3f _nk[NB_TASKS];
   	float _Fdk[NB_TASKS];
   	float _adaptationRate;

    // Object variables
    Eigen::Vector3f _objectDim;       // Object dimensions [m] (3x1)
    Eigen::Vector3f _xoC;             // Measured object center position [m] (3x1)
    Eigen::Vector3f _xoD;             // Measured object dimension vector [m] (3x1)

    // Booleans
		bool _firstRobotPose;																// Monitor the first robot pose update
		bool _firstRobotTwist;															// Monitor the first robot twist update
		bool _firstWrenchReceived;													// Monitor first force/torque data update
    bool _firstOptitrackPose[TOTAL_NB_MARKERS];					// Monitor first optitrack markers update
		bool _firstDampingMatrix;														// Monitor first damping matrix update
		bool _optitrackOK;																	// Check if all markers position is received
		bool _wrenchBiasOK;																	// Check if computation of force/torque sensor bias is OK
		bool _stop;																					// Check for CTRL+C
		bool _firstFalconPosition;
		bool _firstFalconVelocity;
		bool _firstFalconButtons;
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
    Eigen::Vector3f _offset;		// Attractor offset on surface [m] (3x1)
    double _duration;						// Duration of an experiment [s]

    // Force adaptation vaariable
    float _E;
    float _Et;
    float _Emin;
    float _Emax;
    float _epsilon;
    float _sigma;
    float _h;
    float _tankRate;

		// Other variables
    double _timeInit;
		uint32_t _averageCount = 0;
		int _wrenchCount = 0;
		Eigen::Matrix3f _D;
		float _d1;
		uint32_t _sequenceID;
		std::string _filename;
		std::ifstream _inputFile;
		std::ofstream _outputFile;
		std::mutex _mutex;
		static FalconControl* me;

		// Dynamic reconfigure (server+callback)
		dynamic_reconfigure::Server<robot_shared_control::falconControl_paramsConfig> _dynRecServer;
		dynamic_reconfigure::Server<robot_shared_control::falconControl_paramsConfig>::CallbackType _dynRecCallback;

	public:

		// Class constructor
		FalconControl(ros::NodeHandle &n, double frequency, std::string filename);

		// Initialize node
		bool init();

		// Run node
		void run();

	private:
		
		// Callback called when CTRL is detected to stop the node
		static void stopNode(int sig);

		void computeObjectPose();
		
		// Compute command to be sent to the DS-impedance controller
    void computeCommand();

    void taskAdaptation();

    void forceAdaptation();

    void falconDataTransformation();

    void positionVelocityMapping();

    void computeFalconForce();

    void updateIndividualTasks();

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
    void updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg);

    // Callback to update the robot twist
    void updateRobotTwist(const geometry_msgs::Twist::ConstPtr& msg);

    // Callback to update the robot wrench (force/torque sensor data)
    void updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg);

    // Callback to update markers pose from Optitrack
		void updateOptitrackPose(const geometry_msgs::PoseStamped::ConstPtr& msg, int k); 

    // Callback to update data from foot interface
		void updateFalconPosition(const geometry_msgs::PoseStamped::ConstPtr& msg); 

		void updateFalconVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg); 

		void updateFalconButtons(const std_msgs::UInt32::ConstPtr& msg); 

		// Check if the marker is tracked
		uint16_t checkTrackedMarker(float a, float b);

		// Callback to update damping matrix form the DS-impedance controller
    void updateDampingMatrix(const std_msgs::Float32MultiArray::ConstPtr& msg); 

    // Callback for dynamic reconfigure
    void dynamicReconfigureCallback(robot_shared_control::falconControl_paramsConfig &config, uint32_t level);
};


#endif
