#include "C:\Users\dzdun\Documents\Arduino\CollisionAvoidRTH\libraries\mavlink\mavlink.h"
/*///////////////////////////////////////////////////////////
// SONAR VARIABLES
///////////////////////////////////////////////////////////*/
const int PW_PIN = 52; // Pulse Width Pin connection on Arduino MEGA to MaxBotix LV-EZ
const int ARRAY_SIZE = 5; // Size of array to store series of sonar ping measurements, must be an odd number for mode/median functions
long pulse; // Variable to store sonar ping value in microseconds
const int OBSTACLE_DETECT_DIST_INT = 300; // Trigger distance for obstacle detection in centimeters as integer
const float OBSTACLE_DETECT_DIST_FLOAT = (float)300.00; // Trigger distance for obstacle detection in centimeters as float
/*///////////////////////////////////////////////////////////
// MAVLINK VARIABLES
///////////////////////////////////////////////////////////*/
const uint8_t SYSTEM_ID = 100; // System ID of the Arduino MEGA
const uint8_t COMPONENT_ID = 50; //Component ID of the Arduino MEGA
uint8_t received_sysid;// Variable to hold system id of transmitted mavstream
uint8_t received_compid;// Variable to hold component id id of transmitted mavstream
/*///////////////////////////////////////////////////////////
// FLAGS
///////////////////////////////////////////////////////////*/
bool setParams = false; // Flag to ensure ArduCopter parameters for Waypoint Behaviour, RTL and Autonomous Flight speed, and current waypoint message frequency are set only once
bool obstacleFlag = false; //Flag to confirm the modes of two sets of sonar distance measurements concur in detecting an obstacle, warns an obstacle may have been detected
bool kalmanObstacleConfirmed = false; // Flag to confirm two kalman filtered sonar distance measurements concur in detecting an obstacle, confirms an obstacle is detected
bool currentCoordinatesReceived = false; // Flag to confirm most recent current coordinates have been received to calculate reposition coordinates
enum STAGEFLAG {	//Enumeration of the program stages
	STAGE0 = 0,		//STAGE0 - Drone has not activated RTL mode
	STAGE1 = 1,		//STAGE1 - Drone has activated RTL and continuously scans for obstacles
	STAGE2 = 2,		//STAGE2 - Drone has detected an obstacle, calculates reposition coordinates, uploads reposition mission to flight controller and activates the mission
	STAGE3 = 3		//STAGE3 - Drone is conducting reposition mission while scanning for obstacles and reverts to STAGE1 when reposition coordinates are reached
};
STAGEFLAG currentStage = STAGE0; // Sets beginning default stage for test flight
uint8_t currentMode; //Variable to store flight mode of drone, RTL mode is 6
// Current flight mode of the drone based on arducopter custom modes i.e.:
/*
#define STABILIZE 0 // hold level position
#define ACRO 1 // rate control
#define ALT_HOLD 2 // AUTO control
#define AUTO 3 // AUTO control
#define GUIDED 4 // AUTO control
#define LOITER 5 // Hold a single location
#define RTL 6 // AUTO control
#define CIRCLE 7 // AUTO control
#define POSITION 8 // AUTO control
#define LAND 9 // AUTO control
#define OF_LOITER 10 // Hold a single location using optical flow // sensor
#define TOY_A 11 // THOR Enum for Toy mode
#define TOY_M 12 // THOR Enum for Toy mode
#define NUM_MODES 13
#define BRAKE_MODE 17

AltHold (quad disarmed): Base_Mode=81, Custom_Mode=2
Loiter (quad disarmed): Base_Mode=89, Custom_Mode=5
Auto (quad disarmed): Base_Mode=89, Custom_Mode=3
RTL (quad disarmed): Base_Mode=89, Custom_Mode=6
AltHold (quad armed): Base_Mode=209, Custom_Mode=2
Loiter (quad armed): Base_Mode=217, Custom_Mode=5
Auto (quad armed): Base_Mode=217, Custom_Mode=3
RTL (quad armed): Base_Mode=217, Custom_Mode=6
*/
/*///////////////////////////////////////////////////////////
// NAVIGATION VARIABLES
///////////////////////////////////////////////////////////*/
uint16_t currentMissionSequenceNum = NULL; //Index of the current mission item being executed by the drone
int32_t latCurrent = NULL; // Variable to store current latitude
int32_t longCurrent = NULL; // Variable to store current longitude 
uint16_t headingCurrent = NULL; // Variable to store current heading
float latTargetFloat = NULL; // Target latitude calculated for reposition
float longTargetFloat = NULL; // Target longitude calculated for reposition
//int32_t latTarget = NULL; // Target latitude calculated for reposition as int32_t
//int32_t longTarget = NULL; // Target longitude calculated for reposition as int32_t
const uint16_t NEW_COOR_BEARING = 100; // Target direction of new coordinates for drone reposition in degrees
const double REPOSITION_DISTANCE = 3.00; // Target distance of new coordinates for drone reposition in meters
const int WAYPOINT_NAV_SPEED = 60; //RTL and Autonomous speed in centimeters/second
/*///////////////////////////////////////////////////////////
// KALMAN VARIABLES
///////////////////////////////////////////////////////////*/
float previousErrorInEstimate = 2.00; // Default error variance in the estimated distance between drone and obstacle, updated by calculate_kalman_error_in_estimate()
float errorInMeasurement = 4.00; // Default error variance in each distance measurement to calculate to kalman gain adjustments to measurements
/*///////////////////////////////////////////////////////////
// MAIN LOOP
///////////////////////////////////////////////////////////*/
void setup() {
	Serial1.begin(57600); //Initialise serial connection to Pixhawk (Pins 18,19 on Arduino MEGA)
	Serial.begin(57600); //USB serial connection for debugging
}

void loop() {
	/*** STAGE 0 ***/
	/*
	*  Set required arducopter parameters for experiment once, then read mavstream until mode changed to RTL
	*/
	if (setParams == false) {
		set_parameters();
	}
	receive_mavstream();
	/*** STAGE 1 ***/
	/* 
	*  If in RTL or STAGE3 is completed, loop in STAGE1 and continue scanning for an obstacle. 
	*  If an obstacle is detected, set brake mode with a delay to let brake mode counter any inertia, reset global variables and obstacle flags for later obstacles and avoidances, and move to STAGE2.
	*/
	if (currentMode == 6 || currentStage == STAGE1) {
		currentStage = STAGE1;
		while (currentStage == STAGE1) {
			int dist = quick_obstacle_scan_and_confirm();
			if (obstacleFlag == true) {
				//kalman_obstacle_confirm(dist, "ST1_KAL");		// Uncomment if additional kalman-filtered distance measurements are wanted before avoidance.
				//if (kalmanObstacleConfirmed == true) {
					set_brake_mode();
					send_distance_int_command(dist, "ST1_EST");
					delay(2000);
					reset_global_variables();
					reset_obstacle_flags();
					currentStage = STAGE2;
				//} 
			}
		}
	}
	/*** STAGE 2 ***/
	/*
	*  Set high frequency gps messages to quickly obtain the latest GLOBAL_POSITION_INT information i.e. the position and bearing of the drone.
	*  While waiting for the variable information, read the mavstream for the GLOBAL_POSITION_INT information only.
	*  If current longitude, latitude and bearing of the drone is known, GPS messages are set back to low frequency and the reposition coordinates are calculated.
	*  If the target latitude and longitude for reposition is known, a reposition mission is sent to the flight controller.
	*  Auto mode is engaged, followed by the start mission command, to begin the reposition mission.
	*  A two second delay is in place before the sonar sensor continues scanning for obstacles - this prevents the same obstacle being detected before the mission is engaged.
	*  Move to STAGE3.
	*/
	while (currentStage == STAGE2) {
		set_high_frequency_gps_messages();
		while (latCurrent == NULL && longCurrent == NULL && headingCurrent == NULL && currentCoordinatesReceived == false) {
			receive_mavstream_for_current_coordinates_only();

			if (latCurrent != NULL && longCurrent != NULL && headingCurrent != NULL && currentCoordinatesReceived == true) {
				set_low_frequency_gps_messages();
				calculate_coordinates(latCurrent, longCurrent, headingCurrent, NEW_COOR_BEARING);

				if (latTargetFloat != NULL && longTargetFloat != NULL) {
					reposition_mission(latTargetFloat, longTargetFloat);
					set_auto_mode();
					start_mission_command(0, 2);
					delay(2000);
					currentStage = STAGE3;
				}
			}
		}
	}
	/*** STAGE 3 ***/
	/*
	*  While completing the reposition mission, loop in STAGE3 and receive mavstream to keep track of reposition mission progress.
	*  If an obstacle is detected, set brake mode with a delay to let brake mode counter any inertia, reset global variables and obstacle flags for later obstacles and avoidances, and move back to STAGE2 for another reposition.
	*  If no obstacle is detected by the time the reposition mission is complete, global variables are reset, RTL mode is set, move back to loop in STAGE1 to continue checking for obstacles.
	*/
	while (currentStage == STAGE3) {
		int dist = quick_obstacle_scan_and_confirm();
		if (obstacleFlag == true) {
			send_distance_int_command(dist, "ST3_EST");
			//kalman_obstacle_confirm(dist, "ST3_KAL");		// Uncomment if additional kalman-filtered distance measurements are wanted before avoidance.
			//if (kalmanObstacleConfirmed == true) { 
				set_brake_mode();
				delay(2000);
				reset_global_variables();
				reset_obstacle_flags();
				currentStage = STAGE2;
			//} //
		}
		receive_mavstream();
		if (currentMissionSequenceNum == 2) {
			reset_global_variables();
			set_rtl_mode();
			currentStage = STAGE1;
		}
	}
}
/*///////////////////////////////////////////////////////////
// FLAG RESET FUNCTIONS
///////////////////////////////////////////////////////////*/
/*
* Resets navigation global variables to NULL and currentCoordinatesReceived flag to false
*/
void reset_global_variables() {
	latCurrent = NULL;
	longCurrent = NULL;
	headingCurrent = NULL;
	latTargetFloat = NULL;
	longTargetFloat = NULL;
	currentMissionSequenceNum = NULL;
	currentCoordinatesReceived = false;
}
/*
* Resets obstacle warning flag and kalman filter obstacle confirmation flags to false
*/
void reset_obstacle_flags() {
	obstacleFlag = false;
	//kalmanObstacleConfirmed = false;
}
/*///////////////////////////////////////////////////////////
// OBSTACLE DETECTION FUNCTIONS
///////////////////////////////////////////////////////////*/
/*
* Gets mode-filtered distance measurement. If measurement is less than OBSTACLE_DETECT_DIST_INT, takes a second mode-filtered distance measurement. 
* If both measurements are beneath OBSTACLE_DETECT_DIST_INT, drone goes to brake mode and obstacleFlag is set to true.
* If neither measurement is beneath OBSTACLE_DETECT_DIST_INT, obstacleFlag is set to false.
* @return - if less than OBSTACLE_DETECT_DIST_INT, the last mode-filtered sonar distance measurement, else 0 is returned.
*/
int quick_obstacle_scan_and_confirm() {
	int distance = ping_mode_distance_int();
	if (distance < OBSTACLE_DETECT_DIST_INT && distance > 0) {
		int distance2 = ping_mode_distance_int();
		if (distance2 < OBSTACLE_DETECT_DIST_INT && distance2 > 0) {
			set_brake_mode();
			obstacleFlag = true;
			return distance2;
		}
		else { obstacleFlag = false; return 0; }
	}
	else { obstacleFlag = false; return 0; }
}
/*
* Gets kalman-filtered distance measurement. If measurement is less than OBSTACLE_DETECT_DIST_FLOAT, takes a second kalman-filtered distance measurement. 
* If both measurements are beneath OBSTACLE_DETECT_DIST_FLOAT, drone kalmanObstacleConfirmed is set to true, else klamanObstacleConfirmed is set to false and drone goes back to RTL mode.
* @param estimate - the estimated distance between the sonar sensor and obstacle to be fed into the kalman_filter_scan().
* @param message - a message to be sent alongside a kalman-filtered distance measurement.
*/
void kalman_obstacle_confirm(int estimate, char * message) {

	float kalmanConfirmObstacle = kalman_filter_scan((float)0.09, estimate);  //takes apx 3.3 sec per kalman filter


	if (kalmanConfirmObstacle < OBSTACLE_DETECT_DIST_FLOAT) {
		float kalmanConfirmObstacle2 = kalman_filter_scan((float)0.09, estimate);
		if (kalmanConfirmObstacle2 < OBSTACLE_DETECT_DIST_FLOAT) {
			send_distance_float_command(kalmanConfirmObstacle, message);
			send_distance_float_command(kalmanConfirmObstacle2, "true");
			kalmanObstacleConfirmed = true;
		}
		else {
			send_distance_float_command(kalmanConfirmObstacle, message);
			send_distance_float_command(kalmanConfirmObstacle2, "false");
			kalmanObstacleConfirmed = false;
			set_rtl_mode();
		}
	}
	else {
		send_distance_float_command(kalmanConfirmObstacle, message);
		kalmanObstacleConfirmed = false;
		set_rtl_mode();
	}
}
/*///////////////////////////////////////////////////////////
// KALMAN FILTER FUNCTIONS
///////////////////////////////////////////////////////////*/
/*
* Calculates the kalman gain value.
* @return - the kalman gain.
*/
float calculate_kalman_gain() {
	return previousErrorInEstimate / (previousErrorInEstimate + errorInMeasurement);
}
/*
* Calculates a new estimated distance between the sonar sensor and obstacle.
* @param previousEstimate - the estimated distance in the previous kalman filter iteration.
* @param kalmanGain - the most recently calculated kalman gain adjustment
* @param measurement - the new distance measurement between the sonar sensor and obstacle for the current kalman iteration.
* @return - the new estimated distance between the sonar sensor and obstacle.
*/
float calculate_kalman_estimate(float previousEstimate, float kalmanGain, long measurement) {
	return previousEstimate + kalmanGain*(measurement - previousEstimate);
}
/*
* Calculates the new estimate error in kalman estimate.
* @param kalmanGain - the kalmanGain calculated for the current kalman filter iteration.
* @return - the new calculated error in the estimated distance between the sonar sensor and obstacle.
*/
float calculate_kalman_error_in_estimate(float kalmanGain) {
	float errorInEstimate = (1 - kalmanGain)*previousErrorInEstimate;
	previousErrorInEstimate = errorInEstimate;
	return errorInEstimate;
}
/*
* Calculates a kalman-filtered distance between the sonar sensor and obstacle.
* @param kalmanGainTarget - the level of kalmanGain precision needed to be achieved before returning the kalman-filter result. Lower numbers means more kalman filter iterations will run, meaning the function will take longer, and vice versa.
* @param previousEstimate - the raw estimated distance fed to the kalman_filter_scan function.
* @return - the kalman-filtered distance measurement between the sonar sensor and the obstacle.
*/
float kalman_filter_scan(float kalmanGainTarget, float previousEstimate) {
	float kalmanGain = calculate_kalman_gain();
	if (kalmanGain > kalmanGainTarget) {
		while (kalmanGain > kalmanGainTarget) {
			float measurement = ping_mode_distance_float();
			kalmanGain = calculate_kalman_gain();
			float estimate = calculate_kalman_estimate(previousEstimate, kalmanGain, measurement);
			if (kalmanGain < kalmanGainTarget) {
				previousErrorInEstimate = 2.00;
				errorInMeasurement = 4.00;
				return estimate;
			}
			Serial.println("Estimate is:");
			Serial.println(estimate);
			previousEstimate = estimate;
			calculate_kalman_error_in_estimate(kalmanGain);
		}
	}
}
/*///////////////////////////////////////////////////////////
// SONAR PING FUNCTIONS
///////////////////////////////////////////////////////////*/
/*
* Returns the mode-averaged distance between the sonar sensor and obstacle in centimeters as an integer.
* @return - the mode-average distance between the sonar sensor and obstacle.
*/
int ping_mode_distance_int() {
	pinMode(PW_PIN, INPUT);
	int rangevalue[ARRAY_SIZE] = { 0 };
	for (int i = 0; i < ARRAY_SIZE; i++)
	{
		pulse = pulseIn(PW_PIN, HIGH);
		rangevalue[i] = pulse / 58;
		delay(10);

	}
	sort_array_int(rangevalue, ARRAY_SIZE);
	int modeResult = get_mode_int(rangevalue, ARRAY_SIZE);
	return modeResult;
}
/*
* Returns the mode-averaged distance between the sonar sensor and obstacle in centimeters as a float.
* @return - the mode-average distance between the sonar sensor and obstacle.
*/
float ping_mode_distance_float() {
	pinMode(PW_PIN, INPUT);
	float rangevalue[ARRAY_SIZE] = { (float)0 };
	int count = 0;
	for (int i = 0; i < ARRAY_SIZE; i++)
	{
		pulse = pulseIn(PW_PIN, HIGH);
		rangevalue[i] = pulse / (float)58;
		delay(10);

	}
	sort_array_float(rangevalue, ARRAY_SIZE);
	float modeResult = get_mode_float(rangevalue, ARRAY_SIZE);
	return modeResult;
}
/*///////////////////////////////////////////////////////////
// CALCULATE MODE OR MEDIAN FUNCTIONS
///////////////////////////////////////////////////////////*/
/*
* Sorts the stored integers (i.e. distance mesaurements to obstacle) in an array from the smallest to the largest.
* @param *a - a pointer to the array to be sorted.
* @param n - the size of the array to be sorted.
*/
void sort_array_int(int *a, int n) {
	for (int i = 1; i < n; ++i)
	{
		int j = a[i];
		int k;
		for (k = i - 1; (k >= 0) && (j < a[k]); k--)
		{
			a[k + 1] = a[k];
		}
		a[k + 1] = j;
	}
}
/*
* Sorts the stored floats (i.e. distance mesaurements to obstacle) in an array from the smallest to the largest.
* @param *a - a pointer to the array to be sorted.
* @param n - the size of the array to be sorted.
*/
void sort_array_float(float *a, int n) {
	for (int i = 1; i < n; ++i)
	{
		float j = a[i];
		int k;
		for (k = i - 1; (k >= 0) && (j < a[k]); k--)
		{
			a[k + 1] = a[k];
		}
		a[k + 1] = j;
	}
}
/*
* Returns the mode (most frequent number) in a sorted array of integers. If there is no mode, or two or more modes, the median (the middle number of a sorted array) will be returned. Array must be an odd numbered size.
* @param *a - a pointer to the array of integer distances.
* @param n - the size of the array to be averaged.
( @return -  the mode integer distance, or median if there is no mode.
*/
int get_mode_int(int *x, int n) {
	int i = 0;
	int count = 0;
	int maxCount = 0;
	int mode = 0;
	int bimodal;
	int prevCount = 0;

	while (i < (n - 1)) {
		prevCount = count;
		count = 0;
		while (x[i] == x[i + 1]) {
			count++;
			i++;
		}
		if (count > prevCount && count > maxCount) {
			mode = x[i];
			maxCount = count;
			bimodal = 0;
		}
		if (count == 0) {
			i++;
		}
		if (count == maxCount) { 
			bimodal = 1;
		}
		if (mode == 0 || bimodal == 1) {
			mode = x[(n / 2)];
		}
		return mode;
	}
}
/*
* Returns the mode (most frequent number) in a sorted array of floats. If there is no mode, or two or more modes, the median (the middle number of a sorted array) will be returned. Array must be an odd numbered size.
* @param *a - a pointer to the array of float distances.
* @param n - the size of the array to be averaged.
( @return -  the mode float distance, or median if there is no mode.
*/
float get_mode_float(float *x, int n) {
	int i = 0;
	int count = 0;
	int maxCount = 0;
	float mode = (float)0;
	int bimodal;
	int prevCount = 0;

	while (i < (n - 1)) {
		prevCount = count;
		count = 0;
		while (x[i] == x[i + 1]) {
			count++;
			i++;
		}
		if (count > prevCount && count > maxCount) {
			mode = x[i];
			maxCount = count;
			bimodal = 0;
		}
		if (count == 0) {
			i++;
		}
		if (count == maxCount) { 
			bimodal = 1;
		}
		if (mode == 0 || bimodal == 1) { 
			mode = x[(n / 2)];
		}
		return mode;
	}
}
/*///////////////////////////////////////////////////////////
// SET FLIGHT MODE FUNCTIONS
///////////////////////////////////////////////////////////*/
/*
* Sets the drone to brake mode.
*/
void set_brake_mode()
{
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	mavlink_msg_set_mode_pack(SYSTEM_ID, COMPONENT_ID, &msg, 1, 1, 17);								
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	Serial1.write(buf, len);
}
/*
* Sets the drone to return-to-launch mode.
*/
void set_rtl_mode() {
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	mavlink_msg_command_long_pack(SYSTEM_ID, COMPONENT_ID, &msg, 1, 0, MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0);
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	Serial1.write(buf, len);
}
/*
* Sets the drone to auto mode.
*/
void set_auto_mode()
{
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	mavlink_msg_set_mode_pack(SYSTEM_ID, COMPONENT_ID, &msg, 1, 1, 3);	
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	Serial1.write(buf, len);
}
/*///////////////////////////////////////////////////////////
// SET MAVLINK MISSION FUNCTIONS
///////////////////////////////////////////////////////////*/
/*
* Clears all previous mission items stored on flight controller.
*/
void waypoint_clear() {
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	mavlink_msg_mission_clear_all_pack(SYSTEM_ID, COMPONENT_ID, &msg, 1, 0);
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	Serial1.write(buf, len);
}
/*
* Informs flight controller how many mission items to expect prior to sending a list of mission items.
* @param numOfWaypoints - the number of mission items being sent after this function is called, beginning with and including item index 0.
*/
void waypoint_count(uint16_t numOfWaypoints) {
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	mavlink_msg_mission_count_pack(SYSTEM_ID, COMPONENT_ID, &msg, 1, 0, numOfWaypoints);
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	Serial1.write(buf, len);
}
/*
* Sends a waypoint as a mission item to the flight controller. Format is altitude relative to 0, WGS84 coordinate system i.e. MAV_FRAME_GLOBAL_RELATIVE_ALT.
* @param latitude - the mission waypoint latitude in degrees.
* @param longitude - the mission waypoint longitude in degrees.
* @param delaySecs - time delay before executing next mission item in seconds.
* @param altitude - altitude (relative) in meters, 0 to maintain current altitude in arducopter.
* @param sequenceNum - the index number of the mission item within the mission sequence.
* @param current - sets waypoint as the current mission item if 1, not the current mission item if 0.
*/
void set_mission_waypoint_float(float latitude, float longitude, float delaySecs, float altitude, uint16_t sequenceNum, uint8_t current)
{
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	mavlink_msg_mission_item_pack(SYSTEM_ID, COMPONENT_ID, &msg, 1, 0, sequenceNum, MAV_FRAME_GLOBAL_RELATIVE_ALT, 16, current, 1, delaySecs, 0, 0, 0, latitude, longitude, altitude);
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	Serial1.write(buf, len);
}
/*
* Sends a loiter unlimited command as a mission item to the flight controller. Format is altitude relative to 0, WGS84 coordinate system i.e. MAV_FRAME_GLOBAL_RELATIVE_ALT.
* @param latitude - the mission waypoint latitude in degrees.
* @param longitude - the mission waypoint longitude in degrees.
* @param altitude - altitude (relative) in meters, 0 to maintain current altitude in arducopter.
* @param sequenceNum - the index number of the mission item within the mission sequence.
* @param current - sets waypoint as the current mission item if 1, not the current mission item if 0.
*/
void set_mission_loiter_unlimited_float(float latitude, float longitude, float altitude, uint16_t sequenceNum, uint8_t current) {
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	mavlink_msg_mission_item_pack(SYSTEM_ID, COMPONENT_ID, &msg, 1, 0, sequenceNum, MAV_FRAME_GLOBAL_RELATIVE_ALT, 17, current, 1, 0, 0, 0, 0, latitude, longitude, altitude);
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	Serial1.write(buf, len);
}
/*
* Sends a full mission as per the mavlink waypoint protocol (http://qgroundcontrol.org/mavlink/waypoint_protocol). The flight controller's last mission is cleared, the flight controller is told
* to expect three waypoints, the target waypoint is sent as mission items 0 and 1 (arducopter begins at item 1, but the waypoint protocol requires an item 0), and told to loiter at that position until
* commanded to do otherwise.
* @param latitude - the mission waypoint latitude in degrees.
* @param longitude - the mission waypoint longitude in degrees.
*/
void reposition_mission(float latitude, float longitude) {
	waypoint_clear();
	waypoint_count(3);
	set_mission_waypoint_float(latitude, longitude, 0, 0, 0, 0);
	set_mission_waypoint_float(latitude, longitude, 0, 0, 1, 0);
	set_mission_loiter_unlimited_float(latitude, longitude, 0, 2, 0);
}
/*
* Commands flight controller to start a mission in auto mode without having to raise the throttle on radio controller. 
* @param startSeq - the mission item sequence number to begin the mission.
* @param endSeq - the mission item sequence number to finish the mission.
*/
void start_mission_command(float startSeq, float endSeq) {
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	mavlink_msg_command_long_pack(SYSTEM_ID, COMPONENT_ID, &msg, 1, 0, 300, 0, startSeq, endSeq, 0, 0, 0, 0, 0);
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	Serial1.write(buf, len);
}
/*///////////////////////////////////////////////////////////
// LOG OBSTACLE DISTANCE FUNCTIONS
///////////////////////////////////////////////////////////*/
/*
* Sends a float value (in this case a distance measurement) alongside a message to the flight controller.
* @param distanceReading - the distance measurement to be sent.
* @param name - a pointer to a character array (maximum 10 characters) containing a message.
*/
void send_distance_float_command(float distanceReading, char * name) {
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	mavlink_msg_named_value_float_pack(1, 0, &msg, millis(), name, distanceReading);
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	Serial1.write(buf, len);
}
/*
* Sends an integer value (in this case a distance measurement) alongside a message to the flight controller.
* @param distanceReading - the distance measurement to be sent.
* @param name - a pointer to a character array (maximum 10 characters) containing a message.
*/
void send_distance_int_command(int distanceReading, char * name) {
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;
	mavlink_msg_named_value_int_pack(1, 0, &msg, millis(), name, distanceReading);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	Serial1.write(buf, len);
}
/*///////////////////////////////////////////////////////////
// CALCULATE REPOSITION COORDINATES FUNCTIONS
///////////////////////////////////////////////////////////*/
/*
* Calculates new WGS84 latitude and longitude given a current latitude, longitude, current bearing, target bearing and target distance (REPOSITION_DISTANCE constant global variable).
* latTargetFloat and longTargetFloat are set to the calculated coordinates. The function is capable of calculating back to 1e7 degrees in int32_t format too.
* @param currentLatitude - latitude coordinate read from mavstream GLOBAL_POSITION_INT message in degrees 1e7.
* @param currentLongitude - longitude coordinate read from mavstream GLOBAL_POSITION_INT message in degrees 1e7.
* @param currentBearing - the direction of the drone in degrees 1e2.
* @param newCoordBearing - the direction in which to calculate the reposition coordinates (NEW_COOR_BEARING constant global variable).
*/
void calculate_coordinates(int32_t currentLatitude, int32_t currentLongitude, uint16_t currentBearing, float newCoordBearing) {
	float currentLatitudeFloat = currentLatitude / pow(10, 7); //note - need to conver to * pow(10,7) for 1E7 degrees
	float currentLongitudeFloat = currentLongitude / pow(10, 7);
	float currentBearingFloat = currentBearing / pow(10, 2);
	currentBearingFloat = currentBearingFloat + newCoordBearing;
	if (currentBearingFloat >= (float)360) {
		currentBearingFloat = currentBearingFloat - (float)360;
	}
	double earthRadius = 6371e3;
	float latitudeRadians = convert_degrees_to_radians(currentLatitudeFloat);
	float longitudeRadians = convert_degrees_to_radians(currentLongitudeFloat);
	float BearingRadians = convert_degrees_to_radians(currentBearingFloat);
	float angularDistanceRadians = (REPOSITION_DISTANCE / earthRadius);
	float newLatitude = asin(sin(latitudeRadians) * cos(angularDistanceRadians) + cos(latitudeRadians) * sin(angularDistanceRadians) * cos(BearingRadians));
	float newLongitude = longitudeRadians + atan2(sin(BearingRadians) * sin(angularDistanceRadians)*cos(latitudeRadians), cos(angularDistanceRadians) - sin(latitudeRadians)*sin(newLatitude));
	latTargetFloat = convert_radians_to_degrees(newLatitude);
	longTargetFloat = convert_radians_to_degrees(newLongitude);
	float newBearing = convert_radians_to_degrees(BearingRadians);

	//Serial.print("New Latitude is: ");
	//Serial.println(newLatitudeDegrees, 8);
	//Serial.print("New Longitude is: ");
	//Serial.println(newLongitudeDegrees, 8);
	//Serial.print("Bearing is: ");
	//Serial.println(newBearing);
	//latTarget = latTargetFloat * pow(10, 7);			// Uncomment if int32_t coordinates wanted
	//longTarget = longTargetFloat * pow(10, 7);
	//Serial.print("latTarget is: ");
	//Serial.println(latTarget);
	//Serial.print("longTarget is: ");
	//Serial.println(longTarget);
}
/*
* Converts decimal degrees to radians.
* @param deg - the value to be converted to radians.
* @return - the converted radians value.
*/
float convert_degrees_to_radians(float deg) {
	return (deg * M_PI / 180);
}
/*
* Converts radians to decimal degrees.
* @param rad - the value to be converted to decimal degrees.
* @return - the converted decimal degrees value.
*/
float convert_radians_to_degrees(float rad) {
	return (rad * 180 / M_PI);
}
/*///////////////////////////////////////////////////////////
// RECEIVE AND HANDLE MAVSTREAM FUNCTIONS
///////////////////////////////////////////////////////////*/
/*
* Reads the serial port receiving a mavstream from the flight controller and parses the bytes into a mavlink message struct.
* The mavlink message is then handled.
*/
void receive_mavstream(){
	while (Serial1.available() > 0) {
		mavlink_message_t msg;
		mavlink_status_t status;
		uint8_t rec = Serial1.read();//Show bytes send from the pixhawk
									 // Serial.print("len = ");
									 //   Serial.print(" Data = ");
									 //   Serial.println(rec);
		if (mavlink_parse_char(MAVLINK_COMM_0, rec, &msg, &status)) {
			//Serial.println(" ");
			//Serial.print("Message: ");
			//Serial.println(msg.msgid);
			handle_message(&msg);
		}
	}
}
/*
* Handles and decodes mavlink messages based on the message id (http://mavlink.org/messages/common).
* @param msg - a pointer to the mavlink message to be handled and decoded.
*/
void handle_message(mavlink_message_t * msg) {
	switch (msg->msgid) {
		if (received_sysid == NULL) {
			case MAVLINK_MSG_ID_MISSION_CURRENT:
			{
				mavlink_mission_current_t packet;
				mavlink_msg_mission_current_decode(msg, &packet);
				currentMissionSequenceNum = packet.seq; 
				Serial.println("Current Mission Sequence: ");
				Serial.println(currentMissionSequenceNum);
				break;
			}
			case MAVLINK_MSG_ID_HEARTBEAT:
			{
				mavlink_heartbeat_t packet;
				mavlink_msg_heartbeat_decode(msg, &packet);
				if ((*msg).sysid != 0xff) { // do not process mission planner heartbeats if we have two receiver xbees
					received_sysid = (*msg).sysid; // save the sysid and compid of the received heartbeat for use in sending new messages
					received_compid = (*msg).compid;
					currentMode = packet.custom_mode;
					Serial.println("heartbeat");
					Serial.println("Mode is: ");
					Serial.println(currentMode);
				}
				break;
			}
		}
	}
}
/*
* Reads the serial port receiving a mavstream from the flight controller and parses the bytes into a mavlink message struct.
* Only a message containing a GLOBAL_POSITION_INT is decoded.
*/
void receive_mavstream_for_current_coordinates_only(){
	while (Serial1.available() > 0) {
		mavlink_message_t msg;
		mavlink_status_t status;
		uint8_t rec = Serial1.read();//Show bytes send from the pixhawk
									 // Serial.print("len = ");
									 //   Serial.print(" Data = ");
									 //   Serial.println(rec);
		if (mavlink_parse_char(MAVLINK_COMM_0, rec, &msg, &status)) {
			//Serial.println(" ");
			//Serial.print("Message: ");
			//Serial.println(msg.msgid);
			handle_message_for_current_coordinates_only(&msg);
		}
	}
}
/*
* Handles and decodes mavlink messages based on the message id, only decodes a GLOBAL_POSITION_INT message.
* the latCurrent, longCurrent and headingCurrent global variables are set and the currentCoordinatesReceived flag is set to true.
* @param msg - a pointer to the mavlink message to be handled and decoded.
*/
void handle_message_for_current_coordinates_only(mavlink_message_t * msg){
	switch (msg->msgid) {
		if (received_sysid == NULL) {
			case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
			{ 
				mavlink_global_position_int_t gps_packet;
				mavlink_msg_global_position_int_decode(msg, &gps_packet);
				latCurrent = gps_packet.lat;//513558772;
				longCurrent = gps_packet.lon;//-29028808;
				headingCurrent = gps_packet.hdg;//30135
				currentCoordinatesReceived = true;
				break;
			}
		}
	}
}
/*///////////////////////////////////////////////////////////
// SET ARDUPILOT PARAMETER FUNCTIONS
///////////////////////////////////////////////////////////*/
/*
* Increases the frequency of GLOBAL_POSITION_INT mavlink message, among other mavlink messages, by setting the SR1_POSITION parameter stream rate to 10Hz (http://ardupilot.org/copter/docs/parameters.html#sr1-parameters).
*/
void set_high_frequency_gps_messages() {
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	mavlink_msg_param_set_pack(SYSTEM_ID, COMPONENT_ID, &msg, 1, 0, "SR1_POSITION", 10, MAV_PARAM_TYPE_UINT8);
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	Serial1.write(buf, len);
}
/*
* Reduces the frequency of GLOBAL_POSITION_INT mavlink message, among other mavlink messages, by setting the SR1_POSITION parameter stream rate to 1Hz
*/
void set_low_frequency_gps_messages() {
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	mavlink_msg_param_set_pack(SYSTEM_ID, COMPONENT_ID, &msg, 1, 0, "SR1_POSITION", 1, MAV_PARAM_TYPE_UINT8);
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	Serial1.write(buf, len);
}
/*
* Increases the frequency of CURRENT_WAYPOINT mavlink message, among other mavlink messages, by setting the SR1_EXT_STAT parameter stream rate to 10Hz
*/
void set_high_frequency_current_waypoint_message() { 
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	mavlink_msg_param_set_pack(SYSTEM_ID, COMPONENT_ID, &msg, 1, 0, "SR1_EXT_STAT", 10, MAV_PARAM_TYPE_UINT8);
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	Serial1.write(buf, len);
}
/*
* Sets WP_YAW_BEHAVIOUR parameter to always face the next waypoint
*/
void set_waypoint_behaviour_to_face_waypoint() { 
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	mavlink_msg_param_set_pack(SYSTEM_ID, COMPONENT_ID, &msg, 1, 0, "WP_YAW_BEHAVIOR", 1, MAV_PARAM_TYPE_UINT8);
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	Serial1.write(buf, len);
}
/*
* Sets WPNAV_SPEED parameter to a target speed in centimeters/s (i.e. WAYPOINT_NAV_SPEED const global variable).
*/
void set_rtl_and_autonomous_speed() { 
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	mavlink_msg_param_set_pack(SYSTEM_ID, COMPONENT_ID, &msg, 1, 0, "WPNAV_SPEED", WAYPOINT_NAV_SPEED, MAV_PARAM_TYPE_UINT8);
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	Serial1.write(buf, len);
}
/*
* Sets WPNAV_RADIUS parameter to a target acceptable radius in centimeters before arducopter acknowledges it has reached a waypoint (i.e. 50cm).
*/
void set_waypoint_radius() { 
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	mavlink_msg_param_set_pack(SYSTEM_ID, COMPONENT_ID, &msg, 1, 0, "WPNAV_RADIUS", 50, MAV_PARAM_TYPE_UINT8);
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	Serial1.write(buf, len);
}
/*
* Sets high frequency waypoint publishing parameter, waypoint behavious parameter, rtl and autonomous speed parameter, and acceptable waypoint radius parameter. 
* When these initial parameters have been set, setParams flag is set to true.
*/
void set_parameters() {	
	set_high_frequency_current_waypoint_message();
	set_waypoint_behaviour_to_face_waypoint();
	set_rtl_and_autonomous_speed();
	set_waypoint_radius();
	setParams = true;
}




