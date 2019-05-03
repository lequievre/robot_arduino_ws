/*
 * Laurent LEQUIEVRE
 * Research Engineer, CNRS (France)
 * laurent.lequievre@uca.fr
 * UMR 6602 - Institut Pascal
 * 
 * rostopic pub -1 arduino/cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
 * rostopic pub -1 arduino/cmd_pos std_msgs/Float32MultiArray "{data:[0.0, 0.0, 0.0, 0.0]}"
 * rostopic pub -1 arduino/cmd_pos_joint1 std_msgs/Float32 "{data: 0.0}"
 * rostopic pub -1 arduino/cmd_pos_joint2 std_msgs/Float32 "{data: 0.0}"
 * rostopic pub -1 arduino/cmd_pos_joint3 std_msgs/Float32 "{data: 0.0}"
 * rostopic pub -1 arduino/cmd_pos_joint4 std_msgs/Float32 "{data: 0.0}"
 * rostopic echo arduino/joint_states
 * 
 */

#include <DynamixelWorkbench.h>
#include <math.h>
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>

// WHEEL SYNC_READ_HANDLER(Only for Protocol 2.0)
#define WHEEL_SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0

// WHEEL SYNC_WRITE_HANDLER
#define WHEEL_SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY 0

// ARM SYNC_READ_HANDLER(Only for Protocol 2.0)
#define ARM_SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 1

// ARM SYNC_WRITE_HANDLER
#define ARM_SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 1

// Protocol 2.0
#define ADDR_PRESENT_CURRENT_2  126
#define ADDR_PRESENT_VELOCITY_2 128
#define ADDR_PRESENT_POSITION_2 132
#define ADDR_GOAL_VELOCITY_2    104
#define ADDR_GOAL_POSITION_2    116

#define LENGTH_PRESENT_CURRENT_2  2
#define LENGTH_PRESENT_VELOCITY_2 4
#define LENGTH_PRESENT_POSITION_2 4
#define LENGTH_GOAL_VELOCITY_2    4
#define LENGTH_GOAL_POSITION_2    4

#if defined(__OPENCM904__)
  #define DEVICE_NAME "3" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#elif defined(__OPENCR__)
  #define DEVICE_NAME ""
#endif  

#define BAUDRATE  1000000
#define DXL_ID_WHEEL_RIGHT   2
#define DXL_ID_WHEEL_LEFT    3
#define NB_JOINT_WHEEL       2

#define DXL_ID_ARM_JOINT1    4
#define DXL_ID_ARM_JOINT2    5
#define DXL_ID_ARM_JOINT3    6
#define DXL_ID_ARM_JOINT4    7
#define NB_JOINT_ARM         4

#define CURRENT_UNIT          3.36f   //  3.36[mA]
#define RPM_MX_64_2           0.229   // 0.229 rpm
#define WHEEL_RADIUS          0.047  // 4.7 cm
#define WHEEL_SEPARATION      0.21   // 21 cm

DynamixelWorkbench dxl_wb;

//ros::NodeHandle  nh;
ros::NodeHandle_<ArduinoHardware, 25, 25, 2048, 2048> nh;

//sensor_msgs::JointState wheel_joint_states_msg, arm_joint_states_msg;
//ros::Publisher wheel_joint_states_pub("joint_states_wheel", &wheel_joint_states_msg);
//ros::Publisher arm_joint_states_pub("joint_states_arm", &arm_joint_states_msg);

sensor_msgs::JointState joint_states_msg;
ros::Publisher joint_states_pub("arduino/joint_states", &joint_states_msg);

std::map<std::string, uint8_t> map_id_wheel_dynamixels;
std::map<std::string, uint8_t> map_id_arm_dynamixels;

uint8_t wheel_id_array[2] = {DXL_ID_WHEEL_RIGHT, DXL_ID_WHEEL_LEFT};
uint8_t arm_id_array[4] = {DXL_ID_ARM_JOINT1, DXL_ID_ARM_JOINT2, DXL_ID_ARM_JOINT3, DXL_ID_ARM_JOINT4};

// Arrays for Wheel
int32_t wheel_raw_position[2] = {0, 0};
//float wheel_position[2] = { 0.0, 0.0 };
int32_t wheel_raw_current[2] = {0, 0};
//float wheel_current[2] = { 0.0, 0.0 };
int32_t wheel_raw_velocity[2] = {0, 0};
//float wheel_velocity[2] = { 0.0, 0.0 };
//char *wheel_names[2] = {"wheel_right", "wheel_left"};

// Arrays  for Arm
int32_t arm_raw_position[4] = {0, 0, 0, 0};
//float arm_position[4] = { 0.0, 0.0, 0.0, 0.0 };
int32_t arm_raw_current[4] = {0, 0, 0, 0};
//float arm_current[4] = { 0.0, 0.0, 0.0, 0.0 };
int32_t arm_raw_velocity[4] = {0, 0, 0, 0};
//float arm_velocity[4] = { 0.0, 0.0, 0.0, 0.0 };
//char *arm_names[4] = {"joint_1", "joint_2", "joint_3", "joint_4" };

float joint_position[NB_JOINT_WHEEL+NB_JOINT_ARM] = { 0.0 };
float joint_current[NB_JOINT_WHEEL+NB_JOINT_ARM] = { 0.0 };
float joint_velocity[NB_JOINT_WHEEL+NB_JOINT_ARM] = { 0.0 };
char *joint_names[NB_JOINT_WHEEL+NB_JOINT_ARM] = { "wheel_right", "wheel_left", "joint1", "joint2", "joint3", "joint4" };

bool initWheelSyncWrite(void)
{
  bool result = false;
  const char* log;
  
  result = dxl_wb.addSyncWriteHandler(ADDR_GOAL_VELOCITY_2, LENGTH_GOAL_VELOCITY_2, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("WHEEL : Failed to add sync write handler goal velocity");
    return false;
  }
  else
  {
    Serial.println("WHEEL : Succeeded to add sync write handler goal velocity");
  }

  return true;
}

bool initArmSyncWrite(void)
{
  bool result = false;
  const char* log;
  
  result = dxl_wb.addSyncWriteHandler(ADDR_GOAL_POSITION_2, LENGTH_GOAL_POSITION_2, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("ARM : Failed to add sync write handler goal position");
    return false;
  }
  else
  {
    Serial.println("ARM : Succeeded to add sync write handler goal position");
  }
  
  return true;
}


bool initWheelSyncRead(void)
{
  bool result = false;
  const char* log;

  result = dxl_wb.addSyncReadHandler(ADDR_PRESENT_CURRENT_2,
                                          (LENGTH_PRESENT_CURRENT_2 + LENGTH_PRESENT_VELOCITY_2 + LENGTH_PRESENT_POSITION_2),
                                          &log);

  if (result == false)
  {
    Serial.println(log);
    Serial.println("WHEEL : Failed to add sync read handler present position + present velocity + present current");
    return false;
  }
  else
  {
    Serial.println("WHEEL : Succeeded to add sync read handler present position + present velocity + present current");
  }

  return true;                         
}

bool initArmSyncRead(void)
{
  bool result = false;
  const char* log;

  result = dxl_wb.addSyncReadHandler(ADDR_PRESENT_CURRENT_2,
                                          (LENGTH_PRESENT_CURRENT_2 + LENGTH_PRESENT_VELOCITY_2 + LENGTH_PRESENT_POSITION_2),
                                          &log);

  if (result == false)
  {
    Serial.println(log);
    Serial.println("ARM : Failed to add sync read handler present position + present velocity + present current");
    return false;
  }
  else
  {
    Serial.println("ARM : Succeeded to add sync read handler present position + present velocity + present current");
  }

  return true;                         
}

bool readWheelSyncDatas()
{
    bool result = false;
    const char* log;
    
    result = dxl_wb.syncRead(WHEEL_SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                               wheel_id_array,
                               NB_JOINT_WHEEL,
                               &log);

   result = dxl_wb.getSyncReadData(WHEEL_SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                          wheel_id_array,
                          NB_JOINT_WHEEL,
                          ADDR_PRESENT_CURRENT_2,
                          LENGTH_PRESENT_CURRENT_2,
                          wheel_raw_current,
                          &log);

    //wheel_current[0] = wheel_raw_current[0] * CURRENT_UNIT;
    //wheel_current[1] = wheel_raw_current[1] * CURRENT_UNIT;

    joint_current[0] = wheel_raw_current[0] * CURRENT_UNIT;
    joint_current[1] = wheel_raw_current[1] * CURRENT_UNIT;
         
    result = dxl_wb.getSyncReadData(WHEEL_SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                      wheel_id_array,
                                                      NB_JOINT_WHEEL,
                                                      ADDR_PRESENT_VELOCITY_2,
                                                      LENGTH_PRESENT_VELOCITY_2,
                                                      wheel_raw_velocity,
                                                      &log);


    //wheel_velocity[0] = dxl_wb.convertValue2Velocity(wheel_id_array[0], wheel_raw_velocity[0]);
    //wheel_velocity[1] = dxl_wb.convertValue2Velocity(wheel_id_array[1], wheel_raw_velocity[1]);

    joint_velocity[0] = dxl_wb.convertValue2Velocity(wheel_id_array[0], wheel_raw_velocity[0]);
    joint_velocity[1] = dxl_wb.convertValue2Velocity(wheel_id_array[1], wheel_raw_velocity[1]);
  
    result = dxl_wb.getSyncReadData(WHEEL_SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                    wheel_id_array,
                                                    NB_JOINT_WHEEL,
                                                    ADDR_PRESENT_POSITION_2,
                                                    LENGTH_PRESENT_POSITION_2,
                                                    wheel_raw_position,
                                                    &log);

    //wheel_position[0] = fmod(dxl_wb.convertValue2Radian(wheel_id_array[0], wheel_raw_position[0]),2.0*M_PI); 
    //wheel_position[1] = fmod(dxl_wb.convertValue2Radian(wheel_id_array[1], wheel_raw_position[1]),2.0*M_PI);

    joint_position[0] = fmod(dxl_wb.convertValue2Radian(wheel_id_array[0], wheel_raw_position[0]),2.0*M_PI);
    joint_position[1] = fmod(dxl_wb.convertValue2Radian(wheel_id_array[1], wheel_raw_position[1]),2.0*M_PI);

}


bool readArmSyncDatas()
{
    bool result = false;
    const char* log;
    
    result = dxl_wb.syncRead(ARM_SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                               arm_id_array,
                               NB_JOINT_ARM,
                               &log);

   result = dxl_wb.getSyncReadData(ARM_SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                          arm_id_array,
                          NB_JOINT_ARM,
                          ADDR_PRESENT_CURRENT_2,
                          LENGTH_PRESENT_CURRENT_2,
                          arm_raw_current,
                          &log);

    for (int i=0; i<NB_JOINT_ARM; i++)
    {
      joint_current[NB_JOINT_WHEEL+i] = arm_raw_current[i] * CURRENT_UNIT;
    }
    
    //arm_current[0] = arm_raw_current[0] * CURRENT_UNIT;
    //arm_current[1] = arm_raw_current[1] * CURRENT_UNIT;
    //arm_current[2] = arm_raw_current[2] * CURRENT_UNIT;
    //arm_current[3] = arm_raw_current[3] * CURRENT_UNIT;
         
    result = dxl_wb.getSyncReadData(ARM_SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                      arm_id_array,
                                                      NB_JOINT_ARM,
                                                      ADDR_PRESENT_VELOCITY_2,
                                                      LENGTH_PRESENT_VELOCITY_2,
                                                      arm_raw_velocity,
                                                      &log);


    //arm_velocity[0] = dxl_wb.convertValue2Velocity(arm_id_array[0], arm_raw_velocity[0]);
    //arm_velocity[1] = dxl_wb.convertValue2Velocity(arm_id_array[1], arm_raw_velocity[1]);
    //arm_velocity[2] = dxl_wb.convertValue2Velocity(arm_id_array[2], arm_raw_velocity[2]);
    //arm_velocity[3] = dxl_wb.convertValue2Velocity(arm_id_array[3], arm_raw_velocity[3]);

    for (int i=0; i<NB_JOINT_ARM; i++)
    {
      joint_velocity[NB_JOINT_WHEEL+i] = dxl_wb.convertValue2Velocity(arm_id_array[i], arm_raw_velocity[i]);
    }
    
  
    result = dxl_wb.getSyncReadData(ARM_SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                    arm_id_array,
                                                    NB_JOINT_ARM,
                                                    ADDR_PRESENT_POSITION_2,
                                                    LENGTH_PRESENT_POSITION_2,
                                                    arm_raw_position,
                                                    &log);

    //arm_position[0] = fmod(dxl_wb.convertValue2Radian(arm_id_array[0], arm_raw_position[0]),2.0*M_PI); 
    //arm_position[1] = fmod(dxl_wb.convertValue2Radian(arm_id_array[1], arm_raw_position[1]),2.0*M_PI);
    //arm_position[2] = fmod(dxl_wb.convertValue2Radian(arm_id_array[2], arm_raw_position[2]),2.0*M_PI); 
    //arm_position[3] = fmod(dxl_wb.convertValue2Radian(arm_id_array[3], arm_raw_position[3]),2.0*M_PI);

    for (int i=0; i<NB_JOINT_ARM; i++)
    {
      joint_position[NB_JOINT_WHEEL+i] = fmod(dxl_wb.convertValue2Radian(arm_id_array[i], arm_raw_position[i]),2.0*M_PI);
    }

}

void commandVelocityCallback(const geometry_msgs::Twist &msg)
{

  /*
    RPM = revolution per minute
    
    To convert RPM to rad/s, multiply by 0.10472 (which is an approximation of pi/30)
    RPM * 0.10472 = rad/s
    
    To convert rad/s to RPM, multiply by 9.54929 (which is an approximation of 30/pi)
    rad/s * 9.54929 = RPM
    
    The Angular to Linear Velocity formular is : v = r × ω

    Where:
      v: Linear velocity, in m/s
      r: Radius, in meter
      ω: Angular velocity, in rad/s
    
    The RPM to Linear Velocity formular is : v = r × RPM × 0.10472
    
    Where:
      v: Linear velocity, in m/s
      r: Radius, in meter
      RPM: Angular velocity, in RPM (Rounds per Minute)

    ==> v = r * w = r * (RPM * 0.10472)
    ==> v = r * ((RPM * Goal_Velocity) * 0.10472)    
    ==> Goal_Velocity = v / (r * RPM * 0.10472) = v * VELOCITY_CONSTANT_VALUE
      
   */

  bool result = false;
  const char* log;
  
  double robot_lin_vel = msg.linear.x;
  double robot_ang_vel = msg.angular.z;

  double wheel_velocity[2];
  int32_t dynamixel_velocity[2];
  const uint8_t LEFT = 1;
  const uint8_t RIGHT = 0;

  double velocity_constant_value = 1 / (WHEEL_RADIUS * RPM_MX_64_2 * 0.10472);

  // v = r x w ==> r = WHEEL_SEPARATION / 2, w = robot_ang_vel
  // to go on left side => upper wheel right, down wheel left

  wheel_velocity[RIGHT] = robot_lin_vel + (robot_ang_vel * WHEEL_SEPARATION / 2);
  wheel_velocity[LEFT]  = robot_lin_vel - (robot_ang_vel * WHEEL_SEPARATION / 2);

  dynamixel_velocity[RIGHT] = wheel_velocity[RIGHT] * velocity_constant_value;
  dynamixel_velocity[LEFT]  = wheel_velocity[LEFT] * velocity_constant_value;
  
  result = dxl_wb.syncWrite(WHEEL_SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY, wheel_id_array, NB_JOINT_WHEEL, dynamixel_velocity, 1, &log);

  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to sync write handler goal velocity");
  }
  else
  {
    Serial.println("Succeeded to sync write handler goal velocity");
  }

}

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("arduino/cmd_vel", commandVelocityCallback );


void commandArmPositionCallback(const std_msgs::Float32MultiArray &msg)
{
  bool result = false;
  const char* log;
  
  int32_t dynamixel_position[4];

  for (int i=0; i<NB_JOINT_ARM; i++)
  {
      dynamixel_position[i] = dxl_wb.convertRadian2Value(arm_id_array[i], msg.data[i]);
  }

  result = dxl_wb.syncWrite(ARM_SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, arm_id_array, NB_JOINT_ARM, dynamixel_position, 1, &log);

 /* if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to sync write handler goal position");
  }
  else
  {
    Serial.println("Succeeded to sync write handler goal velocity");
  }*/
  
}

void commandArmPositionJoint1Callback(const std_msgs::Float32 &msg)
{
  int32_t dynamixel_position;

  dynamixel_position = dxl_wb.convertRadian2Value(DXL_ID_ARM_JOINT1, msg.data);

  dxl_wb.goalPosition(DXL_ID_ARM_JOINT1, dynamixel_position);
}

void commandArmPositionJoint2Callback(const std_msgs::Float32 &msg)
{
  int32_t dynamixel_position;

  dynamixel_position = dxl_wb.convertRadian2Value(DXL_ID_ARM_JOINT2, msg.data);
  
  dxl_wb.goalPosition(DXL_ID_ARM_JOINT2, dynamixel_position);
}

void commandArmPositionJoint3Callback(const std_msgs::Float32 &msg)
{
  int32_t dynamixel_position;

  dynamixel_position = dxl_wb.convertRadian2Value(DXL_ID_ARM_JOINT3, msg.data);

  dxl_wb.goalPosition(DXL_ID_ARM_JOINT3, dynamixel_position);
}

void commandArmPositionJoint4Callback(const std_msgs::Float32 &msg)
{
  int32_t dynamixel_position;

  dynamixel_position = dxl_wb.convertRadian2Value(DXL_ID_ARM_JOINT4, msg.data);

  dxl_wb.goalPosition(DXL_ID_ARM_JOINT4, dynamixel_position);
}

ros::Subscriber<std_msgs::Float32MultiArray> cmd_arm_position_sub("arduino/cmd_pos", commandArmPositionCallback );
ros::Subscriber<std_msgs::Float32> cmd_arm_joint1_position_sub("arduino/cmd_pos_joint1", commandArmPositionJoint1Callback);
ros::Subscriber<std_msgs::Float32> cmd_arm_joint2_position_sub("arduino/cmd_pos_joint2", commandArmPositionJoint2Callback);
ros::Subscriber<std_msgs::Float32> cmd_arm_joint3_position_sub("arduino/cmd_pos_joint3", commandArmPositionJoint3Callback);
ros::Subscriber<std_msgs::Float32> cmd_arm_joint4_position_sub("arduino/cmd_pos_joint4", commandArmPositionJoint4Callback);


bool initWorkbench(const std::string port_name, const uint32_t baud_rate)
{
  bool result = false;
  const char* log;

  result = dxl_wb.init(port_name.c_str(), baud_rate, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to init workbench !");
    return false;
  }
  else
  {
    Serial.print("Succeeded to init workbench with a baudrate : ");
    Serial.println(BAUDRATE);
  }

  return true;
}

void getDynamixelsWheelInfo()
{
  map_id_wheel_dynamixels["wheel_right"] = DXL_ID_WHEEL_RIGHT;
  map_id_wheel_dynamixels["wheel_left"] = DXL_ID_WHEEL_LEFT;
}

void getDynamixelsArmInfo()
{
  map_id_arm_dynamixels["arm_joint_1"] = DXL_ID_ARM_JOINT1;
  map_id_arm_dynamixels["arm_joint_2"] = DXL_ID_ARM_JOINT2;
  map_id_arm_dynamixels["arm_joint_3"] = DXL_ID_ARM_JOINT3;
  map_id_arm_dynamixels["arm_joint_4"] = DXL_ID_ARM_JOINT4;
}

bool loadWheelDynamixels(void)
{
  bool result = false;
  const char* log;
  uint16_t model_number = 0;

  std::map<std::string, uint8_t>::iterator it = map_id_wheel_dynamixels.begin();

  while (it != map_id_wheel_dynamixels.end())
  {
    result = dxl_wb.ping((uint8_t)it->second, &model_number, &log);
    if (result == false)
    {
      Serial.println(log);
      Serial.print("Failed to ping ");
      Serial.print((it->first).c_str());
      Serial.println(" !");
      Serial.print("ID : ");
      Serial.println((uint8_t)it->second);
      return false;
    }
    else
    {
      Serial.println("Succeeded to ping : ");
      Serial.print("name : ");
      Serial.println((it->first).c_str());
      Serial.print("id : ");
      Serial.println((uint8_t)it->second);
      Serial.print("model_number : ");
      Serial.println(model_number);
      Serial.print("model name : ");
      Serial.println(dxl_wb.getModelName((uint8_t)it->second));
    }
    it++;
  }
  
  return true;
}

bool loadArmDynamixels(void)
{
  bool result = false;
  const char* log;
  uint16_t model_number = 0;

  std::map<std::string, uint8_t>::iterator it = map_id_arm_dynamixels.begin();

  while (it != map_id_arm_dynamixels.end())
  {
    result = dxl_wb.ping((uint8_t)it->second, &model_number, &log);
    if (result == false)
    {
      Serial.println(log);
      Serial.print("Failed to ping ");
      Serial.print((it->first).c_str());
      Serial.println(" !");
      Serial.print("ID : ");
      Serial.println((uint8_t)it->second);
      return false;
    }
    else
    {
      Serial.println("Succeeded to ping : ");
      Serial.print("name : ");
      Serial.println((it->first).c_str());
      Serial.print("id : ");
      Serial.println((uint8_t)it->second);
      Serial.print("model_number : ");
      Serial.println(model_number);
      Serial.print("model name : ");
      Serial.println(dxl_wb.getModelName((uint8_t)it->second));
    }
    it++;
  }
  
  return true;
}

bool initWheelDynamixels(void)
{
  bool result = false;
  const char* log;
  uint16_t model_number = 0;

  std::map<std::string, uint8_t>::iterator it = map_id_wheel_dynamixels.begin();

  while (it != map_id_wheel_dynamixels.end())
  {
    result = dxl_wb.wheelMode((uint8_t)it->second, 0, &log);
    if (result == false)
    {
      Serial.println(log);
      Serial.print("Failed to set the Wheel Mode ");
      Serial.print((it->first).c_str());
      Serial.println(" !");
      Serial.print("ID : ");
      Serial.println((uint8_t)it->second);
      return false;
    }
    else
    {
      Serial.println("Succeeded  to set the Wheel Mode");
      Serial.print("id : ");
      Serial.println((uint8_t)it->second);
      Serial.print("model_number : ");
      Serial.println(model_number);
      Serial.print("model name : ");
      Serial.println(dxl_wb.getModelName((uint8_t)it->second));
    }
    it++;
  }
  
  return true;
}

bool initArmDynamixels(void)
{
  bool result = false;
  const char* log;
  uint16_t model_number = 0;

  std::map<std::string, uint8_t>::iterator it = map_id_arm_dynamixels.begin();

  while (it != map_id_arm_dynamixels.end())
  {
    result = dxl_wb.jointMode((uint8_t)it->second, 100, 100, &log);
    if (result == false)
    {
      Serial.println(log);
      Serial.print("Failed to set the Joint Mode ");
      Serial.print((it->first).c_str());
      Serial.println(" !");
      Serial.print("ID : ");
      Serial.println((uint8_t)it->second);
      return false;
    }
    else
    {
      Serial.println("Succeeded  to set the Joint Mode");
      Serial.print("id : ");
      Serial.println((uint8_t)it->second);
      Serial.print("model_number : ");
      Serial.println(model_number);
      Serial.print("model name : ");
      Serial.println(dxl_wb.getModelName((uint8_t)it->second));
    }
    it++;
  }
  
  return true;
}

void printWheelsInfos(void)
{
  int32_t velocity_limit = 0;
  int32_t  drive_mode = 0;
  int32_t  firmware_version = 0;
  bool result = false;
  const char* log;

  const ModelInfo* modelInfoWheelRight =  dxl_wb.getModelInfo(DXL_ID_WHEEL_RIGHT, &log);
  Serial.println("Wheel Right Infos :");
  Serial.print("rpm : "); Serial.println(modelInfoWheelRight->rpm);
  Serial.print("value_of_min_radian_position : "); Serial.println((int32_t)modelInfoWheelRight->value_of_min_radian_position);
  Serial.print("value_of_zero_radian_position : "); Serial.println((int32_t)modelInfoWheelRight->value_of_zero_radian_position);
  Serial.print("value_of_max_radian_position : "); Serial.println((int32_t)modelInfoWheelRight->value_of_max_radian_position);
  Serial.print("min_radian : "); Serial.println(modelInfoWheelRight->min_radian);
  Serial.print("max_radian : "); Serial.println(modelInfoWheelRight->max_radian);

  result = dxl_wb.readRegister(DXL_ID_WHEEL_RIGHT, "Velocity_Limit", &velocity_limit, &log);
  Serial.print("velocity limit : "); Serial.println(velocity_limit);
  result = dxl_wb.readRegister(DXL_ID_WHEEL_RIGHT, "Drive_Mode", &drive_mode, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.print("Failed to read the Drive Mode !");
  }
  else
  {
    Serial.print("drive mode : "); Serial.println(drive_mode);
  }
  result = dxl_wb.readRegister(DXL_ID_WHEEL_RIGHT, "Firmware_Version", &firmware_version, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.print("Failed to read Firmware version !");
  }
  else
  {
    Serial.print("firmware version : "); Serial.println(firmware_version);
  }
  

  const ModelInfo* modelInfoWheelLeft =  dxl_wb.getModelInfo(DXL_ID_WHEEL_LEFT, &log);
  Serial.println("Wheel Left Infos :");
  Serial.print("rpm : "); Serial.println(modelInfoWheelLeft->rpm);
  Serial.print("value_of_min_radian_position : "); Serial.println((int32_t)modelInfoWheelLeft->value_of_min_radian_position);
  Serial.print("value_of_zero_radian_position : "); Serial.println((int32_t)modelInfoWheelLeft->value_of_zero_radian_position);
  Serial.print("value_of_max_radian_position : "); Serial.println((int32_t)modelInfoWheelLeft->value_of_max_radian_position);
  Serial.print("min_radian : "); Serial.println(modelInfoWheelLeft->min_radian);
  Serial.print("max_radian : "); Serial.println(modelInfoWheelLeft->max_radian);

  result = dxl_wb.readRegister(DXL_ID_WHEEL_LEFT, "Velocity_Limit", &velocity_limit, &log);
  Serial.println("");
  Serial.print("wheel left velocity limit : ");
  Serial.println(velocity_limit);
  result = dxl_wb.readRegister(DXL_ID_WHEEL_LEFT, "Drive_Mode", &drive_mode, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.print("Failed to read the Drive Mode !");
  }
  else
  {
    Serial.print("drive mode : "); Serial.println(drive_mode);
  }
  result = dxl_wb.readRegister(DXL_ID_WHEEL_LEFT, "Firmware_Version", &firmware_version, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.print("Failed to read Firmware version !");
  }
  else
  {
    Serial.print("firmware version : "); Serial.println(firmware_version);
  }
  
}

void setup() {
  
  Serial.begin(57600);
  while(!Serial); // Wait for Opening Serial Monitor

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.getHardware()->setBaud(115200);
  //nh.advertise(wheel_joint_states_pub);
  //nh.advertise(arm_joint_states_pub);

  nh.advertise(joint_states_pub);
  nh.spinOnce();
  nh.subscribe(cmd_vel_sub);
  nh.spinOnce();
  nh.subscribe(cmd_arm_position_sub);
  nh.spinOnce();
  nh.subscribe(cmd_arm_joint1_position_sub);
  nh.spinOnce();
  nh.subscribe(cmd_arm_joint2_position_sub);
  nh.spinOnce();
  nh.subscribe(cmd_arm_joint3_position_sub);
  nh.spinOnce();
  nh.subscribe(cmd_arm_joint4_position_sub);
  nh.spinOnce();

  if (!initWorkbench(DEVICE_NAME,BAUDRATE)) return;

  nh.spinOnce();
  
  // Wheel Part !
  getDynamixelsWheelInfo();

  nh.spinOnce();
  
  if (!loadWheelDynamixels()) return;

  nh.spinOnce();
  
  initWheelDynamixels();

  nh.spinOnce();

  if (!initWheelSyncRead()) return;

  nh.spinOnce();
  
  initWheelSyncWrite();

  nh.spinOnce();

  //printWheelsInfos();

  // Arm Part !
  getDynamixelsArmInfo();

  nh.spinOnce();

  if (!loadArmDynamixels()) return;

  nh.spinOnce();
  
  initArmDynamixels();

  nh.spinOnce();

  if (!initArmSyncRead()) return;
  
  nh.spinOnce();
  
  initArmSyncWrite();

  nh.spinOnce();

  joint_states_msg.header.frame_id = "arduino_robot";
  joint_states_msg.velocity_length = 6;
  joint_states_msg.position_length = 6;
  joint_states_msg.effort_length = 6;
  joint_states_msg.name_length = 6;
  joint_states_msg.name = joint_names;
  joint_states_msg.position = joint_position;
  joint_states_msg.velocity = joint_velocity;
  joint_states_msg.effort = joint_current;

  nh.spinOnce();
 
}

void loop() {

  nh.spinOnce();
  
  readWheelSyncDatas();

  nh.spinOnce();
  
  readArmSyncDatas();

  nh.spinOnce();
  
  // Publish topic "arduino/joint_states"
  joint_states_msg.header.stamp = nh.now();
  joint_states_pub.publish(&joint_states_msg);
  
  nh.spinOnce();
  delay(10);
}
