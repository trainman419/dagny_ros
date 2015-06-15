/* a ROS node to act as a bridge between the serial port to the robot hardware
 * and all of the internal ROS messages that will be flying around.
 *
 * Author: Austin Hendrix
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <math.h>
#include <errno.h>

#include <set>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <dagny_driver/Goal.h>
#include <dagny_driver/Encoder.h>
#include <dagny_driver/Battery.h>

#include <diagnostic_updater/diagnostic_updater.h>


#include "protocol.h"
#include "steer.h"

using namespace std;

char laser_data[512];
int laser_ready;

// for publishing odometry and compass data
float heading;
ros::Publisher odo_pub;
ros::Publisher sonar_pub;
ros::Publisher gps_pub;
ros::Publisher heading_pub;
ros::Publisher bump_pub;

ros::Publisher encoder_pub;

// for publishing raw compass and IMU data
ros::Publisher compass_pub;
ros::Publisher imu_pub;
ros::Publisher i2c_fail_pub;

ros::Publisher battery_pub;

// for publishing user input about goals
ros::Publisher goal_input_pub;

ros::Publisher diagnostics_pub;

#define ROS_PERROR(str) ROS_ERROR("%s: %s", str, strerror(errno))

int cmd_ready = 0;
char cmd_buf[12];
Packet cmd_packet('C', 12, cmd_buf);
ros::Time last_cmd_t;

// TODO: subscribe to ackermann_msgs::AckermannDrive too/instead
void cmdCallback( const geometry_msgs::Twist::ConstPtr & cmd_vel ) {
   // internal speed specified as 2000/(ms per count)
   // 2 / (sec per count)
   // 2 * counts / sec
   // ( 1 count = 0.03 m )
   // 1/2 * 0.032 m / sec
   // 0.016 m / sec
   // target speed in units of 0.016 m / sec
   //
   // internal speed specified as ticks/sec / 10
   // ( 1 count = 0.08m )
   // target speed in increments of 0.08 m/sec
   // 1/0.08 = 12.5
   int16_t target_speed = cmd_vel->linear.x * 12.5;
   // angular z > 0 is left
   // vr = vl / r
   // r = vl / vr
   int8_t steer = 0;
   if( cmd_vel->angular.z == 0.0 ) {
      steer = 0;
   } else {
      float radius = fabs(cmd_vel->linear.x / cmd_vel->angular.z);
      int16_t tmp = radius2steer(radius);

      if( tmp < -120 ) 
         tmp = -120;
      if( tmp > 120 ) 
         tmp = 120;

      if( cmd_vel->angular.z > 0 ) {
         steer = -tmp;
      } else {
         steer = tmp;
      }
   }

   last_cmd_t = ros::Time::now();

   cmd_packet.reset();
   cmd_packet.append(target_speed);
   cmd_packet.append(steer);
   cmd_packet.finish();
   cmd_ready = 1;
}

int goal_ready = 0;
char goal_buf[32];
Packet goal_packet('L', sizeof(goal_buf), goal_buf);

void goalUpdateCallback( const dagny_driver::Goal::ConstPtr & goal) {
   switch( goal->operation ) {
      case dagny_driver::Goal::SET_CURRENT:
         goal_packet.reset();
         goal_packet.append(goal->operation);
         goal_packet.append(goal->id);
         goal_packet.finish();
         goal_ready = 1;
         break;
      default:
         ROS_ERROR("Unknown goal update: %d", goal->operation);
         break;
   }
   return;
}

int compass_cal_ready = 0;
char compass_cal_buf[128];
Packet compass_cal_packet('O', sizeof(compass_cal_buf), compass_cal_buf);

void compassCalCallback( const geometry_msgs::Vector3::ConstPtr & msg ) {
   compass_cal_packet.reset();
   compass_cal_packet.append((float)msg->x);
   compass_cal_packet.append((float)msg->y);
   compass_cal_packet.append((float)msg->z);
   compass_cal_packet.finish();
   compass_cal_ready = 1;
}

int imu_cal_ready = 0;
char imu_cal_buf[128]; // 6 * 4(float) * 2(escape) = 48 bytes max
Packet imu_cal_packet('I', sizeof(imu_cal_buf), imu_cal_buf);

void imuCalCallback( const geometry_msgs::Twist::ConstPtr & msg ) {
   imu_cal_packet.reset();
   imu_cal_packet.append((float)msg->angular.x);
   imu_cal_packet.append((float)msg->angular.y);
   imu_cal_packet.append((float)msg->angular.z);
   imu_cal_packet.append((float)msg->linear.x);
   imu_cal_packet.append((float)msg->linear.y);
   imu_cal_packet.append((float)msg->linear.z);
   imu_cal_packet.finish();
   imu_cal_ready = 1;
}

int steering_offset_ready = 0;
char steering_offset_buf[128]; // 6 * 4(float) * 2(escape) = 48 bytes max
Packet steering_offset_packet('S', sizeof(steering_offset_buf), steering_offset_buf);

void steeringOffsetCallback( const std_msgs::Int8::ConstPtr & msg ) {
   steering_offset_packet.reset();
   steering_offset_packet.append(msg->data);
   steering_offset_packet.finish();
   steering_offset_ready = 1;
}

#define handler(foo) void foo(Packet & p)
typedef void (*handler_ptr)(Packet & p);

handler_ptr handlers[256];

handler(no_handler) {
   int l = p.outsz();
   const char * in = p.outbuf();
   char * buf = (char*)malloc(l + 1);
   memcpy(buf, in, l);
   buf[l] = 0;

   char * tmpbuf = (char*)malloc(5*l + 1);
   int i;
   for( i=0; i<l; i++ ) {
      sprintf(tmpbuf + (i*5), "0x%02X ", 0xFF & buf[i+1]);
   }
   tmpbuf[i*5] = 0;

   ROS_INFO("No handler for message: %02X(%d) %s", buf[0], l, tmpbuf);

   free(buf);
   free(tmpbuf);
}

handler(shutdown_h) {
   int l = p.outsz();
   const char * in = p.outbuf();
   int shutdown = 0;
   if( l == 9 ) {
      shutdown = 1;
      for( int i=0; i<l; i++ ) {
         if( in[i] != 'Z' ) shutdown = 0;
      }
   }
   if( shutdown ) {
      ROS_INFO("Received shutdown");
      if( system("sudo poweroff") < 0 ) {
         ROS_ERROR("Failed to execute shutdown command");
      }
   } else {
      char * buf = (char*)malloc(l + 1);
      memcpy(buf, in, l);
      buf[l] = 0;
      ROS_INFO("Malformed shutdown %s", buf);
      free(buf);
   }
}

ros::Time last_gps;

handler(gps_h) {
   // message format
   // int32_t lat
   // int32_t lon
   int32_t lat = p.reads32();
   int32_t lon = p.reads32();
   //ROS_INFO("GPS lat: %d lon: %d", lat, lon);
   sensor_msgs::NavSatFix gps;
   gps.header.stamp = ros::Time::now();
   gps.header.frame_id = "gps";
   gps.latitude = lat / 1000000.0;
   gps.longitude = lon / 1000000.0;

   // pass altitude up from driver
   int32_t altitude = p.reads32(); // altitude in centimeters
   gps.altitude = altitude / 100.0;


   // fill in static data
   gps.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
   gps.position_covariance_type = 
      sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

   // pass HDOP up from driver
   uint32_t hdop_100 = p.readu32(); // HDOP * 100
   double hdop = hdop_100 / 100.0;
   // covariance from HDOP calculation borrowd from nmea_navsat driver
   gps.position_covariance[0] = hdop*hdop;
   gps.position_covariance[4] = hdop*hdop;
   gps.position_covariance[8] = (2*hdop)*(2*hdop); // FIXME
   if( hdop_100 == 0 ) {
      gps.position_covariance_type =
         sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
   } else {
      gps.position_covariance_type =
         sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
   }

   // publish
   gps_pub.publish(gps);
   last_gps = ros::Time::now();
}

// set up odometry handling
void odometry_setup(void) {
}

// squares per encoder count
#define Q_SCALE 0.29

handler(odometry_h) {
   static tf::TransformBroadcaster odom_tf;
   // message format:
   // float linear
   // float angular
   // float x
   // float y
   // float yaw
   nav_msgs::Odometry odo_msg;
   odo_msg.header.stamp = ros::Time::now();
   odo_msg.header.frame_id = "odom";
   odo_msg.child_frame_id = "base_link";
   odo_msg.twist.twist.linear.x = p.readfloat();
   odo_msg.twist.twist.angular.z = p.readfloat();
   odo_msg.pose.pose.position.x = p.readfloat();
   odo_msg.pose.pose.position.y = p.readfloat();
   float yaw = p.readfloat();
   odo_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

   odo_pub.publish(odo_msg);

   // tf transform
   geometry_msgs::TransformStamped transform;
   transform.header = odo_msg.header;
   transform.child_frame_id = odo_msg.child_frame_id;
   transform.transform.translation.x = odo_msg.pose.pose.position.x;
   transform.transform.translation.y = odo_msg.pose.pose.position.y;
   transform.transform.translation.z = odo_msg.pose.pose.position.z;
   transform.transform.rotation = odo_msg.pose.pose.orientation;
   odom_tf.sendTransform(transform);

   uint8_t b = p.readu8();
   std_msgs::Bool bump;
   bump.data = (b != 0);
   bump_pub.publish(bump);

   int16_t qcount = p.reads16();
   int8_t steer = p.reads8();

   dagny_driver::Encoder enc_msg;
   enc_msg.header = odo_msg.header;
   enc_msg.count = qcount;
   enc_msg.steer = steer;
   encoder_pub.publish(enc_msg);
}

uint16_t idle_cnt;
uint8_t i2c_resets;

handler(idle_h) {
   // idle message format:
   // uint16_t idle
   // uint8_t i2c_failures
   // uint8_t i2c_resets
   idle_cnt = p.readu16();
   std_msgs::UInt8 i2c_fail;
   i2c_fail.data = p.readu8();
   i2c_resets = p.readu8();
   //ROS_INFO("I2C state: %d", p.readu8());
   i2c_fail_pub.publish(i2c_fail);
}

#define NUM_SONARS 5
handler(sonar_h) {
   // sonar message format:
   // uint8_t[5] sonars
   char sonar_frames[5][8] = { "sonar_1", "sonar_2", "sonar_3", "sonar_4", "sonar_5" };
   uint8_t s;
   ros::Time n = ros::Time::now();
   sensor_msgs::Range sonar;
   int i = p.readu8();
   s = p.readu8();
   sonar.range = s * 0.0254; // convert inches to m
   sonar.min_range = 6 * 0.0254;
   sonar.max_range = 255 * 0.0254;
   sonar.field_of_view = 45 * M_PI / 180.0; // approx 45-degree FOV
   sonar.radiation_type = sensor_msgs::Range::ULTRASOUND;

   sonar.header.stamp = n;
   sonar.header.frame_id = sonar_frames[i];

   sonar_pub.publish(sonar);
}

handler(imu_h) {
   // imu message format:
   // float[3]
   //float x, y, z;
   float z;
   /* x = */ p.readfloat();
   /* y = */ p.readfloat();
   z = p.readfloat();
   //ROS_INFO("IMU data: (% 03.7f, % 03.7f, % 03.7f)", x, y, z);
   heading = z;
   std_msgs::Float32 h;
   h.data = z;
   heading_pub.publish(h);
}

handler(raw_imu_h) {
   // gyro: xyz, accel: xyz
   float gx, gy, gz, ax, ay, az;
   gx = p.readfloat();
   gy = p.readfloat();
   gz = p.readfloat();
   ax = p.readfloat();
   ay = p.readfloat();
   az = p.readfloat();
   sensor_msgs::Imu imu;
   imu.header.stamp = ros::Time::now();
   imu.header.frame_id = "base_link";

   // we are providing gyro and accel data
   // don't know covariances yet
   imu.angular_velocity.x = gx;
   imu.angular_velocity.y = gy;
   imu.angular_velocity.z = gz;
   imu.linear_acceleration.x = ax;
   imu.linear_acceleration.y = ay;
   imu.linear_acceleration.z = az;

   /* Analysis of gyro and accelerometer bias
Analyzing 10019 samples
Gyro bias [ 0.00445241  0.00769426  0.00205139]
Gyro covariance 
[[  5.95420731e-07   9.98214066e-08   2.42934262e-08]
 [  9.98214066e-08   6.01292941e-07   5.17353335e-08]
 [  2.42934262e-08   5.17353335e-08   2.66484062e-07]]
Accelerometer bias [  0.06085436   0.26828517 -10.04750346]
Accelerometer covaraince 
[[ 0.48834601  0.06811885 -0.17546435]
 [ 0.06811885  0.22851019 -0.262803  ]
 [-0.17546435 -0.262803    0.74666544]]

Analyzing 10002 samples
Gyro bias [ 0.00070364  0.00198244  0.00054909]
Gyro covariance 
[[  6.73927948e-07   3.19005125e-08   1.02816917e-08]
 [  3.19005125e-08   5.00567915e-07   1.76859385e-08]
 [  1.02816917e-08   1.76859385e-08   3.04065499e-07]]
Accelerometer bias [-0.01961441 -0.04073214 -9.72075944]
Accelerometer covaraince 
[[ 0.41593466  0.04711931 -0.17112025]
 [ 0.04711931  0.18881759 -0.231505  ]
 [-0.17112025 -0.231505    0.67130527]]

Analyzing 1007 samples
Gyro bias [-0.0106572  -0.03959371 -0.12854336]
Gyro covariance 
[[  8.46049026e-07   6.01570997e-09   8.28312447e-09]
 [  6.01570997e-09   3.67726194e-07  -1.30979674e-08]
 [  8.28312447e-09  -1.30979674e-08   2.98437229e-07]]
Accelerometer bias [ 0.18425708  0.41223157 -9.11863052]
Accelerometer covaraince 
[[ 0.16192873  0.0078776  -0.05683903]
 [ 0.0078776   0.09970528 -0.0982002 ]
 [-0.05683903 -0.0982002   0.30818165]]

Analyzing 10009 samples
Gyro bias [  4.48896858e-05   4.73079522e-04   6.43365116e-05]
Gyro covariance 
[[  8.98930371e-07   1.76998656e-08   2.23517490e-09]
 [  1.76998656e-08   4.18385259e-07  -2.29157393e-10]
 [  2.23517490e-09  -2.29157393e-10   3.00280631e-07]]
Accelerometer bias [ 0.01731151  0.01019684 -9.80573254]
Accelerometer covaraince 
[[ 0.14006225  0.0123979  -0.05739361]
 [ 0.0123979   0.08528796 -0.07799387]
 [-0.05739361 -0.07799387  0.25775234]]

Analyzing 10011 samples
Gyro bias [-0.00011972 -0.00025375  0.00016198]
Gyro covariance 
[[  8.83834931e-07   3.17464674e-08  -5.98197474e-09]
 [  3.17464674e-08   4.45522774e-07  -1.37527660e-08]
 [ -5.98197474e-09  -1.37527660e-08   3.10821882e-07]]
Accelerometer bias [ -6.89599977e-03  -5.49639670e-03  -9.81453175e+00]
Accelerometer covaraince 
[[ 0.13798447  0.01432473 -0.06066546]
 [ 0.01432473  0.0873555  -0.08171402]
 [-0.06066546 -0.08171402  0.26964842]]
*/

   // no orientation data
   imu.orientation_covariance[0] = -1;

   imu_pub.publish(imu);
}

handler(compass_h) {
   float mx, my, mz;
   mx = p.readfloat();
   my = p.readfloat();
   mz = p.readfloat();
   geometry_msgs::Vector3Stamped compass;
   compass.header.stamp = ros::Time::now();
   compass.vector.x = mx;
   compass.vector.y = my;
   compass.vector.z = mz;
   compass_pub.publish(compass);
}

handler(goal_h) {
   int8_t op;
   op = p.reads8();
   dagny_driver::Goal g;
   g.operation = op;
   switch(op) {
      case dagny_driver::Goal::APPEND:
         g.goal.latitude = p.reads32() / 1000000.0;
         g.goal.longitude = p.reads32() / 1000000.0;
         ROS_INFO("Add goal at lat %lf, lon %lf", g.goal.latitude, 
               g.goal.longitude);
         break;
      case dagny_driver::Goal::DELETE:
         g.id = p.reads32();
         ROS_INFO("Remove goal at %d", g.id);
         break;
      default:
         ROS_ERROR("Got unknown goal update %d", op);
         return;
   }
   goal_input_pub.publish(g);
}

handler(battery_h) {
   uint8_t main = p.readu8();
   uint8_t motor = p.readu8();
   const uint8_t battery_max = 84;
   const uint8_t battery_min = 70;
   // below this, we assume we're on wall power
   const uint8_t main_cutoff = 60; // TODO: update properly for LiPo batteries
   // lower limit; don't report below this. I usually see 1 or 2 when the
   // motor power switch is OFF
   const uint8_t motor_cutoff = 4;
   dagny_driver::Battery battery_msg;
   battery_msg.header.stamp = ros::Time::now();
   battery_msg.main_raw = main;
   battery_msg.motor_raw = motor;

   // bounds checking
   if( main > battery_max ) {
      ROS_WARN("Main battery is above maximum: %d", main);
   }
   if( main < battery_min && main > main_cutoff ) {
      ROS_WARN("Main battery is low: %d", main);
   }

   if( motor > battery_max ) {
      ROS_WARN("Motor battery is above maximum: %d", motor);
   }
   if( motor < battery_min && motor > motor_cutoff ) {
      ROS_WARN("Motor battery is low: %d", motor);
   }

   // compute very rough and wrong battery level
   if( main <= main_cutoff ) {
      battery_msg.main = 1.0;
   } else {
      battery_msg.main = float(main - battery_min)/(battery_max - battery_min);
      battery_msg.main = min(max(battery_msg.main, 0.0f), 1.0f);
   }

   // motors can't run off wall power
   battery_msg.motor = float(motor - battery_min)/(battery_max - battery_min);
   battery_msg.motor = min(max(battery_msg.motor, 0.0f), 1.0f);

   battery_pub.publish(battery_msg);
}

int bandwidth = 0;

void idle_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat) {
   // Idle Count
   if( idle_cnt < 200 ) {
      // error
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
            "ERROR: AVR too busy");
   } else if( idle_cnt < 400 ) {
      // warn
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,
            "Warning: AVR load high");
   } else {
      // OK
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
            "OK: AVR load normal");
   }
   stat.addf("Idle Count", "%d", idle_cnt);
}

void bandwidth_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat) {
   if( bandwidth == 0 ) {
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
            "ERROR: No AVR data");
   } else if( bandwidth < 1000 ) {
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,
            "Warning: Low AVR bandwidth");
   } else if( bandwidth > 1500 ) {
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,
            "Warning: High AVR bandwidth");
   } else {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
            "OK: AVR bandwidth normal");
   }
   stat.addf("Bandwidth", "%d bytes/sec", bandwidth);
}

void i2c_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat) {
   if( i2c_resets == 0 ) {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
            "OK: No I2C resets");
   } else if( i2c_resets < 5 ) {
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN,
            "Warning: %d I2C resets", i2c_resets);
   } else {
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
            "Error: %d I2C resets", i2c_resets);
   }
}

void gps_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat) {
   double gps_diff = (ros::Time::now() - last_gps).toSec();
   if( gps_diff < 1.1 ) {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
            "OK: GPS fix good");
   } else {
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,
            "Warning: GPS out of date");
   }
}

#define IN_BUFSZ 1024

int main(int argc, char ** argv) {
   unsigned char in_buffer[IN_BUFSZ];
   int in_cnt = 0;
   int cnt = 0;
   int i;

   char heartbeat_buf[8];
   Packet heartbeat_packet('H', 8, heartbeat_buf);

   laser_ready = 0;

   for( i=0; i<512; i++ ) {
      laser_data[i] = 64;
   }

   // Set up message handler array
   for( i=0; i<256; i++ ) {
      handlers[i] = no_handler;
   }

   odometry_setup();
   handlers['O'] = odometry_h;
   handlers['I'] = idle_h;

   //gps_setup();
   handlers['G'] = gps_h;
   handlers['S'] = sonar_h;
   handlers['U'] = imu_h;
   
   // raw IMU handlers
   handlers['M'] = compass_h;
   handlers['V'] = raw_imu_h;

   // goal hander
   handlers['L'] = goal_h;

   // battery handler
   handlers['B'] = battery_h;

   ros::init(argc, argv, "dagny_driver");

   ros::NodeHandle n;

   last_cmd_t = ros::Time::now();

   // I'm going to hardcode the port and settings because this is hardware-
   // specific anyway
   // open serial port
   string serial_port;
   n.param<std::string>("port", serial_port, "/dev/ttyACM0");
   int serial = open(serial_port.c_str(), O_RDWR | O_NOCTTY);
   if( serial < 0 ) {
      perror(("Failed to open " + serial_port).c_str());
      // die. ungracefully.
      return -1;
   }

   struct termios tio;
   tcgetattr(serial, &tio);

   // set non-blocking input mode
   tio.c_lflag = 0; // raw input
   tio.c_cc[VMIN] = 0;
   tio.c_cc[VTIME] = 0;

   // no input options, just normal input
   tio.c_iflag = 0;

   // set baud rate
   cfsetospeed(&tio, B115200);
   cfsetispeed(&tio, B115200);
   
   tcsetattr(serial, TCSANOW, &tio);

   sleep(2); // sleep for two seconds while bootloader runs

   ros::Subscriber cmd_sub = n.subscribe("cmd_vel", 1, cmdCallback);

   ros::Subscriber goal_updates_sub = n.subscribe("goal_updates", 10, goalUpdateCallback);
   ros::Subscriber compass_offset_sub = n.subscribe("compass_cal", 2, compassCalCallback);
   ros::Subscriber imu_offset_sub = n.subscribe("imu_cal", 2, imuCalCallback);
   ros::Subscriber steering_offset_sub = n.subscribe("steering_offset",
         2, steeringOffsetCallback);

   odo_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
   sonar_pub = n.advertise<sensor_msgs::Range>("sonar", 10);
   gps_pub = n.advertise<sensor_msgs::NavSatFix>("gps", 10);
   heading_pub = n.advertise<std_msgs::Float32>("heading", 10);
   bump_pub = n.advertise<std_msgs::Bool>("bump", 10);
   encoder_pub = n.advertise<dagny_driver::Encoder>("encoder", 10);

   compass_pub = n.advertise<geometry_msgs::Vector3Stamped>("magnetic", 10);
   imu_pub = n.advertise<sensor_msgs::Imu>("imu", 10);
   i2c_fail_pub = n.advertise<std_msgs::UInt8>("i2c_fail", 10);

   // latched, so we can always pick up the most recent data
   battery_pub = n.advertise<dagny_driver::Battery>("battery", 1, true);

   goal_input_pub = n.advertise<dagny_driver::Goal>("goal_input", 10);

   diagnostic_updater::Updater updater;
   updater.setHardwareID("Dagny");
   updater.add("AVR Load", idle_diagnostics);
   updater.add("AVR Bandwidth", bandwidth_diagnostics);
   updater.add("I2C Status", i2c_diagnostics);
   updater.add("GPS Status", gps_diagnostics);

   ROS_INFO("dagny_driver ready");

   ros::Rate loop_rate(20);
   ros::Time now;

   int itr = 0;
   int bw = 0;

   while( ros::ok() ) {
      cnt = read(serial, in_buffer + in_cnt, IN_BUFSZ - in_cnt - 1); 
      if( cnt > 0 ) {
         // append a null byte
         in_buffer[cnt + in_cnt] = 0;
         in_cnt += cnt;
         // parse out newline-terminated strings and call appropriate functions
         int start = 0;
         int i = 0;
         while( i < in_cnt ) {
            for( ; i < in_cnt && in_buffer[i] != '\r' ; i++);

            if( i < in_cnt && in_buffer[i] == '\r' ) {
               // check that our string isn't just the terminating character
               if( i - start > 1 ) {
                  // we got a string. call the appropriate function
                  Packet p((char*)(in_buffer+start), i-start);
                  handlers[in_buffer[start]](p);
               }
               start = i+1;
            }
            i++;
         }

         // shift remaining data to front of buffer
         for( i=start; i<in_cnt; i++ ) {
            in_buffer[i-start] = in_buffer[i];
         }

         in_cnt -= start;
      }
      bw += cnt;
      
      ros::spinOnce();
      now = ros::Time::now();

      if( laser_ready ) {
         cnt = write(serial, "L", 1);
         cnt = write(serial, laser_data, 512);
         cnt = write(serial, "\r\r\r\r\r\r\r\r", 1);
         laser_ready = 0;
      }

      if( cmd_ready ) {
         cnt = write(serial, cmd_packet.outbuf(), cmd_packet.outsz());
         //ROS_INFO("cmd_vel sent");
         if( cnt != cmd_packet.outsz() ) {
            ROS_ERROR("Failed to send cmd_vel data");
         }
         cmd_ready = 0;
      } else if( now - last_cmd_t > ros::Duration(1.0) ) {
         last_cmd_t = now;
         ROS_WARN("Command timeout; restting to 0");
         // command timeout; send zero
         cmd_packet.reset();
         cmd_packet.append(uint16_t(0));
         cmd_packet.append(int8_t(0));
         cmd_packet.finish();

         cnt = write(serial, cmd_packet.outbuf(), cmd_packet.outsz());
         //ROS_INFO("cmd_vel sent");
         if( cnt != cmd_packet.outsz() ) {
            ROS_ERROR("Failed to send cmd_vel data");
         }
         cmd_ready = 0;
      }

      if( goal_ready ) {
         cnt = write(serial, goal_packet.outbuf(), goal_packet.outsz());
         if( cnt != goal_packet.outsz() ) {
            ROS_ERROR("Failed to send goal update");
         }
         goal_ready = 0;
      }

      if( compass_cal_ready ) {
         cnt = write(serial, compass_cal_packet.outbuf(), 
               compass_cal_packet.outsz());
         if( cnt != compass_cal_packet.outsz() ) {
            ROS_ERROR("Failed to send compass update");
         }
         compass_cal_ready = 0;
      }

      if( imu_cal_ready ) {
         cnt = write(serial, imu_cal_packet.outbuf(), 
               imu_cal_packet.outsz());
         if( cnt != imu_cal_packet.outsz() ) {
            ROS_ERROR("Failed to send imu update");
         }
         imu_cal_ready = 0;
      }

      if( steering_offset_ready ) {
         cnt = write(serial, steering_offset_packet.outbuf(), 
               steering_offset_packet.outsz());
         if( cnt != steering_offset_packet.outsz() ) {
            ROS_ERROR("Failed to send imu update");
         }
         steering_offset_ready = 0;
      }

      // send heartbeat
      ++itr;

      // heartbeat and bandwidth measurement every 0.5 sec
      if( itr == 9 ) {
         heartbeat_packet.reset();
         heartbeat_packet.finish();
         cnt = write(serial,heartbeat_packet.outbuf(),heartbeat_packet.outsz());
         itr = 0;
         bandwidth = bw * 2;
         bw = 0;
      }

      updater.update();

      loop_rate.sleep();
   }
}
