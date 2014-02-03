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

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "protocol.h"

char laser_data[512];
int laser_ready;

// callback on laser scan received.
void laserCallback(const sensor_msgs::LaserScan::ConstPtr & msg) {
   //ROS_INFO("Data size %d", msg->ranges.size());
   //for(int i=0; i<msg->ranges.size(); i+= 2 ) {
   for(unsigned int i=0; i<msg->ranges.size(); i++ ) {
      if( i < 512 ) {
         // average adjacent data points
         //float data = (msg->ranges[i] + msg->ranges[i+1]) / 2;
         float data = msg->ranges[i];
         // scale to fit into a byte. 250 = 5.0m
         data = data * 50;
         //laser_data[i/2] = (char)data;
         laser_data[i] = (char)data;
      }
   }

   /*ROS_INFO("Angle min: %lf, angle delta: %lf, angle max: %lf",
      msg->angle_min * 180.0 / M_PI, 
      msg->angle_increment * 180.0 / M_PI,
      msg->angle_max * 180.0 / M_PI);
      */

   laser_ready = 1;
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
   ROS_INFO("No handler for message: %s", buf);
   free(buf);
}

handler(shutdown_h) {
   int l = p.outsz();
   const char * in = p.outbuf();
   int shutdown = 1;
   if( l == 9 ) {
      for( int i=0; i<l; i++ ) {
         if( in[i] != 'Z' ) shutdown = 0;
      }
   }
   if( shutdown ) {
      ROS_INFO("Received shutdown");
      // FIXME: shutdown here
      //system("sudo poweroff");
   } else {
      char * buf = (char*)malloc(l + 1);
      memcpy(buf, in, l);
      buf[l] = 0;
      ROS_INFO("Malformed shutdown %s", buf);
      free(buf);
   }
}

// set up whatever we decide to do for GPS
void gps_setup(void) {
}

handler(gps_h) {
   // take data from GPS and spew it to a fifo somewhere on disk
   //
   // for now, just de-encapsulate and print it to info
   int l = p.outsz() - 1;
   char * buf = (char*)malloc(l + 1);
   memcpy(buf, p.outbuf() + 1, l);
   buf[l] = 0;
   ROS_INFO("Received GPS: %s", p.outbuf());
   free(buf);
}

// set up odometry handling
void odometry_setup(void) {
}

inline int read16(char * in) {
   return in[0] | (in[1] << 8);
}
handler(odometry_h) {
   int rcount = p.readu16();
   int lcount = p.readu16();
   int qcount = p.readu16();
   int rspeed = p.reads16();
   int lspeed = p.reads16();
   int qspeed = p.reads16();
   // TODO: publish odometry
   ROS_INFO("Odo: rc: %d, lc: %d, qc: %d, rs: %d, ls: %d, qs: %d",
            rcount, lcount, qcount, rspeed, lspeed, qspeed);
}

#define IN_BUFSZ 1024

int main(int argc, char ** argv) {
   char in_buffer[IN_BUFSZ];
   int in_cnt = 0;
   int cnt = 0;
   int i;

   laser_ready = 0;

   for( i=0; i<512; i++ ) {
      laser_data[i] = 64;
   }

   // Set up message handler array
   for( i=0; i<256; i++ ) {
      handlers[i] = no_handler;
   }
   handlers['Z'] = shutdown_h;

   odometry_setup();
   handlers['O'] = odometry_h;

   handlers['G'] = gps_h;

   ros::init(argc, argv, "hardware_interface");

   ros::NodeHandle n;

   // TODO: set up publishers here. I don't think we have any yet
   //  will probably want publishers for battery, sonar, wheel and bump data

   // deal with GPS data some other way; probably write it to a fifo and have
   //  gpsd pick it up. TODO
   //
   
   // I'm going to hardcode the port and settings because this is hardware-
   // specific anyway
   // open serial port
   int serial = open("/dev/ttyS1", O_RDWR | O_NOCTTY);
   if( serial < 0 ) {
      perror("Failed to open /dev/ttyS1");
      // die. ungracefully.
      return -1;
   }

   struct termios tio;
   tcgetattr(serial, &tio);

   // set non-blocking input mode
   tio.c_lflag = 0; // raw input
   tio.c_cc[VMIN] = 0;
   tio.c_cc[VTIME] = 0;

   /*ROS_INFO("c_iflag %X", tio.c_iflag);
   ROS_INFO("INLCR %X", INLCR);
   ROS_INFO("IGNCR %X", IGNCR);
   ROS_INFO("ICRNL %X", ICRNL);
   ROS_INFO("IXON  %X", IXON);
   ROS_INFO("IXOFF  %X", IXOFF);*/
   // no input options, just normal input
   tio.c_iflag = 0;

   // set baud rate
   cfsetospeed(&tio, B115200);
   cfsetispeed(&tio, B115200);
   
   tcsetattr(serial, TCSANOW, &tio);

   ros::Subscriber sub = n.subscribe("scan", 5, laserCallback);

   ros::Rate loop_rate(10);

   while( ros::ok() ) {
      
      // write pending data to serial port
      //ROS_INFO("start laser transmit");
      if( laser_ready ) {
         cnt = write(serial, "L", 1);
         //ROS_INFO("Wrote %d bytes", cnt);
         cnt = write(serial, laser_data, 512);
         //ROS_INFO("Wrote %d bytes", cnt);
         cnt = write(serial, "\r\r\r\r\r\r\r\r", 1);
         //ROS_INFO("Wrote %d bytes", cnt);
         laser_ready = 0;
      }

      //ROS_INFO("start serial input");
      cnt = read(serial, in_buffer + in_cnt, IN_BUFSZ - in_cnt - 1); 
      if( cnt > 0 ) {
         // append a null byte
         in_buffer[cnt + in_cnt] = 0;
         //ROS_INFO("Read %d characters", cnt);
         //ROS_INFO("Read %s", in_buffer);
         in_cnt += cnt;
         ROS_INFO("Buffer size %d", in_cnt);

         // parse out newline-terminated strings and call appropriate functions
         int start = 0;
         int i = 0;
         while( i < in_cnt ) {
            for( ; i < in_cnt && in_buffer[i] != '\r' ; i++);

            if( in_buffer[i] == '\r' ) {
               // check that our string isn't just the terminating character
               if( i - start > 1 ) {
                  // we got a string. call the appropriate function
                  Packet p(in_buffer+start, i-start);
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
      
      ros::spinOnce();

      loop_rate.sleep();
   }
}
