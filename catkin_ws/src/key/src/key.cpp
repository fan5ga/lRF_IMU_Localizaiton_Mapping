#include <ros/ros.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h> /* File control definitions */
#include <errno.h>
#include <termios.h> /* POSIX terminal control definitions */


#define KEYCODE_R 0x64 
#define KEYCODE_L 0x61
#define KEYCODE_U 0x77
#define KEYCODE_D 0x73

int fd; /* File descriptor for the port */

class TeleopKey
{
public:

  void keyLoop();

private:

  ros::NodeHandle nh_; 
  
};


int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

int open_port(char** argv)
{
	fd = open(argv[1], O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1)
	{
		/*
		 * Could not open the port.
		 */
		perror("open_port: Unable to open /dev/ttyUSB0-");
	}
	else
	{
		return (fd);
	}
}

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "Teleop_Key");
 
  TeleopKey Teleop_Key;

  fd = open_port(argv);

  signal(SIGINT,quit);

  Teleop_Key.keyLoop();
  
  return(0);
}


void TeleopKey::keyLoop()
{
  char c;
  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use w, s, a, d keys to move");


  for(;;)
  {
	tcflush(fd, TCIFLUSH);
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_L:
        ROS_INFO("LEFT");
		write(fd, "mvtc=12,-12\r\n", 13);		
        break;
      case KEYCODE_R:
        ROS_INFO("RIGHT");
		write(fd, "mvtc=-12,12\r\n", 13);
        break;
      case KEYCODE_U:
        ROS_INFO("UP");
		write(fd, "mvtc=8,8\r\n", 12);
		break;
      case KEYCODE_D:
        ROS_INFO("DOWN");
		write(fd, "mvtc=-8,-8\r\n", 14);
        break;
		
	  default :
	    ROS_INFO("0");
		write(fd, "mvtc=0,0\r\n", 10);
    }
   

 }


  return;
}

