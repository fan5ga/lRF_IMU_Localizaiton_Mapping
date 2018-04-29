#include <ros/ros.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h> /* File control definitions */
#include <errno.h>
#include <termios.h> /* POSIX terminal control definitions */
#include<tf/transform_broadcaster.h>
#include<nav_msgs/Odometry.h>

//#include<rtt/os/Timer.hpp>

#define KEYCODE_R 0x64
#define KEYCODE_L 0x61
#define KEYCODE_U 0x77
#define KEYCODE_D 0x73
#define DIST 0.0000050265482457

int kfd;
struct termios cooked, raw;
int fd; /* File descriptor for the port */
int fd1;

int open_port(char* argv)
{
    int fd;
    fd = open(argv, O_RDWR|O_NOCTTY|O_NDELAY);
//    fd = open("/dev/ttyUSB0", O_RDWR|O_NOCTTY|O_NDELAY);
    if (fd == -1)
    {
        perror("open_port: Unable to open /dev/ttyUSB0-");
    }
    else
    {
        return fd;
    }
}
int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio,oldtio;
    if  ( tcgetattr( fd,&oldtio)  !=  0)
    {
        perror("SetupSerial 1");
        return -1;
    }
    bzero( &newtio, sizeof( newtio ) );
    newtio.c_cflag  |=  CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    switch( nBits )
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    }

    switch( nEvent )
    {
    case 'O':                     //奇校验
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'E':                     //偶校验
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'N':                    //无校验
        newtio.c_cflag &= ~PARENB;
        break;
    }

switch( nSpeed )
    {
    case 57600:
        cfsetispeed(&newtio, B57600);
        cfsetospeed(&newtio, B57600);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }
    if( nStop == 1 )
    {
        newtio.c_cflag &=  ~CSTOPB;
    }
    else if ( nStop == 2 )
    {
        newtio.c_cflag |=  CSTOPB;
    }
    newtio.c_cc[VTIME]  = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush(fd,TCIFLUSH);
    if((tcsetattr(fd,TCSANOW,&newtio))!=0)
    {
        perror("com set error");
        return -1;
    }
    printf("set done!\n");
    return 0;
}

char callrc(std::string data, int len)
{
    int i;
    char tmp = data[1];
    for(i = 2; i<len; i++)
        tmp = tmp^data[i];
    return tmp;
}
std::string servo_on(void)
{
    char stx = 0x02;
    char etx = 0x03;
    std::stringstream s;
    s<<stx;
    s<< "DB11";
    s<<etx;
    char lrc = callrc(s.str(), s.str().length());
    s<<lrc;
    return s.str();
}
std::string mod_p(void)
{
    char stx = 0x02;
    char etx = 0x03;
    std::stringstream s;
    s<<stx;
    s<< "CZ00";
    s<<etx;
    char lrc = callrc(s.str(), s.str().length());
    s<<lrc;
    return s.str();
}
std::string speed(int left, int right)
{
    char stx = 0x02;
    char etx = 0x03;
    std::stringstream s;
    s<<stx;
    s<< "BH";
    s<< left;
    s<< ';';
    s<< right;
    s<<etx;
    char lrc = callrc(s.str(), s.str().length());
    s<<lrc;
    std::cout<<s.str().c_str()<<std::endl;
    return s.str();
}
std::string posi(void)
{
    char stx = 0x02;
    char etx = 0x03;
    std::stringstream s;
    s<<stx;
    s<< "AC0";
    s<<etx;
    char lrc = callrc(s.str(), s.str().length());
    s<<lrc;
    return s.str();
}

void quit(int sig)
{
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
    float heading;
    char ch_heading[4];
    int start = 1;
    char ce = 0xce;
    int c_l_encode, c_r_encode, p_l_encode, p_r_encode;
    double dist, posi_x, posi_y;
    posi_x = 0;
    posi_y = 0;
    char temp[24];
    std::string encode;
    std::string command;
    int int_left=0;
    int int_right=0;
    ros::init(argc, argv, "Teleop_Key");
    ros::NodeHandle nh_;
    tf::TransformBroadcaster br;
    ros::Publisher odom_pub = nh_.advertise<nav_msgs::Odometry>("odom", 50);
    nav_msgs::Odometry odom;
    tf::Transform transform0;
    tf::Quaternion q;

    fd = open_port(argv[1]);
    fd1 = open_port(argv[2]);
    set_opt(fd,115200,8,'N',1);
    set_opt(fd1,115200,8,'N',1);

//    signal(SIGINT,quit);
    command = servo_on();
    write(fd,command.c_str(),command.size());
    ros::Duration(0.01).sleep();
    command = mod_p();
    write(fd,command.c_str(),command.size());
    ros::Duration(0.01).sleep();
    fd_set inputs;
    struct timeval timeout = {5,0};

    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
    while(1)
    {
        FD_ZERO(&inputs);
        FD_SET(kfd,&inputs);
        timeout.tv_sec = 0;
        if(select(kfd+1,&inputs,NULL,NULL,&timeout)<=0);
//            std::cout<<"error"<<std::endl;
        else
        {
            char c;
            read(kfd,&c,1);
            if(c == 0x77)
            {
                int_right += 100;
                int_left += 100;
            }
            if(c == 0x73)
            {
                int_right -= 100;
                int_left -= 100;
            }
            if(c == 0x64)
            {
                int_right -= 50;
                int_left += 50;
            }
            if(c == 0x61)
            {
                int_right += 50;
                int_left -= 50;
            }
            write(kfd,&c,1);
            command = speed(int_left,int_right);
            write(fd,command.c_str(),command.size());
            ros::Duration(0.01).sleep();
        }
        tcflush(fd,TCIFLUSH);
        command = posi();
        write(fd,command.c_str(),command.size());
        write(fd1,&ce,1);
        ros::Duration(0.01).sleep();
        read(fd,temp,24);
        encode = temp;
        c_l_encode = atoi(encode.substr(2,10).c_str());
        c_r_encode = atoi(encode.substr(12,10).c_str());
        if(start == 1)
        {
            start = 0;
            p_l_encode = atoi(encode.substr(2,10).c_str());
            p_r_encode = atoi(encode.substr(12,10).c_str());
        }
        dist = ((c_l_encode - p_l_encode) + (c_r_encode - p_r_encode))*0.5*DIST;
        int a = read(fd1,temp,19);
        ch_heading[3]=temp[9];
        ch_heading[2]=temp[10];
        ch_heading[1]=temp[11];
        ch_heading[0]=temp[12];
        memcpy(&heading,ch_heading,4);
        heading = -heading;
//        std::cout<<heading<<std::endl;

        posi_x += dist*cos(heading);
        posi_y += dist*sin(heading);

        transform0.setOrigin(tf::Vector3(posi_x,posi_y,0));
        q.setRPY(0, 0, heading);
        transform0.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform0, ros::Time::now(), "/map", "/base_footprint"));

        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "map";

        //set the position

        odom.pose.pose.position.x = posi_x;
        odom.pose.pose.position.y = posi_y;
        odom.pose.pose.position.z = 0.0;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(heading);
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "/base_footprint";

        //publish the message
        odom_pub.publish(odom);


        p_l_encode = c_l_encode;
        p_r_encode = c_r_encode;
//        std::cout<<int_left<<','<<int_right<<std::endl;
//        std::cout<<c_l_encode<<','<<c_r_encode<<std::endl;

//        std::cout<< dist<< std::endl;

    }
    return(0);
}


