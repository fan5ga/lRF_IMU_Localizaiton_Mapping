#include <ros/ros.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h> /* File control definitions */
#include <errno.h>
#include <termios.h>

  int fd;
void pose(int data)
{
    int i = 0;
    char high0, low0;
    high0 = 0;
    low0 = 0;
    char high1, low1;
    high1 = 0;
    low1 = 0;
    low0 |= data;
    low1 = (4095-data)|low1;
    high1 = ((4095-data)>>8)|high1;
    data = data>>8;
    high0 |= data;
    char s[18]={0xff,0xff,0xfe,0x0e,0x83,0x1e,0x04,0x00,0x00,0x00,0xff,0x03,0x01,0x00,0x00,0xff,0x03,0x00};
    s[8]=low0;
    s[9]=high0;
    s[13]=low1;
    s[14]=high1;

    for(i=2;i<17;i++)
        s[17]+=s[i];
    s[17]=~s[17];

    write(fd, s, 18);

//    for(i=0;i<18;i++)
//        Serial3.write(s[i]);
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
    ros::Time::init();

    int data = 2048;
//    char dd[3]="12";
  fd = open_port(argv);
  set_opt(fd,57600,8,'N',1);

  pose(data);
  ros::Duration(0.5).sleep();
  //  write(fd, dd, 3);

  return(0);
}


