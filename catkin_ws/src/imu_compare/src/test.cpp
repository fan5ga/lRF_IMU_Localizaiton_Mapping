#include <ros/ros.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h> /* File control definitions */
#include <errno.h>
#include <termios.h>
#include <iostream>
#include <fstream>

int fd1; /* File descriptor for the port */
int fd2;


int open_port(char* argv)
{
    int fd;
    fd = open(argv, O_RDWR|O_NOCTTY|O_NDELAY);
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


int main(int argc, char** argv)
{
    std::ofstream file1(argv[2],std::ios::app);
    std::ofstream file2(argv[4],std::ios::app);



    file1.close();
    file2.close();


    char cb = 0xcb;

    char c;

    float acc[3];
    float ang[3];
    float mag[3];

    char ch_acc[3][4];
    char ch_ang[3][4];
    char ch_mag[3][4];



    fd1 = open_port(argv[1]);
//    fd1 = open_port("/dev/ttyUSB0");
//    fd2 = open_port("/dev/ttyACM0");
    fd2 = open_port(argv[3]);
    set_opt(fd1,115200,8,'N',1);
    set_opt(fd2,115200,8,'N',1);

    while(1)
    {
        char temp1[70]={0};
        do read(fd1,&c,1);
        while(c != '*');
        int i = 0;
        for(i=0; c!='\n'; i++)
        {
            while (read(fd1,&c,1)==0);
            temp1[i] = c;
        }
        std::cout<<temp1<<std::endl;
        file1.open(argv[2],std::ios::app);
        file1<<temp1;
        file1.close();

        char temp2[43];
        write(fd2,&cb,1);
        read(fd2,temp2,43);

        for(int i = 0; i<4; i++)
        {
            ch_acc[0][i] = temp2[4-i];
        }
        memcpy(&acc[0],ch_acc[0],4);
        for(int i = 0; i<4; i++)
        {
            ch_acc[1][i] = temp2[8-i];
        }
        memcpy(&acc[1],ch_acc[1],4);
        for(int i = 0; i<4; i++)
        {
            ch_acc[2][i] = temp2[12-i];
        }
        memcpy(&acc[2],ch_acc[2],4);


        for(int i = 0; i<4; i++)
        {
            ch_ang[0][i] = temp2[16-i];
        }
        memcpy(&ang[0],ch_ang[0],4);
        for(int i = 0; i<4; i++)
        {
            ch_ang[1][i] = temp2[20-i];
        }
        memcpy(&ang[1],ch_ang[1],4);
        for(int i = 0; i<4; i++)
        {
            ch_ang[2][i] = temp2[24-i];
        }
        memcpy(&ang[2],ch_ang[2],4);


        for(int i = 0; i<4; i++)
        {
            ch_mag[0][i] = temp2[28-i];
        }
        memcpy(&mag[0],ch_mag[0],4);
        for(int i = 0; i<4; i++)
        {
            ch_mag[1][i] = temp2[32-i];
        }
        memcpy(&mag[1],ch_mag[1],4);
        for(int i = 0; i<4; i++)
        {
            ch_mag[2][i] = temp2[36-i];
        }
        memcpy(&mag[2],ch_mag[2],4);

        std::cout<<acc[0]<<','<<acc[1]<<','<<acc[2]<<','<<ang[0]<<','<<ang[1]<<','<<ang[2]<<','<<mag[0]<<','<<mag[1]<<','<<mag[2]<<std::endl;
        file2.open(argv[4],std::ios::app);
        file2<<acc[0]<<','<<acc[1]<<','<<acc[2]<<','<<ang[0]<<','<<ang[1]<<','<<ang[2]<<','<<mag[0]<<','<<mag[1]<<','<<mag[2]<<std::endl;
        file2.close();
    }
    return(0);
}



