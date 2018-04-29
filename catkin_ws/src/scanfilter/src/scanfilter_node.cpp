#include <ros/ros.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h> /* File control definitions */
#include <errno.h>
#include "std_msgs/String.h"
#include<sensor_msgs/LaserScan.h>
#include<tf/transform_broadcaster.h>
#include <termios.h>
#include <laser_geometry/laser_geometry.h>

#include<pcl/common/transforms.h>
#include<pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include<pcl/conversions.h>
#include<pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>

int fd;
//int a = 1535;
int a = 2048;
int i = 1;

laser_geometry::LaserProjection projector;
sensor_msgs::PointCloud2 cloud;
Eigen::Matrix4f cloud_trans = Eigen::Matrix4f::Identity();

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source1 (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ> cloud_source2;
pcl::PCLPointCloud2 tem_cloud;

pcl::PointCloud<pcl::PointXYZ> cloud_save;// (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ> cloud_save2;// (new pcl::PointCloud<pcl::PointXYZ>);
pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
float pos[3] = {0};

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
    case 1000000:
        cfsetispeed(&newtio, B1000000);
        cfsetospeed(&newtio, B1000000);
    default:
        cfsetispeed(&newtio, B1000000);
        cfsetospeed(&newtio, B1000000);
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
void chatterCallback(const std_msgs::String::ConstPtr& msg)//IMU position receiving
{

  char *s = new char[strlen(msg->data.c_str())+1];
  strcpy(s, msg->data.c_str());

  const char *d = ", ";
  char *px;
  px = strtok(s,d);
  int nn = 0;
  while(px)
  {
    printf("%s\n",px);
    pos[nn] = atof(px);
    px=strtok(NULL,d);
    nn++;
    
  }
}

void scanCallback(ros::Publisher &pub, const sensor_msgs::LaserScan::ConstPtr& scan)
{
    projector.projectLaser(*scan,cloud);
    pcl_conversions::toPCL(cloud, tem_cloud);//laser to point pcl data
    pcl::fromPCLPointCloud2(tem_cloud, *cloud_source1);//PCLPointcloud2 binary to pointcloud2(pcl::PointCloud<T>)
    pcl::transformPointCloud(*cloud_source1, cloud_source2, cloud_trans);
    pcl::toPCLPointCloud2(cloud_source2, tem_cloud);
    pcl_conversions::fromPCL(tem_cloud, cloud);


    pub.publish(cloud);
    i=2;
}


int main(int argc, char** argv)
{
    int file = 0;
    char j;

  ros::init(argc, argv, "scanfilter");
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::NodeHandle nlisten;
  ros::Subscriber sublisten = nlisten.subscribe("chatter", 256, chatterCallback);

  ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("/cloud_in", 200);
  ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan> ("/scan", 1, boost::bind(&scanCallback, boost::ref(pub), _1));

  tf::TransformBroadcaster br;
  tf::Transform transform0;
  transform0.setOrigin(tf::Vector3(0.095,0,0.48));
  tf::Transform transform1;
  transform1.setOrigin(tf::Vector3(0,0,0.08));
  tf::Quaternion q;
  tf::Quaternion q1;
  cloud_trans(2,3) = 0.56;

 // ros::Rate r(10);
  int a_e = 16;

  fd = open_port(argv);
  set_opt(fd,1000000,8,'N',1);

while(n.ok())
{
    pose(a);


    q.setRPY(-(0.087890625*a-180)/57.3, 0, 0);
    //q.setRPY(0, 0, 0);
    transform1.setRotation(q);
    q1.setRPY(0, 0, 0);
    transform0.setRotation(q);
    transform1.setOrigin(tf::Vector3(pos[0],pos[1],pos[2]));//position setting
    transform0.setOrigin(tf::Vector3(pos[0],pos[1],pos[2]));//positiion setting
    //float th = -(0.087890625*a-180)/57.3;
    float th = 0;

    cloud_trans(1,1) = cos(th);
    cloud_trans(1,2) = -sin(th);
    cloud_trans(2,1) = sin(th);
    cloud_trans(2,2) = cos(th);

   // r.sleep();

  br.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "/dynamixel", "/laser"));
  while(i==1)
  ros::spinOnce();

//  system("rostopic echo -n 1 /scan/ranges >> ~/con.txt");
//  if(a>=2559)
//  if(a>=2248)
  if(a>=3072)
  {
    
      a_e=-16;
  }
  else if(a<=1024)
  {
      a_e=16; 
  }
  else if(a>=2040 && a<2056 && a_e>0)
  {
      printf("print Enter to start/continue");
      char sss;
      //scanf("%c",  &sss);
  }
  a+=a_e;
  i=1;
}
  return(0);
}
