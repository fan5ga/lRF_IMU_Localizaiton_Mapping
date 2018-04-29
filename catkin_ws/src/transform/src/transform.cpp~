#include"stdlib.h"
#include"fstream"
#include"iostream"
#include"cv.h"
#include "opencv2/core/core.hpp"
#include"opencv2/opencv.hpp"


using namespace std;

#define bufLen 100

static char buffer[bufLen];

int main(void)
{
	int point[12];
	int i = 0;

	ifstream file("out1.txt",ios::in);
	if(!file)return 0;
	const char *delima = ",";
	
	while(!file.eof())
	{
		file.getline(buffer,bufLen,'\n');
		char *p;
		p = strtok(buffer, delima);
		int temp;
		while(p)
		{
			temp = atoi(p);
			p=strtok(NULL,delima);
			//cout<<temp<<endl;
			point[i]=temp;
			i++;
		}
	if(i=12) break;
	//	file.getline(buffer,bufLen,'\n');
	}
	cout<<*point<<endl;
//	waitKey();
	return 0 ;
} 
