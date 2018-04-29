#include"cv.h"
#include "opencv2/core/core.hpp"
#include"opencv2/opencv.hpp"
#include"stdlib.h"
#include"fstream"
#include"iostream"


using namespace std;
using namespace cv;

#define BUFLEN 10000
#define ROW 69
#define COL 512

static char buffer[BUFLEN];

int main(void)
{

    double data[ROW][COL];

    ifstream file("data0.txt",ios::in);
    if(!file)return 0;

    int row=0;
    const char *delima = "[, ]";

    while(!file.eof())
    {
        file.getline(buffer,BUFLEN,'\n');
        char *p;
        p = strtok(buffer, delima);
        int col = 0;
        while(p)
        {
            data[ROW-1-row][COL-1-col] = atof(p);
//            data[1][1] = temp;
            p=strtok(NULL,delima);
            col++;
            //cout<<temp<<endl;
        }
        file.getline(buffer,BUFLEN,'\n');

        cout<<row<<endl;
        row++;

    }
//    file.close();

//    for(int i = 0; i++; i<ROW)
//    {
//        cout<<data[i][0]<<endl;

//    }
    return 0;
}

