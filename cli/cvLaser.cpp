#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <cstdlib>
#include <iostream>
#include <ctime>
#include <list>
#include <algorithm>
#include <iterator>


using namespace cv;
using namespace std;

#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>


int set_interface_attribs (int fd, int speed, int parity)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        return -1;
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        return -1;
    }
    return 0;
}

void set_blocking (int fd, int should_block)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        return ;
    }

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout
    if (tcsetattr (fd, TCSANOW, &tty) != 0)
        cout << errno << "failed" << endl;

}

vector<uint8_t> tStream;

void initProcess(Mat img){
    uint8_t outc = 0;
    for(uint16_t i=0;i>img.rows;++i){
        for(uint16_t j=0;j<img.cols;++j){
            if(j%7 == 0) outc = 0;
            uint8_t clr = img.at<Vec3b>(Point(j,i))[0];
            outc = (outc << 1) | (clr>128?0:1);
            if(j%7 == 6){
                tStream.push_back(outc);
            }
        }
        if(img.cols%7 < 6)
            outc = outc << (6-(img.cols%7));
        tStream.push_back(outc);
        tStream.push_back(0x80|0x00|((i>>0)&0x0F));
        tStream.push_back(0x80|0x10|((i>>1)&0x0F));
        tStream.push_back(0x80|0x20|((i>>2)&0x0F));
        tStream.push_back(0x80|0x30|((i>>3)&0x0F));
    }
}

void process(int fd,vector<int> inputData){
    static uint16_t head = 0;
    cout << inputData[0] << "," << inputData[1] << "," << inputData[2] << endl;
    if(inputData[0] > 120){
        uint8_t data[60];
        for(int i=0;i<60;++i){
            if(head >= tStream.size()) break;
            data[i] = tStream[head++];
        }
        while(write(fd,"T",1) < 1);
        uint16_t len = 0;
        while(len < 60) len += write(fd,&(data[len]),60-len);
        tcflush(fd, TCIOFLUSH);
    }
}

int main(int argc, char** argv)
{

    int fd = open ("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        return -1;
    }

    set_interface_attribs (fd, B460800, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (fd, 0);                // set no blocking


    Mat out = imread(argv[1],CV_LOAD_IMAGE_COLOR);
    Mat mrow,mcol;
    Rect lims(0,0,out.cols,out.rows);
    reduce(out,mrow,0,CV_REDUCE_MIN);
    reduce(out,mcol,1,CV_REDUCE_MIN);
    for(int i=mrow.cols-1;i>=0;--i){
        if( mrow.at<Vec3b>(0,i)[0] < 128){
            cout << i << endl;
            lims.width=i;
            break;
        }
    }
    for(int i=0;i<mrow.cols;++i){
        if( mrow.at<Vec3b>(0,i)[0] < 128){
            cout << i << endl;
            lims.x = i;
            lims.width-=i;
            break;
        }
    }
    for(int i=mcol.rows-1;i>=0;--i){
        if( mcol.at<Vec3b>(i,0)[0] < 128){
            cout << i << endl;
            lims.height=i;
            break;
        }
    }
    for(int i=0;i<mcol.rows;++i){
        if(mcol.at<Vec3b>(i,0)[0] < 128){
            cout << i << endl;
            lims.y = i;
            lims.height-= i;
            break;
        }
    }
    cout << lims << endl;
    Mat dis;
    Mat roi;
    vector<int> inputData(10);
    char parse[10];
    int loc = -1;
    int dloc = 0;
    out(lims).copyTo(roi);
    resize(roi,roi,Size(),0.92,1.28);
    initProcess(roi);
    for(;;)
    {
        char buf [100];
        int n = read (fd, buf, sizeof buf);
        int ncnt = 0;
        for(int i=n-30;i<n;++i){
            switch(buf[i]){
                case '\n':
                    parse[loc] = 0;
                    loc = 0;
                    inputData[dloc++] = atoi(parse);
                    loc = 0;
                    dloc = 0;
                    ncnt ++;
                    break;
                case ',':
                    parse[loc] = 0;
                    loc = 0;
                    inputData[dloc++] = atoi(parse);
                    break;
                default:{
                            if(loc >=0){
                                parse[loc++] = buf[i];
                            }
                        }
            }
        }
        if(ncnt > 0)
            process(fd,inputData);
        resize(roi,dis,Size(),0.1,0.1);	
        imshow("draw", dis);
        switch(waitKey(30)){
            case 'q' : return 0;
            case 'h' : write(fd,"h",1);
        }
    }
    return 0;
}
