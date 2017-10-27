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

void mpas(int fd, char* str, Mat img){
	static int lastlnum = -1;
	int lnum = atoi(str);
	if(lnum != lastlnum ){
		cout << lnum << endl;
		lastlnum = lnum;
		if(lnum < 0 ) return;
		if(lnum > img.rows-1) return;
		char out = 'Q'+((lnum+3)%4);
		write (fd,&out,1);
		for(int i=0;i<300;++i){
			uint8_t outc = 0;
			for(int j=0;j<8;++j){
				if(i*8+j >= img.cols-1) break;
				uint8_t clr = img.at<Vec3b>(Point(i*8+j,lnum))[0];
				outc = (outc << 1) | (clr>128?0:1);
			}
			write (fd,&outc,1);
			tcflush(fd, TCIOFLUSH);
			usleep(50);
		}
		char c;
		while(true){
			read (fd, &c,1);
			if(c == '\n') break;
			c = 0;
			write (fd,&c,1);
		}
	}
	line(img,Point(0,lnum),Point(3200,lnum),Scalar(255,255,0),20);
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
	char parse[30];
	int loc = -1;
	for(;;)
	{
		Mat roi;
		char buf [1000];
		int n = read (fd, buf, sizeof buf);
		for(int i=0;i<n;++i){
			switch(buf[i]){
				case '\n':
					loc = 0;
					break;
				case ',':
					parse[loc] = 0;
					loc = -1;
					out(lims).copyTo(roi);
					resize(roi,roi,Size(),0.92,1.28);	
					mpas(fd,parse,roi);
					break;
				default:{
					if(loc >=0){
						parse[loc] = buf[i];
						loc ++;
					}
				}
			}
		}
		resize(roi,dis,Size(),0.1,0.1);	
		imshow("draw", dis);
		switch(waitKey(30)){
			case 'q' : return 0;
		}
	}
	return 0;
}
