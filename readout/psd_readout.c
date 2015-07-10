#include <errno.h>
#include <termios.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>

double alphax_neg =  -9.97302277;
double alphax_pos = -10.0618347;

double alphay_neg =  10.21630588;
double alphay_pos =  10.19731683;

double betax      =  -0.14902363;
double betay      =   0.98126698;

double theta      =   0;

int set_interface_attribs (int fd, int speed, int parity); 
void set_blocking (int fd, int should_block); 

int main(int argc, char* argv[]) {

    char *portname = "/dev/ttyACM0";
    int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        printf ("error %d opening %s: %s", errno, portname, strerror (errno));
        return;
    }

    set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (fd, 0);                    // set no blocking

    double x,y;


    while (true) {
        char buf[19]; 
        memset (buf, 0, sizeof buf); 

        // read one byte
        char tmp[] = "X";
        int n = read (fd, tmp, 1);
        if (n==1) {
            //printf("%c", tmp[0]); 
        }	
        else {
            continue; 
        }

        int count    = 0;
        if (tmp[0] == '\n') {
            int i;
            while (count < 19) {
                n = read (fd, tmp, 1); 
                if (n==1) {
                    buf[count] = tmp[0]; 
                    count+=1;
                }	
            }
            count    = 0;

            n = sscanf(buf, "%lf %lf", &x, &y); 


            if (x<0) {
                x = x*alphax_neg - betax; 
            } 
            else {
                x = x*alphax_pos - betax; 
            }

            if (y<0) {
                y = y*alphay_neg - betay; 
            } 
            else {
                y = y*alphay_pos - betay; 
            }


            printf("% 8.6f % 8.6f\n", x, y); 
        }
    }
}

int set_interface_attribs (int fd, int speed, int parity)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        printf ("error %d from tcgetattr", errno);
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
    tty.c_cc[VTIME] = 20;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    //tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag &= ~(PARENB);      // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_lflag |= ICANON; // Canonical Mode
    tty.c_iflag |= IGNCR; // Ignore carriage returns

    cfmakeraw (&tty);
    tcflush(fd,TCIFLUSH);

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        printf ("error %d from tcsetattr", errno);
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
        printf ("error %d from tggetattr", errno);
        return;
    }

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
        printf ("error %d setting term attributes", errno);
}
