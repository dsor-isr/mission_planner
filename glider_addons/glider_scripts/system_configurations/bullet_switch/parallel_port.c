/* After compiling execute:
 * sudo chown 0.0 bin/Leaks
 * sudo chmod ug+s bin/Leaks
 */

#include <stdio.h>
#include <sys/io.h>
#include <stdlib.h>

/*
 * Parallel port pins and bits of the water leak detectors.
 *
 * DB25    Leaks   Parallel port
 * ----------------------------------
 * 9       VCC     Output data bit 7
 * 13      Hull    Input status bit 4
 * 18-25   Ground  Ground
 */
#define LEAK1	0x10
#define LEAK2	0x20
/*
 * Parallel port I/O addresses.
 */
#define DATA	0x378 
#define STATUS	(DATA+1)


/*
 * Bullet Swith Output
 */
#define INTERNALPICO	0x10
#define EXTERNALPICO	0x20
// bit 8 (Vcc Leak 2)
// bit 7 (Vcc Leak 1)
// bit 6 external pico
// bit 5 internal pico
// bit 4 Vcc picos
// bit 2 ssd

/*
 * SSD Swith Output
 */
#define SSDON   0x02

/*void PicoCallback(const std_msgs::Bool &ptr){
    if(ptr.data == true)
        outb(0xC8 | INTERNALPICO, DATA);
    else
        outb(0xC8 | EXTERNALPICO, DATA);
}*/

int main(int argc, char **argv)
{
    int internal=-1;
    int ssd=0;

    if(argc>=2 && strcasecmp(argv[1], "internal")==0)
        internal = 1;
    else if(argc>=2 && strcasecmp(argv[1], "external")==0)
        internal=0;
    else{
        printf("%s [internal|external] [ssdon|ssdoff]\n",argv[0]);
        return -1;
    }

    if(argc==3 && strcasecmp(argv[2], "ssdon")==0)
        ssd=1;
    else if(argc==3 && strcasecmp(argv[2], "ssdoff")==0)
        ssd=0;
    else{
        printf("%s [internal|external] [ssdon|ssdoff]\n",argv[0]);
        return -1;
    }

    // Check permissions
    if (ioperm(DATA, 1, 1)){
        printf("You must be sudo\n");
        return -1;
    }
    
    if(!ssd)
        if(system("sudo umount /media/ssd")==-1)
            printf("Unable to unmount external drive!");
    
    outb(0xC8 | (internal?INTERNALPICO:EXTERNALPICO) | (ssd?SSDON:0x00), DATA);

    printf("%s bullet was activated\n", internal?"internal":"external");

	return 0;
}
