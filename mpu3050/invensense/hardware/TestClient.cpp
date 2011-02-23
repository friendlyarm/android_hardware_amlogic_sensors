/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */

/**
 *  Dummy client used for testing IPC calls and data handoff.
 */
 
#include "Impld.h"
#include <binder/MemoryHeapBase.h>
#include <binder/IServiceManager.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stddef.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/poll.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <linux/input.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <fcntl.h>

/*
 * Create a UNIX-domain socket address in the Linux "abstract namespace".
 *
 * The socket code doesn't require null termination on the filename, but
 * we do it anyway so string functions work.
 */
int makeAddr(const char* name, struct sockaddr_un* pAddr, socklen_t* pSockLen)
{
    int nameLen = strlen(name);
    if (nameLen >= (int) sizeof(pAddr->sun_path) -1)  /* too long? */
        return -1;
    pAddr->sun_path[0] = '\0';  /* abstract namespace */
    strcpy(pAddr->sun_path+1, name);
    pAddr->sun_family = AF_LOCAL;
    *pSockLen = 1 + nameLen + offsetof(struct sockaddr_un, sun_path);
    return 0;
}

int socketLoop(android::sp<android::Impld> p_mpld)
{
    struct sockaddr_un sockAddr;
    socklen_t sockLen;
    int result = 1;
    int fd;
    int token;
    struct pollfd *fds;
    int fdcount;
    struct input_event event;
    char msg[64];
    

    /* Allocate and clear out fds */
    fdcount = 2;
    fds = (struct pollfd*)calloc(fdcount, sizeof(struct pollfd));
    
    if (makeAddr("mpld1", &sockAddr, &sockLen) < 0)
        return 1;
    //connect to sensor data stream
    fd = socket(AF_LOCAL, SOCK_STREAM, PF_UNIX);
    if (fd < 0) {
        perror("client socket()");
        return 1;
    }

    printf("CLIENT %s\n", sockAddr.sun_path+1);

    if (connect(fd, (const struct sockaddr*) &sockAddr, sockLen) < 0) {
        perror("client connect()");
        goto bail;
    }
    
    read(fd, &token, sizeof(token));
    printf("client got token: %d\n", token);
    
    fds[0].fd = fd;
    fds[0].events |= POLLIN;
    
    fds[1].fd = open("/dev/input/event0", O_RDONLY);
    fds[1].events |= POLLIN;
    
    while(1) {
        int nread;
        poll(fds, fdcount, -1);

        if(fds[0].revents & POLLIN) {
            int sen, acc;
            float data[3];
            fds[0].revents = 0;
            nread = read(fds[0].fd, &sen, sizeof(int));
            nread += read(fds[0].fd, &acc, sizeof(int));
            nread += read(fds[0].fd, &data[0], sizeof(float));
            nread += read(fds[0].fd, &data[1], sizeof(float));
            nread += read(fds[0].fd, &data[2], sizeof(float));
            printf("socket input: %d bytes [%d %f %f %f]\n", nread, sen, data[0], data[1], data[2]); 
        }
        
        if(fds[1].revents & POLLIN) {
            fds[1].revents = 0;
            nread = read(fds[1].fd, &event, sizeof(event));
            printf("event input: %d %X %d\n", (int)event.type, (int)event.value, (int)event.code);
            if(event.type == 4 && event.value == 0x90001) {
                //ask for gyro data
                p_mpld->activateSensor(token, 1, 1);
            }
            if(event.type == 4 && event.value == 0x90002) {
                //turn off gyro data
                //p_mpld->activateSensor(token, 1, 0);
                close(fd);
                while(1) sleep(10);
            }
        }
    }

bail:
    close(fd);
    return result;
}
    
namespace android {


}

using namespace android;

int main(int argc, char** argv)
{
  static sp<Impld> s_mpld = 0;

  /* Get the mpl service */
  if (s_mpld == NULL)
  {
    sp<IServiceManager> sm = defaultServiceManager();
    sp<IBinder> binder;
    binder = sm->getService(String16("vendor.invensense.mpld"));
    if (binder != 0)
    {
      s_mpld = Impld::asInterface(binder);
    }
  }
  if (s_mpld == NULL)
  {
    LOGE("The mpld is not published");
    return -1; /* return an errorcode... */
  }
  
  int sock = s_mpld->getSocket();
  printf("got socket: %d\n", sock);
  
  socketLoop( s_mpld );
  
  return 0;
}


