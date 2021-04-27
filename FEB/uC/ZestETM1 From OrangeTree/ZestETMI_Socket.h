//////////////////////////////////////////////////////////////////////
//
// File:      Socket.h
//
// Purpose:
//    ZestETM1 Example Programs
//    Socket layer to interface to GigExpedite device.
//
// Copyright (c) 2015 Orange Tree Technologies.
// May not be reproduced without permission.
//
//////////////////////////////////////////////////////////////////////

#ifndef SOCKET_H_
#define SOCKET_H_

// Type declarations
typedef int SOCKET;

struct in_addr {
    unsigned long s_addr;
};

struct sockaddr {
    unsigned short sa_family;
    char sa_data[14];
}; 

struct sockaddr_in {
    short sin_family;
    unsigned short sin_port;
    struct in_addr sin_addr;
    char sin_zero[8];
};

#define LISTENQ 0

#define AF_INET 0

#define SOCK_STREAM 0
#define SOCK_DGRAM 1

#define IPPROTO_UDP 0
#define IPPROTO_TCP 1

#define INVALID_SOCKET -1
#define SOCKET_ERROR 1

#define htonl(x) (x)
#define htons(x) (x)

#define INADDR_ANY 0

// API
void SocketInit(void);
SOCKET socket(int af, int type, int protocol);
int bind(SOCKET s, const struct sockaddr *name, int namelen);
int listen(SOCKET s, int backlog);
SOCKET accept(SOCKET s, struct sockaddr *addr, int *addrlen);
int connect(SOCKET s, const struct sockaddr *addr, int addrlen); 
int send(SOCKET s, const char *buf, int len, int flags);
int recv(SOCKET s, char *buf, int len, int flags);
int sendto(SOCKET s, const char *buf, int len, int flags,
           const struct sockaddr *toaddr, int tolen);
int recvfrom(SOCKET s, char *buf, int len, int flags,
             const struct sockaddr *fromaddr, int *fromlen);
int socket_close(SOCKET s);
void SocketPause(SOCKET s, int p);
unsigned long SocketGetLocalIPAddr(void);
void SocketISR(void);

#endif /*SOCKET_H_*/
