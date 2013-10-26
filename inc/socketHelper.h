#ifndef _SOCKETHELPER_H
#define _SOCKETHELPER_H

//#include <Python.h>
//#include "wip.h"
#include "adl_global.h"
#define MAX_ADDR_LEN (9)

//typedef u32 socklen_t;

#ifdef __REMOTETASKS__ 
typedef unsigned long u_long;
typedef unsigned char u_char;
typedef unsigned short u_short;
#endif


//Define the AF values if they aren't defined in the python code (More for the stand alone test)

//Using the definitions from the Linux book at Terry's desk
#define USING_UNIX_DEFS (1)
#ifdef USING_UNIX_DEFS
	struct sockaddr{
		short int sa_family;              /* address family */
		char sa_data[14];          /* up to 14 bytes of direct address */
	};

	struct in_addr {
		unsigned long int	s_addr;
	};

	struct sockaddr_in {
		short int	sin_family;
		unsigned short int	sin_port;
		struct in_addr	sin_addr;
	};

	struct sockaddr_un {
		short int	sun_family;
		char		sun_path[6];
	}
#ifdef __GNUC__
__attribute__((packed))
#endif
	;

#define USB_COMM_PORT 	"USB"
#define UART1_COMM_PORT "UART1"
#define UART2_COMM_PORT "UART2"

#ifndef __REMOTETASKS__ 
	#define in_addr_t u_long
#else
	typedef u_long in_addr_t;
#endif
#endif


/*These enumerations are used for the socket shutdown method*/
enum {
	SHUT_RD,
	SHUT_WR,
	SHUT_RDWR
};

/*-------- Added to make getaddrinfo.c to work (from GNU C Library) -------*/
struct servent{
	char *s_name;
	char **s_aliases;
	int s_port;
	char *s_proto;
};

struct hostent{
	char *h_name;
	char **h_aliases;
	int h_addrtype;
	int h_length;
	char **h_addr_list;
	char *h_addr;
};

struct protoent{
	char *p_name;
	char **p_aliases;
	int p_proto;
};
/*-------- Stuff added from in.h Linux (can we use this) -----------*/
#define IN_CLASSA_NSHIFT 24
#define IN_LOOPBACKNET 127

/*! \struct sockCall_s
 *  \brief	Structure for passing a socket call to a higher context */
struct sockCall_s{
	int sc_domain;
	int sc_type;
	int sc_protocol;
	int sc_returnCode;
	int sc_socketCount;
};

/*! \struct bincon_s
 *  \brief	Structure for passing connect and bind calls to a higher context */
struct bincon_s{
	int bc_socket;
	struct sockaddr *bc_address;
	int bc_address_len;
	int bc_returnCode;
};

typedef struct sockCall_s socketBuffer;
typedef struct bincon_s bindConnectBuffer;

#define  SOCKET_MSG_ID     (0x01000000)		/*!< Message ID for socket() calls */
#define  TCPSERV_MSG_ID    (0x00100000)		/*!< Message ID for bind() calls */
#define  CONNECT_MSG_ID    (0x00010000)		/*!< Message ID for connect() calls */
#define  UDPSERV_MSG_ID    (0x00001000)		/*!< Message ID for bind() calls */
/*! Offset for the init semaphores, so that the IDs don't conflict with FD numbers */
#define  INIT_SEM_OFFSET (100)			
extern int socketCount;		/*!< The number of calls to socket() */

//Socket Operations
/*---- High priority socket calls ----*/
int socket(int domain, int type, int protocol);
int connect (int socket, const struct sockaddr *address, int address_len);
int bind(int socket, const struct sockaddr *address, int address_len);
int _socket(int domain, int type, int protocol);
int _bind(int socket, const struct sockaddr *address, int address_len);
int _connect (int socket, const struct sockaddr *address, int address_len);
int recvfrom (int socket, void * buf, size_t len, int flags, struct sockaddr *from, int * fromlen);
int recv (int socket, void * buf, size_t len, int flags);
int accept(int s, struct sockaddr *addr, int *addrlen);
int socketpair(int d, int type, int protocol, int fd[2]);

/*---- Lower priority socket calls ----*/
int sendto(int socket, const void *buf, size_t len,int flags
			, const struct sockaddr *to,int tolen);

int send(int socket, const void *buf, size_t len, int flags);
int s_close(int socket);

int getsockopt (int s, int level, int optname, char * optval, int *optlen);

int setsockopt(int socket, int level, int option_name
					, const void *option_value, size_t option_len);

int shutdown(int socket,int how);

/*-------- Added Stuff for python --------*/
unsigned long int htonl(unsigned long int hostlong);
unsigned short int htons(unsigned short int hostlong);
unsigned long int ntohl(unsigned long int netlong);
unsigned short int ntohs(unsigned short int netshort);

struct servent *getservbyname(const char *name, const char *proto);
struct servent *getservbyport(int port, const char *proto);
struct hostent *gethostbyname(const char *name);
struct protent *getprotobyname(const char *name);
int getsockname(int socket, struct sockaddr *addr, socklen_t *address_len);
int getpeername(int s, struct sockaddr *name, socklen_t *namelen);

in_addr_t inet_addr(const char *cp);
char *inet_ntoa(struct in_addr in);


#endif
