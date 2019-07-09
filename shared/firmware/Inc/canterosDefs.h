#ifndef CANTEROS_DEFS_H_INCLUDED
#define CANTEROS_DEFS_H_INCLUDED

#define NODE_STATUS_HEADER 14

#define HEALTH_OK          0
#define HEALTH_WARNING     1
#define HEALTH_ERROR       2
#define HEALTH_CRITICAL    3

#define MODE_OPERATIONAL       0
#define MODE_INITIALIZATION    1
#define MODE_MAINTENANCE       2
#define MODE_SOFTWARE_UPDATE   3
#define MODE_STANDBY           4
#define MODE_OFFLINE           7

typedef struct {
	uint32_t uptimeSeconds;
	uint8_t health; //2 bits
	uint8_t mode; //3 bits
	uint8_t submode; //3 bits
	uint8_t deviceSpecificCode;
} NodeStatusMessage;

#endif //CANTEROS_DEFS_H_INCLUDED
