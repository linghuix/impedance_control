#ifndef	_M8128FORCECOLLECTOR_H_
#define	_M8128FORCECOLLECTOR_H_


#include "BSP.h"

#include "func_can.h"
#include "debug.h"



#define ForceBufferSize 20

struct ForceBuffer{
	float data[ForceBufferSize];
	uint16_t in;
	uint16_t out;
};

void ForceCollector_Init (void);
void StartCollect (void);
void StopCollect (void);
float getCurrentForce(void);
extern void forceDispatch(CanRxMsg * ForceData);
	
TEST ForceCollector_test(void);
TEST SendString_test(void);

#endif	//_APP_H_
