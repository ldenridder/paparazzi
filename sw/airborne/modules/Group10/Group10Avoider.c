#include "modules/project/Group10Avoider.h"
#include "subsystems/abi.h"

static abi_event allowable_distance_ev;
static void allowable_distance_cb(uint8_t __attribute__((unused)) sender_id,
                               int32_t __attribute__((unused)) allowableDistance)
{
	navInput = allowableDistance; //Pointer to a vector containing the allowable distances in each lane
}

void avoiderInit(void){
	AbiBindMsgNAVIGATION_VECTOR(NAVIGATION_VECTOR_ID, &allowable_distance_ev, allowable_distance_cb);
}

void avoiderPeriodic(void){
	
}

