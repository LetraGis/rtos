/*
 * rtos.h
 *
 *  Created on: Oct 19, 2022
 *      Author: LARL
 */

#ifndef RTOS_H_
#define RTOS_H_

/******************************************************************************
EXTERNAL DEPENDENCIES
******************************************************************************/

#include <stdint.h>

/******************************************************************************
DEFINITION OF CONSTANTS
******************************************************************************/

/******************************************************************************
DECLARATION OF TYPES
******************************************************************************/

/* Thread Control Block (TCB) */
typedef struct
{
	void *sp; /* Stack Pointer */
	uint32_t timeout; /* timeout delay down-counter */
} OSThread;

typedef void (*OSThreadHandler)();

/******************************************************************************
DECLARATION OF VARIABLES
******************************************************************************/

extern OSThread * volatile OS_Curr;
extern OSThread * volatile OS_Next;

/* This is an array of thread pointers. All tasks live here and the array is
   used to schedule the tasks in a Round-Robin fashion. The tasks are
   allocated to this array using the function OSThread_Start() with the
   corresponding arguments. */
extern OSThread *OS_Thread[32 + 1];
/* Keeps track of the number of the thread being executed. */
extern uint8_t OS_ThreadNum;
/* Index that keeps track of the current index on the array of threads. Since
   this RTOS is using Round-Robin Algorithm for task scheduling, all created 
   threads are stored in an array. This index ensures that we are pointing to 
   the correct thread. */
extern uint8_t OS_CurrIdx;
/* This bitfield is used to tell if the thread is ready to run (1U) or thread is
   blocked (0U). It is used by the scheduling function OS_Sched() to determine
   which tasks are ready to run, or if no tasks are ready, execute the Idle
   thread. */
extern uint32_t OS_readySet; 

/******************************************************************************
DECLARATION OF CONSTANT DATA
******************************************************************************/

/******************************************************************************
DECLARATION OF FUNCTIONS
******************************************************************************/

void OS_Init(void *stkSto, uint32_t StkSize);
void OS_Sched(void);
void OS_Run(void);
void OS_delay(uint32_t ticks);
/* process all timeouts */
void OS_Tick(void);
/* Needs to be implemented by developer in application SW. */
void OS_onStartup_Callback(void);
/* Needs to be implemented by developer in application SW. */
void OS_onIdle_Callback(void);
void OSThread_Start(
		OSThread *me,
		OSThreadHandler threadHandler,
		void *stkSto, uint32_t stkSize);

/******************************************************************************
DECLARATION OF FUNCTION-LIKE MACROS
******************************************************************************/

/******************************************************************************
End Of File
******************************************************************************/

#endif /* RTOS_H_ */
