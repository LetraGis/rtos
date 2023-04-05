/*
 * rtos.c
 *
 *  Created on: Oct 19, 2022
 *      Author: LARL
 */

/******************************************************************************
EXTERNAL DEPENDENCIES
******************************************************************************/

#include "rtos.h"
#include "qassert.h"
#include "stm32f446xx.h"

/******************************************************************************
MODULE DEFINES
******************************************************************************/

Q_DEFINE_THIS_FILE

/******************************************************************************
MODULE TYPES
******************************************************************************/

/******************************************************************************
DECLARATION OF LOCAL FUNCTIONS
******************************************************************************/

/******************************************************************************
DEFINITION OF LOCAL VARIABLES
******************************************************************************/

/* Idle Task defined. OS_onIdle_Callback() needs to be implemented by developer
   in application software. */
OSThread idleThread;

/******************************************************************************
DEFINITION OF EXPORTED VARIABLES
******************************************************************************/

OSThread * volatile OS_Curr;
OSThread * volatile OS_Next;

/* This is an array of thread pointers. All tasks live here and the array is
   used to schedule the tasks in a Round-Robin fashion. The tasks are
   allocated to this array using the function OSThread_Start() with the
   corresponding arguments. */
OSThread *OS_Thread[32 + 1];
/* Keeps track of the number of the thread being executed. */
uint8_t OS_ThreadNum;
/* Index that keeps track of the current index on the array of threads. Since
   this RTOS is using Round-Robin Algorithm for task scheduling, all created 
   threads are stored in an array. This index ensures that we are pointing to 
   the correct thread. */
uint8_t OS_CurrIdx;
/* This bitfield is used to tell if the thread is ready to run (1U) or thread is
   blocked (0U). It is used by the scheduling function OS_Sched() to determine
   which tasks are ready to run, or if no tasks are ready, execute the Idle
   thread. */
uint32_t OS_readySet; 

/******************************************************************************
DEFINITION OF LOCAL CONSTANT DATA
******************************************************************************/

/******************************************************************************
DEFINITION OF EXPORTED CONSTANT DATA
******************************************************************************/

/******************************************************************************
MODULE FUNCTION-LIKE MACROS
******************************************************************************/

/******************************************************************************
DEFINITION OF APIs
******************************************************************************/


/*!****************************************************************************
 * @brief			Initializes RTOS kernel.
 * @details		   	Initializes RTOS by setting the priority of PendSV
 *                  exception to the lowest (which is used for context
 *                  switching) and starts the idle thread.
 * @param[in]      	void    No input parameters.
 * @return         	void    No output parameters.
 ******************************************************************************/
void main_idleThread() {
    while (1) {
        OS_onIdle_Callback();
    }
}

/*!****************************************************************************
 * @brief			Initializes RTOS kernel.
 * @details		   	Initializes RTOS by setting the priority of PendSV
 *                  exception to the lowest (which is used for context
 *                  switching) and starts the idle thread.
 * @param[in]      	void    No input parameters.
 * @return         	void    No output parameters.
 ******************************************************************************/
void OS_Init(void *stkSto, uint32_t stkSize)
{
    /* set the PendSV interrupt priority to the lowest level 0xFF */
    NVIC_SetPriority(PendSV_IRQn, 0xFFUL);
	/* Instruction above can also be accomplished by next line. */
	/* *(uint32_t volatile *)0xE000ED20 |= (0xFFU << 16); */

	/* Start Idle thread. Developer needs to provide an array for the
	   Idle thread stack as well as its size. */
    OSThread_Start(&idleThread,
                   &main_idleThread,
                   stkSto, stkSize);
}

/*!****************************************************************************
 * @brief			Implements the scheduling algorithm (Round-Robin) 
 * @details		   	Implements the scheduling algorithm using the Round-Robin
 *                  scheduling. Analyzes 
 * @param[in]      	void    No input parameters.
 * @return         	void    No output parameters.
 ******************************************************************************/
void OS_Sched(void)
{

	/* Round-Robin algorithm implementation. If the bitfield is equal to 0,
	   it means that no thread is ready to be executed. */
    if (OS_readySet == 0U) { /* idle condition? */
		/* If no thread is in ready state, the control will be passed to Idle
		   thread, which has index 0 in the array. */
        OS_CurrIdx = 0U; /* index of the idle thread */
    }
    else {
        do {
            ++OS_CurrIdx;
			/* If the current index is equal to number of started threads,
			   go back and execute the first thread (which is in this case
			   the one with index 1U, because the index 0U is reserved for Idle
			   thread). This is only executed while the corresponding bit in 
			   the bitfield is not set (it means, the thread is not ready to 
               run). */
            if (OS_CurrIdx == OS_ThreadNum) {
                OS_CurrIdx = 1U;
            }
        } while ((OS_readySet & (1U << (OS_CurrIdx - 1U))) == 0U);
    }
    /* temporary for the next thread */
    OSThread * const next = OS_Thread[OS_CurrIdx];

	if(next != OS_Curr)
	{
		OS_Next = next;
        /* Setting the corresponding bit in the Interrupt Control and State 
           Register (ICSR) of the System Control Block (SCB) peripheral to 
           trigger PendSV exception. */
		SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
	}
}

/*!****************************************************************************
 * @brief			Initializes RTOS kernel.
 * @details		   	Initializes RTOS by setting the priority of PendSV
 *                  exception to the lowest (which is used for context
 *                  switching) and starts the idle thread.
 * @param[in]      	void    No input parameters.
 * @return         	void    No output parameters.
 ******************************************************************************/
void OS_Run(void)
{
	/* Callback needs to be implemented by the developer. Here, the needed
	   initializations will be executed, as configuring system clock and
	   its interrupts, etc. */
	OS_onStartup_Callback();

    __disable_irq();
    OS_Sched();
    __enable_irq();

	Q_ERROR();
}

/*!****************************************************************************
 * @brief			Initializes RTOS kernel.
 * @details		   	Initializes RTOS by setting the priority of PendSV
 *                  exception to the lowest (which is used for context
 *                  switching) and starts the idle thread.
 * @param[in]      	void    No input parameters.
 * @return         	void    No output parameters.
 ******************************************************************************/
void OS_delay(uint32_t ticks)
{
    __disable_irq();

    /* never call OS_delay from the idleThread */
    Q_REQUIRE(OS_Curr != OS_Thread[0]);

	/* Assign the number of given ticks (for delay) to corresponding member. */
    OS_Curr->timeout = ticks;
	/* The task will be set immediately to blocked state (clearing its
	   corresponding bit). */
    OS_readySet &= ~(1U << (OS_CurrIdx - 1U));
    /* By calling the scheduler, the control will be taken from the blocked
	   task and the next available task will be executed. */
	OS_Sched();
	/* When enabling interrupts, the control will be passed to another
	   task which is ready to be executed. If no functions are ready,
	   Idle task will be executed. */
    __enable_irq();
}

/*!****************************************************************************
 * @brief			Initializes RTOS kernel.
 * @details		   	Initializes RTOS by setting the priority of PendSV
 *                  exception to the lowest (which is used for context
 *                  switching) and starts the idle thread.
 * @param[in]      	void    No input parameters.
 * @return         	void    No output parameters.
 ******************************************************************************/
void OS_Tick(void) {
    uint8_t n;
	/* Loop through all started threads except index 0 (which is reserved for
	   Idle thread). */
    for (n = 1U; n < OS_ThreadNum; ++n) {
		/* Only decrement the timeout when is greater than 0U; */
        if (OS_Thread[n]->timeout != 0U) {
            --OS_Thread[n]->timeout;
			/* If timer is expired, task is ready to run (bit in bitfield for
			   corresponding thread is set). */
            if (OS_Thread[n]->timeout == 0U) {
                OS_readySet |= (1U << (n - 1U));
            }
        }
    }
}

/*!****************************************************************************
 * @brief			Initializes RTOS kernel.
 * @details		   	Initializes RTOS by setting the priority of PendSV
 *                  exception to the lowest (which is used for context
 *                  switching) and starts the idle thread.
 * @param[in]      	void    No input parameters.
 * @return         	void    No output parameters.
 ******************************************************************************/
void OSThread_Start(
		OSThread *me,
		OSThreadHandler threadHandler,
		void *stkSto, uint32_t stkSize)
{
    /* round down the stack top to the 8-byte boundary
    * NOTE: ARM Cortex-M stack grows down from hi -> low memory
    */
    uint32_t *sp = (uint32_t *)((((uint32_t)stkSto + stkSize) / 8) * 8);
    uint32_t *stk_limit;

    *(--sp) = (1U << 24);  /* xPSR */
    *(--sp) = (uint32_t)threadHandler; /* PC */
    *(--sp) = 0x0000000EU; /* LR  */
    *(--sp) = 0x0000000CU; /* R12 */
    *(--sp) = 0x00000003U; /* R3  */
    *(--sp) = 0x00000002U; /* R2  */
    *(--sp) = 0x00000001U; /* R1  */
    *(--sp) = 0x00000000U; /* R0  */
    /* additionally, fake registers R4-R11 */
    *(--sp) = 0x0000000BU; /* R11 */
    *(--sp) = 0x0000000AU; /* R10 */
    *(--sp) = 0x00000009U; /* R9 */
    *(--sp) = 0x00000008U; /* R8 */
    *(--sp) = 0x00000007U; /* R7 */
    *(--sp) = 0x00000006U; /* R6 */
    *(--sp) = 0x00000005U; /* R5 */
    *(--sp) = 0x00000004U; /* R4 */

    /* save the top of the stack in the thread's attribute */
    me->sp = sp;

    /* round up the bottom of the stack to the 8-byte boundary */
    stk_limit = (uint32_t *)(((((uint32_t)stkSto - 1U) / 8) + 1U) * 8);

    /* pre-fill the unused part of the stack with 0xDEADBEEF */
    for (sp = sp - 1U; sp >= stk_limit; --sp) {
        *sp = 0xDEADBEEFU;
    }

	Q_ASSERT(OS_ThreadNum < Q_DIM(OS_Thread));
	
	OS_Thread[OS_ThreadNum] = me;
    /* make the thread ready to run */
    if (OS_ThreadNum > 0U) {
        OS_readySet |= (1U << (OS_ThreadNum - 1U));
    }
	++OS_ThreadNum;	
}

/*!****************************************************************************
 * @brief			Initializes RTOS kernel.
 * @details		   	Initializes RTOS by setting the priority of PendSV
 *                  exception to the lowest (which is used for context
 *                  switching) and starts the idle thread.
 * @param[in]      	void    No input parameters.
 * @return         	void    No output parameters.
 ******************************************************************************/
__attribute__ ((naked, optimize("-fno-stack-protector")))
void PendSV_Handler(void)
{
	__asm volatile (
	    /* __disable_irq(); */
	    "  CPSID         I                 \n" /* Disabling interrupts */

	    /* if (OS_Curr != (OSThread *)0) { */
	    "  LDR           r1,=OS_Curr       \n" /* Loads the address of OS_Curr into R1 */
	    "  LDR           r1,[r1,#0x00]     \n" /* Loads the value pointed to by R1, which is the address pointed to by OS_Curr */
	    "  CBZ           r1,PendSV_restore \n" /* Compares R1 to 0 to check for NULL pointers. If value is 0, will jump to PendSV_restore */

	    /*     push registers r4-r11 on the stack */
	    "  PUSH          {r4-r11}          \n" /* Pushing R4 to R11 into the stack */

	    /*     OS_Curr->sp = sp; */
	    "  LDR           r1,=OS_Curr       \n" /* Loads the address of OS_Curr into R1 */
	    "  LDR           r1,[r1,#0x00]     \n" /* Loads the value pointed to by R1, which is the address pointed to by OS_Curr */
	    "  STR           sp,[r1,#0x00]     \n" /* Stores the content of SP into R1, which is the address pointed to by OS_Curr */
	    /* } */

	    "PendSV_restore:                   \n"
	    /* sp = OS_Next->sp; */
	    "  LDR           r1,=OS_Next       \n" /* Loads the address of OS_Next into R1 */
	    "  LDR           r1,[r1,#0x00]     \n" /* Loads the value pointed to by R1, which is the address pointed to by OS_Next */
	    "  LDR           sp,[r1,#0x00]     \n" /* Loads into SP the contents of R1, which is the address pointed to by OS_Next */

	    /* OS_Curr = OS_Next; */
	    "  LDR           r1,=OS_Next       \n" /* Loads the address of OS_Next into R1 */
	    "  LDR           r1,[r1,#0x00]     \n" /* Loads the value pointed to by R1, which is the address pointed to by OS_Next */
	    "  LDR           r2,=OS_Curr       \n" /* Loads the address of OS_Curr into R2 */
	    "  STR           r1,[r2,#0x00]     \n" /* Stores the content of R1 into R2 */

	    /* pop registers r4-r11 */
	    "  POP           {r4-r11}          \n" /* Popping R4 to R11 from the stack */

	    /* __enable_irq(); */
	    "  CPSIE         I                 \n" /* Enabling interrupts */

	    /* return to the next thread */
	    "  BX            lr                \n" /* Returning from PendSV interrupt */
	    );
}
/******************************************************************************
End Of File
******************************************************************************/
