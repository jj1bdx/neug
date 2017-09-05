#ifdef GNU_LINUX_EMULATION
#define SIZE_1 4096
#define SIZE_2 4096
#define SIZE_3 4096
#else
#define SIZE_0 0x0140 /* Main */
#define SIZE_1 0x00c0 /* LED */
#define SIZE_2 0x0180 /* RNG */
#define SIZE_3 0x0140 /* USB */
#endif

#if defined(STACK_MAIN) && !defined(GNU_LINUX_EMULATION) 
/* Idle+Exception handlers */
char __main_stack_end__[0] __attribute__ ((section(".main_stack")));
char main_base[0x0080] __attribute__ ((section(".main_stack")));

/* Main program            */
char __process0_stack_end__[0] __attribute__ ((section(".process_stack.0")));
char process0_base[SIZE_0] __attribute__ ((section(".process_stack.0")));
#endif

/* First thread program    */
#if defined(STACK_PROCESS_1)
char process1_base[SIZE_1] __attribute__ ((section(".process_stack.1"))); 
#endif

/* Second thread program   */
#if defined(STACK_PROCESS_2)
char process2_base[SIZE_2] __attribute__ ((section(".process_stack.2")));
#endif

/* Third thread program    */
#if defined(STACK_PROCESS_3)
char process3_base[SIZE_3] __attribute__ ((section(".process_stack.3")));
#endif
