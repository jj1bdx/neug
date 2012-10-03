#define NEUG_NO_KICK      0
#define NEUG_KICK_FILLING 1

#define NEUG_PRE_LOOP 32

#define NEUG_MODE_CONDITIONED 0
#define NEUG_MODE_RAW         1
#define NEUG_MODE_RAW_DATA    2

extern uint8_t neug_mode;
extern uint16_t neug_err_cnt;
extern uint16_t neug_err_cnt_rc;
extern uint16_t neug_err_cnt_p64;
extern uint16_t neug_err_cnt_p4k;

void neug_init (uint32_t *buf, uint8_t size);
void neug_prng_reseed (void);
uint32_t neug_get (int kick);
void neug_kick_filling (void);
void neug_flush (void);
void neug_wait_full (void);
void neug_fini (void);
void neug_mode_select (uint8_t mode);
