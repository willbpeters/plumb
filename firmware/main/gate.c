#include "gate.h"

/* Atomic, because the state machine that will arm it runs on the other core. */
static bool s_armed;

void gate_arm(void)
{
    __atomic_store_n(&s_armed, true, __ATOMIC_SEQ_CST);
}

void gate_open(void)
{
    __atomic_store_n(&s_armed, false, __ATOMIC_SEQ_CST);
}

bool gate_armed(void)
{
    return __atomic_load_n(&s_armed, __ATOMIC_SEQ_CST);
}
