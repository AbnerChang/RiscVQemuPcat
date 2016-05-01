/*
 *  QEMU RISC-V timer support
 *
 *  Author: Sagar Karandikar, skarandikar@berkeley.edu
 *  Based on the MIPS target timer support
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "hw/hw.h"
#include "hw/riscv/cpudevs.h"
#include "qemu/timer.h"
#include "riscv_pcat_board.h"

static uint64_t last_count_update;

uint64_t cpu_riscv_get_cycle (CPURISCVState *env) {
    uint64_t now;
    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    // first, convert _now_ to seconds by dividing by get_ticks_per_sec
    // and then multiply by the timer freq.
    return muldiv64(now, TIMER_FREQ, get_ticks_per_sec());
}

static void cpu_riscv_timer_update(CPURISCVState *env)
{
    uint64_t now, next;
    uint32_t diff;

    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    diff = (uint32_t)(env->helper_csr[CSR_COMPARE] - env->helper_csr[CSR_COUNT]);
    next = now + muldiv64(diff, get_ticks_per_sec(), TIMER_FREQ);
    timer_mod(env->timer, next);
}

static void cpu_riscv_machine_timer_update(CPURISCVState *env)
{
    uint64_t now, next;
    uint32_t diff;

    if (!env->MachineTimerCmpSet) {
      return;
    }

    diff = (uint32_t)(env->helper_csr[CSR_MACHINE_MTIMECMP] - env->helper_csr[CSR_MACHINE_MTIME]);
    //printf("RISCV-QEMU : MachineTimerCmpSet. Now: %ld, Expire:%ld, diff:%d\n", env->helper_csr[CSR_MACHINE_MTIME],
    //       env->helper_csr[CSR_MACHINE_MTIMECMP],
    //       diff
    //       );
    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    next = now + muldiv64(diff, get_ticks_per_sec(), MACHINE_MODE_TIMER_FREQ);
    timer_mod(env->MachineTimer, next);
}

static void cpu_riscv_timer_expire(CPURISCVState *env)
{
    cpu_riscv_timer_update(env);
    qemu_irq_raise(env->irq[7]);
}

uint32_t cpu_riscv_get_count (CPURISCVState *env)
{
    uint64_t diff;

    diff = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - last_count_update;
    return env->helper_csr[CSR_COUNT] +
        (uint32_t)muldiv64(diff, TIMER_FREQ, get_ticks_per_sec());
}

uint32_t cpu_riscv_machine_get_count (CPURISCVState *env)
{
    uint64_t diff;
    uint64_t NewClock_ns;

    NewClock_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    diff = NewClock_ns - env->LastMahineTimer_ns;
    env->LastMahineTimer_ns = NewClock_ns;
    env->helper_csr[CSR_MACHINE_MTIME] += muldiv64(diff, MACHINE_MODE_TIMER_FREQ, get_ticks_per_sec());
    return (uint32_t)env->helper_csr[CSR_MACHINE_MTIME];
}

void cpu_riscv_store_count (CPURISCVState *env, uint32_t count)
{
    /* Store new count register */
    env->helper_csr[CSR_COUNT] = count;
    last_count_update = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    /* Update timer timer */
    cpu_riscv_timer_update(env);
}

void cpu_riscv_machine_store_count (CPURISCVState *env, uint32_t count, uint32_t countH)
{
    /* Store new count register */
    env->helper_csr[CSR_MACHINE_MTIME] = count;
    env->helper_csr[CSR_MACHINE_MTIMEH] = countH;
    env->LastMahineTimer_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    /* Update timer timer */
    //cpu_riscv_machine_timer_update(env);
}

void cpu_riscv_store_compare (CPURISCVState *env, uint32_t value)
{
    env->helper_csr[CSR_COMPARE] = value;
    qemu_irq_lower(env->irq[7]);
    // according to RISCV spec, any write to compare clears timer interrupt
    cpu_riscv_timer_update(env);
}

void cpu_riscv_machine_eret_callback (CPURISCVState *env)
{
  cpu_riscv_machine_timer_update(env);
}

void cpu_riscv_machine_store_compare (CPURISCVState *env, uint32_t value)
{
    qemu_irq_lower(env->irq[MACHINE_MODE_QEMU_IRQ]); // according to RISCV spec, any write to compare clears timer interrupt    
    env->helper_csr[CSR_MACHINE_MTIMECMP] = (uint32)value;
    if (!env->MachineTimerCmpSet) {
      env->MachineTimerCmpSet = true;
      cpu_riscv_machine_timer_update(env); 

    } else {
      //
      // We update cpu_riscv_machine_timer_update in ERET callback. This prevents
      // from rapid timer happened and causes reentry.
      //
      env->MachineERetCallback = cpu_riscv_machine_eret_callback;
    }
}

static void riscv_timer_cb (void *opaque)
{
    CPURISCVState *env;
    env = opaque;

    env->helper_csr[CSR_COUNT]++;
    cpu_riscv_timer_expire(env);    
    env->helper_csr[CSR_COUNT]--;
}

static void riscv_machine_timer_cb (void *opaque)
{
    CPURISCVState *env;

    env = opaque;

    if (!env->MachineTimerCmpSet) {
        return;
    }
    //
    // Real RISC-V h/w suppose has to check below condition for generating interrtup.
    // if (env->helper_csr[CSR_MACHINE_MTIME] == env->helper_csr[CSR_MACHINE_MTIMECMP])
    // 
    // However, in QEMU, CSR_MACHINE_MTIME is only updated when read CSR_MACHINE_MTIME.
    // This timer callback is invoked when CSR_MACHINE_MTIMECMP is set.
    // 
    //env->MachineTimerCmpSet = FALSE;
    //printf("qemu riscvpcat: CSR_MACHINE_MTIME == CSR_MACHINE_MTIMECMP issue QEMU IRQ:%d\n", MACHINE_MODE_QEMU_IRQ); 
    qemu_irq_raise(env->irq[MACHINE_MODE_QEMU_IRQ]);
}

void cpu_riscv_clock_init (CPURISCVState *env)
{
    env->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, &riscv_timer_cb, env);
    env->helper_csr[CSR_COMPARE] = 0;
    cpu_riscv_store_count(env, 0);

    //
    // Initial machine timer.
    //
    env->MachineTimer = timer_new_ns(QEMU_CLOCK_VIRTUAL, &riscv_machine_timer_cb, env);
    env->MachineTimerCmpSet = false;
    //env->helper_csr [CSR_MACHINE_MTIMECMP] = 1000;// test
    cpu_riscv_machine_store_count(env, 0, 0);    
    printf("qemu riscvpcat: Inimachine_tal QEMU timer : %ld ns\n",  env->LastMahineTimer_ns);
}
