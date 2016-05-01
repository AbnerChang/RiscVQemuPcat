#if !defined (__RISCV_CPU_H__)
#define __RISCV_CPU_H__

//#define DEBUG_OP

#define TARGET_HAS_ICE 1

#define ELF_MACHINE	EM_RISCV

#define CPUArchState struct CPURISCVState

#define RISCV_START_PC 0xffffffffffffff00

#include "config.h"
#include "qemu-common.h"
#include "riscv-defs.h"
#include "exec/cpu-defs.h"

#define NB_MMU_MODES 2

#define RISCV_PRIVILEGE_MODE_MASK       0x300
#define RISCV_PRIVILEGE_MODE_SHIFT      8
#define RISCV_USER_MODE_PRIVILEGE       0x0
#define RISCV_SUPERVISOR_MODE_PRIVILEGE 0x1
#define RISCV_HYPERVISOR_MODE_PRIVILEGE 0x2
#define RISCV_MACHINE_MODE_PRIVILEGE    0x3

struct CPURISCVState;

// RISCV CSR mappings. These are not the "real" mappings defined by the isa.
// Instead, they are the indices into our csr array (ie the output given when
// calling translate.c:csr_regno(REAL_CSR_REGNO)).
#define CSR_SUP0       0x0
#define CSR_SUP1       0x1
#define CSR_EPC        0x2
#define CSR_BADVADDR   0x3
#define CSR_PTBR       0x4
#define CSR_ASID       0x5
#define CSR_COUNT      0x6
#define CSR_COMPARE    0x7
#define CSR_EVEC       0x8
#define CSR_CAUSE      0x9
#define CSR_STATUS     0xa
#define CSR_HARTID     0xb
#define CSR_IMPL       0xc
#define CSR_FATC       0xd
#define CSR_SEND_IPI   0xe
#define CSR_CLEAR_IPI  0xf
#define CSR_CYCLE     0x10
#define CSR_TIME      0x11
#define CSR_INSTRET   0x12
#define CSR_FFLAGS    0x13
#define CSR_FRM       0x14
#define CSR_FCSR      0x15
//...
#define CSR_TOHOST    0x1e
#define CSR_FROMHOST  0x1f

//
// For machine mode CSR.
//
#define CSR_MACHINE_MSTATUS   0x300
// MSTATUS definition.
  #define CSR_MACHINE_MODE_PRIVILEGE_STACK_MASK       0xfff
  #define CSR_MACHINE_MODE_PRIVILEGE_STACK_PUSH_SHIFT 3

  #define CSR_MACHINE_MSTATUS_IE_MASK  0x1
  #define CSR_MACHINE_MSTATUS_IE_POS  0x0
    #define CSR_MACHINE_MSTATUS_IE_ENABLE 0x1
    #define CSR_MACHINE_MSTATUS_IE_DISABLE 0x0
  #define CSR_MACHINE_MSTATUS_PRV_POS  0x1
  #define CSR_MACHINE_MSTATUS_PRV_MASK (0x3 << CSR_MACHINE_MSTATUS_PRV_POS)

#define CSR_MACHINE_MTVEC     0x301
#define CSR_MACHINE_MTDELEG   0x302
#define CSR_MACHINE_MIE       0x304
  #define CSR_MACHINE_MIE_TIMER 0x80
  #define CSR_MACHINE_MIE_TIMER_MASK 0x80
#define CSR_MACHINE_MTIMECMP  0x321
#define CSR_MACHINE_MSCRATCH  0x340
#define CSR_MACHINE_MIP       0x344
  #define CSR_MACHINE_MIP_TIMER 0x80
  #define CSR_MACHINE_MIP_TIMER_MASK 0x80
#define CSR_MACHINE_MTIME     0x701
#define CSR_MACHINE_MTIMEH    0x741

// RISCV Exception Codes
#define EXCP_NONE                       -1   // not a real RISCV exception code
#define RISCV_EXCP_INST_ADDR_MIS        0x0
#define RISCV_EXCP_INST_ACCESS_FAULT    0x1
#define RISCV_EXCP_ILLEGAL_INST         0x2
#define RISCV_EXCP_PRIV_INST            0x3
#define RISCV_EXCP_FP_DISABLED          0x4
#define RISCV_EXCP_SCALL                0x6
#define RISCV_EXCP_BREAK                0x7
#define RISCV_EXCP_LOAD_ADDR_MIS        0x8
#define RISCV_EXCP_STORE_ADDR_MIS       0x9
#define RISCV_EXCP_LOAD_ACCESS_FAULT    0xa
#define RISCV_EXCP_STORE_ACCESS_FAULT   0xb
#define RISCV_EXCP_STORE_ACCEL_DISABLED 0xc
#define RISCV_EXCP_TIMER_INTERRUPT      (0x7 | (1 << 31)) 
#define RISCV_EXCP_HOST_INTERRUPT       (0x6 | (1 << 31)) 
#define RISCV_EXCP_SERIAL_INTERRUPT     (0x4 | (1 << 31)) // not part of ISA

// RISCV Status Reg Bits
#define SR_S           0x1
#define SR_PS          0x2
#define SR_EI          0x4
#define SR_PEI         0x8
#define SR_EF         0x10
#define SR_U64        0x20
#define SR_S64        0x40
#define SR_VM         0x80
#define SR_EA        0x100
#define SR_IM     0xFF0000
#define SR_IP   0xFF000000

// RISCV pte bits
#define PTE_V    0x1
#define PTE_T    0x2
#define PTE_G    0x4
#define PTE_UR   0x8
#define PTE_UW  0x10
#define PTE_UX  0x20
#define PTE_SR  0x40
#define PTE_SW  0x80
#define PTE_SX 0x100

typedef struct riscv_def_t riscv_def_t;

typedef struct TCState TCState;
struct TCState {
    target_ulong gpr[32];
    target_ulong fpr[32];
    target_ulong PC;
};

typedef struct CPURISCVState CPURISCVState;
struct CPURISCVState {
    TCState active_tc;
    TCState saved_active_tc;
    uint32_t current_tc;
    uint32_t SEGBITS;
    uint32_t PABITS;
    target_ulong SEGMask;
    target_ulong PAMask;

    uint64_t helper_csr[0x1000]; // RISCV CSR registers

    /* QEMU */
    CPU_COMMON

    /* Fields from here on are preserved across CPU reset. */
    const riscv_def_t *cpu_model;
    void *irq[8];
    QEMUTimer *timer; /* Internal timer */
    QEMUTimer *MachineTimer; /* Internal timer */
    uint64_t LastMahineTimer_ns;
    bool MachineTimerCmpSet;
    void (*MachineERetCallback)(CPURISCVState *evn);
};

#include "cpu-qom.h"

#if !defined(CONFIG_USER_ONLY)
void riscv_cpu_unassigned_access(CPUState *cpu, hwaddr addr,
                                bool is_write, bool is_exec, int unused,
                                unsigned size);
#endif

void riscv_cpu_list (FILE *f, fprintf_function cpu_fprintf);

#define cpu_exec cpu_riscv_exec
#define cpu_gen_code cpu_riscv_gen_code
#define cpu_signal_handler cpu_riscv_signal_handler
#define cpu_list riscv_cpu_list

extern void cpu_wrdsp(uint32_t rs, uint32_t mask_num, CPURISCVState *env);
extern uint32_t cpu_rddsp(uint32_t mask_num, CPURISCVState *env);

int riscv_cpu_get_current_mode (CPURISCVState *env);

#define CPU_SAVE_VERSION 3

static inline int cpu_mmu_index (CPURISCVState *env)
{
    return env->helper_csr[CSR_STATUS] & SR_S;
}

static inline int cpu_riscv_hw_interrupts_pending(CPURISCVState *env)
{
    int32_t pending;
    int32_t status;
    int r;

    r = 0;
    /* first check if interrupts are disabled */
    if (riscv_cpu_get_current_mode (env) == RISCV_SUPERVISOR_MODE_PRIVILEGE) {
        //if (!((env->helper_csr[CSR_STATUS] >> 2) & 0x1)) {
        //    // interrupts disabled
        //    return 0;
        //}
        //pending = (env->helper_csr[CSR_STATUS] >> 24) & 0xFF;
        //status = (env->helper_csr[CSR_STATUS] >> 16) & 0xFF;
        //r = pending & status;
        if (((env->helper_csr[CSR_STATUS] >> 2) & 0x1)) {
            // interrupts enabled
            pending = (env->helper_csr[CSR_STATUS] >> 24) & 0xFF;
            status = (env->helper_csr[CSR_STATUS] >> 16) & 0xFF;
            r = pending & status;
            return r;
        }   
    } else if (riscv_cpu_get_current_mode (env) == RISCV_MACHINE_MODE_PRIVILEGE) {
        /*check machine mode*/
        if (env->helper_csr[CSR_MACHINE_MSTATUS] & CSR_MACHINE_MSTATUS_IE_MASK) {
            r = (int)(env->helper_csr[CSR_MACHINE_MIP] & env->helper_csr[CSR_MACHINE_MIE]); 
        }
    }
    return r;
}

#include "exec/cpu-all.h"

/* Memory access type :
 * may be needed for precise access rights control and precise exceptions.
 */
enum {
    /* 1 bit to define user level / supervisor access */
    ACCESS_USER  = 0x00,
    ACCESS_SUPER = 0x01,
    /* 1 bit to indicate direction */
    ACCESS_STORE = 0x02,
    /* Type of instruction that generated the access */
    ACCESS_CODE  = 0x10, /* Code fetch access                */
    ACCESS_INT   = 0x20, /* Integer load/store access        */
    ACCESS_FLOAT = 0x30, /* floating point load/store access */
};

int cpu_riscv_exec(CPURISCVState *s);
void riscv_tcg_init(void);
RISCVCPU *cpu_riscv_init(const char *cpu_model);
int cpu_riscv_signal_handler(int host_signum, void *pinfo, void *puc);
void riscv_save_gpr (CPURISCVState *env);
void riscv_restore_gpr (CPURISCVState *env);

static inline CPURISCVState *cpu_init(const char *cpu_model)
{
    RISCVCPU *cpu = cpu_riscv_init(cpu_model);
    if (cpu == NULL) {
        return NULL;
    }
    return &cpu->env;
}

/* TODO QOM'ify CPU reset and remove */
void cpu_state_reset(CPURISCVState *s);

/* hw/riscv/cputimer.c */
uint64_t cpu_riscv_get_cycle (CPURISCVState *env);
uint32_t cpu_riscv_get_random (CPURISCVState *env);
uint32_t cpu_riscv_get_count (CPURISCVState *env);
void cpu_riscv_store_count (CPURISCVState *env, uint32_t value);
void cpu_riscv_store_compare (CPURISCVState *env, uint32_t value);
void cpu_riscv_start_count(CPURISCVState *env);

void cpu_riscv_machine_store_count (CPURISCVState *env, uint32_t count, uint32_t countH);
uint32_t cpu_riscv_machine_get_count (CPURISCVState *env);
void cpu_riscv_machine_store_compare (CPURISCVState *env, uint32_t value);
void cpu_riscv_machine_eret_callback (CPURISCVState *env);

/* hw/riscvpcat/riscv_int.c */
void cpu_riscv_soft_irq(CPURISCVState *env, int irq, int level);
qemu_irq * cpu_riscvpcat_irq_init_cpu(CPURISCVState *env);

/* helper.c */
int riscv_cpu_handle_mmu_fault(CPUState *cpu, vaddr address, int rw,
                              int mmu_idx);
#if !defined(CONFIG_USER_ONLY)
hwaddr cpu_riscv_translate_address (CPURISCVState *env, target_ulong address,
		                               int rw);
#endif

static inline void cpu_get_tb_cpu_state(CPURISCVState *env, target_ulong *pc,
                                        target_ulong *cs_base, int *flags)
{
    *pc = env->active_tc.PC;
    *cs_base = 0;
    *flags = 0; // necessary to avoid compiler warning
}

void riscv_cpu_dump(CPUState *cs);

#include "exec/exec-all.h"

#endif /* !defined (__RISCV_CPU_H__) */
