// SPDX-License-Identifier: GPL-2.0
/*
 * Exception handling code
 *
 * Copyright (C) 2019 ARM Ltd.
 */

#include <linux/context_tracking.h>
#include <linux/ptrace.h>
#include <linux/thread_info.h>

#include <asm/cpufeature.h>
#include <asm/daifflags.h>
#include <asm/esr.h>
#include <asm/exception.h>
#include <asm/kprobes.h>
#include <asm/mmu.h>
#include <asm/sysreg.h>

static DEFINE_PER_CPU(int, dbg_count);
/*
 * This is intended to match the logic in irqentry_enter(), handling the kernel
 * mode transitions only.
 */
static void noinstr enter_from_kernel_mode(struct pt_regs *regs)
{
	regs->exit_rcu = false;

	if (!IS_ENABLED(CONFIG_TINY_RCU) && is_idle_task(current)) {
		lockdep_hardirqs_off(CALLER_ADDR0);
		rcu_irq_enter();
		trace_hardirqs_off_finish();

		regs->exit_rcu = true;
		return;
	}

	lockdep_hardirqs_off(CALLER_ADDR0);
	rcu_irq_enter_check_tick();
	trace_hardirqs_off_finish();
}

/*
 * This is intended to match the logic in irqentry_exit(), handling the kernel
 * mode transitions only, and with preemption handled elsewhere.
 */
static void noinstr exit_to_kernel_mode(struct pt_regs *regs)
{
	lockdep_assert_irqs_disabled();

	if (interrupts_enabled(regs)) {
		if (regs->exit_rcu) {
			trace_hardirqs_on_prepare();
			lockdep_hardirqs_on_prepare(CALLER_ADDR0);
			rcu_irq_exit();
			lockdep_hardirqs_on(CALLER_ADDR0);
			return;
		}

		trace_hardirqs_on();
	} else {
		if (regs->exit_rcu)
			rcu_irq_exit();
	}
}

void noinstr arm64_enter_nmi(struct pt_regs *regs)
{
	regs->lockdep_hardirqs = lockdep_hardirqs_enabled();

	__nmi_enter();
	lockdep_hardirqs_off(CALLER_ADDR0);
	lockdep_hardirq_enter();
	rcu_nmi_enter();

	trace_hardirqs_off_finish();
	ftrace_nmi_enter();
}

void noinstr arm64_exit_nmi(struct pt_regs *regs)
{
	bool restore = regs->lockdep_hardirqs;

	ftrace_nmi_exit();
	if (restore) {
		trace_hardirqs_on_prepare();
		lockdep_hardirqs_on_prepare(CALLER_ADDR0);
	}

	rcu_nmi_exit();
	lockdep_hardirq_exit();
	if (restore)
		lockdep_hardirqs_on(CALLER_ADDR0);
	__nmi_exit();
}

asmlinkage void noinstr enter_el1_irq_or_nmi(struct pt_regs *regs)
{
	if (IS_ENABLED(CONFIG_ARM64_PSEUDO_NMI) && !interrupts_enabled(regs))
		arm64_enter_nmi(regs);
	else
		enter_from_kernel_mode(regs);
}

asmlinkage void noinstr exit_el1_irq_or_nmi(struct pt_regs *regs)
{
	if (IS_ENABLED(CONFIG_ARM64_PSEUDO_NMI) && !interrupts_enabled(regs))
		arm64_exit_nmi(regs);
	else
		exit_to_kernel_mode(regs);
}

static void noinstr el1_abort(struct pt_regs *regs, unsigned long esr)
{
	unsigned long far = read_sysreg(far_el1);

	enter_from_kernel_mode(regs);
	local_daif_inherit(regs);
	far = untagged_addr(far);
	do_mem_abort(far, esr, regs);
	local_daif_mask();
	exit_to_kernel_mode(regs);
}

static void noinstr el1_pc(struct pt_regs *regs, unsigned long esr)
{
	unsigned long far = read_sysreg(far_el1);

	enter_from_kernel_mode(regs);
	local_daif_inherit(regs);
	do_sp_pc_abort(far, esr, regs);
	local_daif_mask();
	exit_to_kernel_mode(regs);
}

static void noinstr el1_undef(struct pt_regs *regs, unsigned long esr)
{
	enter_from_kernel_mode(regs);
	local_daif_inherit(regs);
	do_el1_undef(regs, esr);
	local_daif_mask();
	exit_to_kernel_mode(regs);
}

static void noinstr el1_bti(struct pt_regs *regs, unsigned long esr)
{
	enter_from_kernel_mode(regs);
	local_daif_inherit(regs);
	do_el1_bti(regs, esr);
	local_daif_mask();
	exit_to_kernel_mode(regs);
}

static void noinstr el1_inv(struct pt_regs *regs, unsigned long esr)
{
	enter_from_kernel_mode(regs);
	local_daif_inherit(regs);
	bad_mode(regs, 0, esr);
	local_daif_mask();
	exit_to_kernel_mode(regs);
}

int in_dbg(void)
{
	return this_cpu_read(dbg_count) > 0;
}

static void noinstr arm64_enter_el1_dbg(struct pt_regs *regs)
{
	regs->lockdep_hardirqs = lockdep_hardirqs_enabled();

	this_cpu_inc(dbg_count);
	lockdep_hardirqs_off(CALLER_ADDR0);
	rcu_nmi_enter();

	trace_hardirqs_off_finish();
}

static void noinstr arm64_exit_el1_dbg(struct pt_regs *regs)
{
	bool restore = regs->lockdep_hardirqs;

	if (restore) {
		trace_hardirqs_on_prepare();
		lockdep_hardirqs_on_prepare(CALLER_ADDR0);
	}

	rcu_nmi_exit();
	if (restore)
		lockdep_hardirqs_on(CALLER_ADDR0);
	this_cpu_dec(dbg_count);
}

static void noinstr el1_dbg(struct pt_regs *regs, unsigned long esr)
{
	unsigned long far = read_sysreg(far_el1);

	arm64_enter_el1_dbg(regs);
	do_debug_exception(far, esr, regs);
	arm64_exit_el1_dbg(regs);
}

static void noinstr el1_fpac(struct pt_regs *regs, unsigned long esr)
{
	enter_from_kernel_mode(regs);
	local_daif_inherit(regs);
	do_el1_fpac(regs, esr);
	local_daif_mask();
	exit_to_kernel_mode(regs);
}

asmlinkage void noinstr el1_sync_handler(struct pt_regs *regs)
{
	unsigned long esr = read_sysreg(esr_el1);

	switch (ESR_ELx_EC(esr)) {
	case ESR_ELx_EC_DABT_CUR:
	case ESR_ELx_EC_IABT_CUR:
		el1_abort(regs, esr);
		break;
	/*
	 * We don't handle ESR_ELx_EC_SP_ALIGN, since we will have hit a
	 * recursive exception when trying to push the initial pt_regs.
	 */
	case ESR_ELx_EC_PC_ALIGN:
		el1_pc(regs, esr);
		break;
	case ESR_ELx_EC_SYS64:
	case ESR_ELx_EC_UNKNOWN:
		el1_undef(regs, esr);
		break;
	case ESR_ELx_EC_BTI:
		el1_bti(regs, esr);
		break;
	case ESR_ELx_EC_BREAKPT_CUR:
	case ESR_ELx_EC_SOFTSTP_CUR:
	case ESR_ELx_EC_WATCHPT_CUR:
	case ESR_ELx_EC_BRK64:
		el1_dbg(regs, esr);
		break;
	case ESR_ELx_EC_FPAC:
		el1_fpac(regs, esr);
		break;
	default:
		el1_inv(regs, esr);
	}
}

asmlinkage void noinstr enter_from_user_mode(void)
{
	lockdep_hardirqs_off(CALLER_ADDR0);
	CT_WARN_ON(ct_state() != CONTEXT_USER);
	user_exit_irqoff();
	trace_hardirqs_off_finish();
}

asmlinkage void noinstr exit_to_user_mode(void)
{
	trace_hardirqs_on_prepare();
	lockdep_hardirqs_on_prepare(CALLER_ADDR0);
	user_enter_irqoff();
	lockdep_hardirqs_on(CALLER_ADDR0);
}

static void noinstr el0_da(struct pt_regs *regs, unsigned long esr)
{
	unsigned long far = read_sysreg(far_el1);

	enter_from_user_mode();
	local_daif_restore(DAIF_PROCCTX);
	far = untagged_addr(far);
	do_mem_abort(far, esr, regs);
}

static void noinstr el0_ia(struct pt_regs *regs, unsigned long esr)
{
	unsigned long far = read_sysreg(far_el1);

	/*
	 * We've taken an instruction abort from userspace and not yet
	 * re-enabled IRQs. If the address is a kernel address, apply
	 * BP hardening prior to enabling IRQs and pre-emption.
	 */
	if (!is_ttbr0_addr(far))
		arm64_apply_bp_hardening();

	enter_from_user_mode();
	local_daif_restore(DAIF_PROCCTX);
	do_mem_abort(far, esr, regs);
}

static void noinstr el0_fpsimd_acc(struct pt_regs *regs, unsigned long esr)
{
	enter_from_user_mode();
	local_daif_restore(DAIF_PROCCTX);
	do_fpsimd_acc(esr, regs);
}

static void noinstr el0_sve_acc(struct pt_regs *regs, unsigned long esr)
{
	enter_from_user_mode();
	local_daif_restore(DAIF_PROCCTX);
	do_sve_acc(esr, regs);
}

static void noinstr el0_fpsimd_exc(struct pt_regs *regs, unsigned long esr)
{
	enter_from_user_mode();
	local_daif_restore(DAIF_PROCCTX);
	do_fpsimd_exc(esr, regs);
}

static void noinstr el0_sys(struct pt_regs *regs, unsigned long esr)
{
	enter_from_user_mode();
	local_daif_restore(DAIF_PROCCTX);
	do_el0_sys(esr, regs);
}

static void noinstr el0_pc(struct pt_regs *regs, unsigned long esr)
{
	unsigned long far = read_sysreg(far_el1);

	if (!is_ttbr0_addr(instruction_pointer(regs)))
		arm64_apply_bp_hardening();

	enter_from_user_mode();
	local_daif_restore(DAIF_PROCCTX);
	do_sp_pc_abort(far, esr, regs);
}

static void noinstr el0_sp(struct pt_regs *regs, unsigned long esr)
{
	enter_from_user_mode();
	local_daif_restore(DAIF_PROCCTX);
	do_sp_pc_abort(regs->sp, esr, regs);
}

static void noinstr el0_undef(struct pt_regs *regs, unsigned long esr)
{
	enter_from_user_mode();
	local_daif_restore(DAIF_PROCCTX);
	do_el0_undef(regs, esr);
}

static void noinstr el0_bti(struct pt_regs *regs)
{
	enter_from_user_mode();
	local_daif_restore(DAIF_PROCCTX);
	do_el0_bti(regs);
}

static void noinstr el0_inv(struct pt_regs *regs, unsigned long esr)
{
	enter_from_user_mode();
	local_daif_restore(DAIF_PROCCTX);
	bad_el0_sync(regs, 0, esr);
}

static void noinstr el0_dbg(struct pt_regs *regs, unsigned long esr)
{
	/* Only watchpoints write FAR_EL1, otherwise its UNKNOWN */
	unsigned long far = read_sysreg(far_el1);

	enter_from_user_mode();
	do_debug_exception(far, esr, regs);
	local_daif_restore(DAIF_PROCCTX_NOIRQ);
}

static void noinstr el0_svc(struct pt_regs *regs)
{
	enter_from_user_mode();
	do_el0_svc(regs);
}

static void noinstr el0_fpac(struct pt_regs *regs, unsigned long esr)
{
	enter_from_user_mode();
	local_daif_restore(DAIF_PROCCTX);
	do_el0_fpac(regs, esr);
}

asmlinkage void noinstr el0_sync_handler(struct pt_regs *regs)
{
	unsigned long esr = read_sysreg(esr_el1);

	switch (ESR_ELx_EC(esr)) {
	case ESR_ELx_EC_SVC64:
		el0_svc(regs);
		break;
	case ESR_ELx_EC_DABT_LOW:
		el0_da(regs, esr);
		break;
	case ESR_ELx_EC_IABT_LOW:
		el0_ia(regs, esr);
		break;
	case ESR_ELx_EC_FP_ASIMD:
		el0_fpsimd_acc(regs, esr);
		break;
	case ESR_ELx_EC_SVE:
		el0_sve_acc(regs, esr);
		break;
	case ESR_ELx_EC_FP_EXC64:
		el0_fpsimd_exc(regs, esr);
		break;
	case ESR_ELx_EC_SYS64:
	case ESR_ELx_EC_WFx:
		el0_sys(regs, esr);
		break;
	case ESR_ELx_EC_SP_ALIGN:
		el0_sp(regs, esr);
		break;
	case ESR_ELx_EC_PC_ALIGN:
		el0_pc(regs, esr);
		break;
	case ESR_ELx_EC_UNKNOWN:
		el0_undef(regs, esr);
		break;
	case ESR_ELx_EC_BTI:
		el0_bti(regs);
		break;
	case ESR_ELx_EC_BREAKPT_LOW:
	case ESR_ELx_EC_SOFTSTP_LOW:
	case ESR_ELx_EC_WATCHPT_LOW:
	case ESR_ELx_EC_BRK64:
		el0_dbg(regs, esr);
		break;
	case ESR_ELx_EC_FPAC:
		el0_fpac(regs, esr);
		break;
	default:
		el0_inv(regs, esr);
	}
}

#ifdef CONFIG_AARCH32_EL0
static void noinstr el0_cp15(struct pt_regs *regs, unsigned long esr)
{
	enter_from_user_mode();
	local_daif_restore(DAIF_PROCCTX);
	do_el0_cp15(esr, regs);
}

static void noinstr el0_svc_compat(struct pt_regs *regs)
{
	enter_from_user_mode();
	do_el0_svc_compat(regs);
}

asmlinkage void noinstr el0_sync_compat_handler(struct pt_regs *regs)
{
	unsigned long esr = read_sysreg(esr_el1);

	switch (ESR_ELx_EC(esr)) {
	case ESR_ELx_EC_SVC32:
		el0_svc_compat(regs);
		break;
	case ESR_ELx_EC_DABT_LOW:
		el0_da(regs, esr);
		break;
	case ESR_ELx_EC_IABT_LOW:
		el0_ia(regs, esr);
		break;
	case ESR_ELx_EC_FP_ASIMD:
		el0_fpsimd_acc(regs, esr);
		break;
	case ESR_ELx_EC_FP_EXC32:
		el0_fpsimd_exc(regs, esr);
		break;
	case ESR_ELx_EC_PC_ALIGN:
		el0_pc(regs, esr);
		break;
	case ESR_ELx_EC_UNKNOWN:
	case ESR_ELx_EC_CP14_MR:
	case ESR_ELx_EC_CP14_LS:
	case ESR_ELx_EC_CP14_64:
		el0_undef(regs, esr);
		break;
	case ESR_ELx_EC_CP15_32:
	case ESR_ELx_EC_CP15_64:
		el0_cp15(regs, esr);
		break;
	case ESR_ELx_EC_BREAKPT_LOW:
	case ESR_ELx_EC_SOFTSTP_LOW:
	case ESR_ELx_EC_WATCHPT_LOW:
	case ESR_ELx_EC_BKPT32:
		el0_dbg(regs, esr);
		break;
	default:
		el0_inv(regs, esr);
	}
}
#endif /* CONFIG_AARCH32_EL0 */
