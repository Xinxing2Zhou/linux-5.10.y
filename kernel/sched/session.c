// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) Huawei Technologies Co., Ltd. 2014-2021. All rights reserved.
 * Author: Huawei OS Kernel Lab
 *
 * This file is part of Session scheduler.
 *
 * This program is a free software, you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "sched.h"
#include "session.h"

#define MS_TO_NS 1000000
int session_task_running_limit = 50; /* session task limit running time(ms) */

void init_session_rq(struct session_rq *session)
{
	INIT_LIST_HEAD(&session->session_list);
	session->highest_session_prio = 0;
	session->session_nr_running = 0;
	session->session_running_begin = 0;
	session->session_last_preempt = 0;
	session->session_running_delta = 0;
}

void init_session_task(struct task_struct *p)
{
	INIT_LIST_HEAD(&p->session.session_entry);
	p->session.session_prio = 0;
}

void enqueue_session_thread(struct rq *rq, struct task_struct *p)
{
	struct list_head *pos = NULL;
	struct list_head *n = NULL;
	bool exist = false;

	list_for_each_safe(pos, n, &rq->session.session_list) {
		if (pos == &p->session.session_entry) {
			exist = true;
			break;
		}
	}

	if (!exist) {
		list_add_tail(&p->session.session_entry, &rq->session.session_list);
		rq->session.session_nr_running++;
		trace_sched_session_queue_op(p, "session_enqueue");
		get_task_struct(p);
	}

	if (p->session.session_prio > rq->session.highest_session_prio)
		rq->session.highest_session_prio = p->session.session_prio;
}

void dequeue_session_thread(struct rq *rq, struct task_struct *p)
{
	struct list_head *pos = NULL;
	struct list_head *n = NULL;
	unsigned int highest_prio;

	highest_prio = 0;
	list_for_each_safe(pos, n, &rq->session.session_list) {
		if (pos == &p->session.session_entry) {
			list_del_init(&p->session.session_entry);
			rq->session.session_nr_running--;
			trace_sched_session_queue_op(p, "session_dequeue");
			put_task_struct(p);
			break;
		}
	}

	if (p->session.session_prio >= rq->session.highest_session_prio)
		update_rq_highest_prio(rq);
}

int find_session_cpu(struct task_struct *p)
{
	int i;
	int target_cpu = -1;
	cpumask_t search_cpus = CPU_MASK_NONE;
	unsigned int min_session_prio = UINT_MAX;
	unsigned int min_nr_session_running = UINT_MAX;

	cpumask_and(&search_cpus, &p->cpus_mask, cpu_active_mask);

	/* find a cpu with lowest session prio */
	for_each_cpu(i, &search_cpus) {
		unsigned int cpu_session_prio = cpu_rq(i)->session.highest_session_prio;
		unsigned int cpu_nr_session_running = cpu_rq(i)->session.session_nr_running;

		if (check_session_content(i, p))
			continue;

		if (target_cpu != -1) {
			if (cpu_session_prio > min_session_prio)
				continue;
			if (cpu_session_prio == min_session_prio) {
				if (cpu_nr_session_running > min_nr_session_running)
					continue;
			}
		}

		target_cpu = i;
		min_session_prio = cpu_session_prio;
		min_nr_session_running = cpu_nr_session_running;
	}

	return target_cpu;
}

struct task_struct *pick_next_session_task(struct rq *rq)
{
	struct task_struct *task = NULL, *target_task = NULL, *n = NULL;
	struct session_rq *session_rq = &rq->session;
	u64 session_running_time;

	list_for_each_entry_safe(task, n, &session_rq->session_list, session.session_entry) {
		if (unlikely(task_rq(task) != rq)) {
			pr_warn("task %d on another rq\n", task->pid);
			list_del_init(&task->session.session_entry);
			continue;
		}

		if (unlikely(!task_on_rq_queued(task))) {
			pr_warn("task %d not queued\n", task->pid);
			list_del_init(&task->session.session_entry);
			continue;
		}

		if (!target_task ||
		    task->session.session_prio > target_task->session.session_prio)
			target_task = task;
	}

	if (target_task) {
		if (rq->session.session_running_begin == 0) {
			rq->session.session_running_begin = rq_clock(rq);
		} else {
			if (unlikely(rq->session.session_last_preempt)) {
				session_running_time = rq->session.session_last_preempt - rq->session.session_running_begin;
				rq->session.session_last_preempt = 0;
			} else
				session_running_time = rq_clock(rq) - rq->session.session_running_begin;

			rq->session.session_running_delta += session_running_time;
			rq->session.session_running_begin = rq_clock(rq);

			if (unlikely(rq->session.session_running_delta > (session_task_running_limit * MS_TO_NS))) {
				rq->session.session_running_delta = 0;
				target_task = NULL;
			}
		}
	}

	if (!target_task) {
		rq->session.session_running_begin = 0;
		rq->session.session_running_delta = 0;
	}

	return target_task;
}

#define MIN_SESSION_PRIO 1
#define MAX_SESSION_PRIO 10

int sched_setsession(pid_t pid, unsigned int prio)
{
	struct rq_flags rf;
	struct rq *rq;
	struct task_struct *p;
	bool queued, running;

	if (prio > MAX_SESSION_PRIO)
		return -EINVAL;

	rcu_read_lock();
	p = find_task_by_vpid(pid);

	if (unlikely(!p))
		return -ESRCH;

	if (p->sched_class != &fair_sched_class)
		return -EINVAL;

	if (p->session.session_prio == prio)
		return 0;

	get_task_struct(p);
	rcu_read_unlock();

	rq = task_rq_lock(p, &rf);
	queued = task_on_rq_queued(p);
	running = task_current(rq, p);

	if (queued) {
		update_rq_clock(rq);
		p->sched_class->dequeue_task(rq, p, DEQUEUE_SAVE | DEQUEUE_NOCLOCK);
	}

	if (running)
		put_prev_task(rq, p);

	p->session.session_prio = prio;

	if (queued)
		p->sched_class->enqueue_task(rq, p, ENQUEUE_RESTORE | ENQUEUE_NOCLOCK);
	if (running)
		set_next_task(rq, p);

	put_task_struct(p);
	task_rq_unlock(rq, p, &rf);
	return 0;
}
