/* SPDX-License-Identifier: GPL-3.0-or-later */

#ifndef __SCHED_SESSION_H
#define __SCHED_SESSION_H

#include <linux/sched.h>

static inline void update_rq_highest_prio(struct rq *rq)
{
	unsigned int highest_prio = 0;
	struct task_struct *task = NULL;

	list_for_each_entry(task, &rq->session.session_list, session.session_entry) {
		highest_prio = max(highest_prio, task->session.session_prio);
	}

	rq->session.highest_session_prio = highest_prio;
}

static inline bool check_session_content(int cpu, struct task_struct *p)
{
	struct rq *rq = cpu_rq(cpu);

	/* No session thread in this rq. */
	if (list_empty(&rq->session.session_list))
		return false;

	return p->session.session_prio <= rq->session.highest_session_prio;
}

extern void init_session_task(struct task_struct *p);
extern void init_session_rq(struct session_rq *session);
extern void enqueue_session_thread(struct rq *rq, struct task_struct *p);
extern void dequeue_session_thread(struct rq *rq, struct task_struct *p);
extern int find_session_cpu(struct task_struct *p);
extern struct task_struct *pick_next_session_task(struct rq *rq);
#endif /* __SCHED_SESSION_H */
