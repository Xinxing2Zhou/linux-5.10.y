/* SPDX-License-Identifier: GPL-3.0-or-later */

#ifndef __LINUX_SCHED_SESSION_H
#define __LINUX_SCHED_SESSION_H

struct sched_session_entity {
	struct list_head	session_entry;
	unsigned int		session_prio;
};

int sched_setsession(pid_t pid, unsigned int prio);
#endif /* __LINUX_SCHED_SESSION_H */
