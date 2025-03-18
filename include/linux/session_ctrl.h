/* SPDX-License-Identifier: GPL-3.0-or-later */

#ifndef __SESSION_CTRL_H
#define __SESSION_CTRL_H

#define SESSION_CTRL_MAGIC		'x'

enum {
	SESSION_CTRL_ADD = 1,
	/* ADD BELOW  */
	SESSION_CTRL_MAX_NR,
};

struct session_stat {
	int pid;
	int prio;
};

#define SESSION_CTRL_ADD	\
	_IOW(SESSION_CTRL_MAGIC, SESSION_CTRL_ADD, struct session_stat)

int session_ctrl_add(void __user *uarg);
#endif /* __SESSION_CTRL_H */
