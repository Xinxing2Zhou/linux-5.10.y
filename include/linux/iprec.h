#ifndef IPREC_H
#define IPREC_H

#include <linux/securec.h>

#define	IPREC        "iprec"
#define IPREC_DUMP   "dump"
#define IPREC_DLAY   "delay"
#define IPREC_LINE   "line"
#define IPREC_LOCK   "lock"
#define IPREC_DEFAULT_DLAY  800
#define IPREC_DEFAULT_LINE  2000
#define IPREC_STR_LEN       512

#define LOG_IPREC_ON 1
#define LOG_IPREC_OFF 0

#define LOG_IPREC_SWITCH LOG_IPREC_OFF

char *iprec_tm(void);
char *iprec_pool(void);
void iprec_inc(void);
void iprec_slock(void);
int iprec_glock(void);
spinlock_t *iprec_spinlock(void);


#if (LOG_IPREC_SWITCH == LOG_IPREC_ON)

#define iprec_fmt(fmt) fmt

#define iprec(fmt, ...) \
	do { \
		unsigned long flags; \
		spin_lock_irqsave(iprec_spinlock(), flags); \
		if (!iprec_glock()) { \
			sprintf_s(iprec_pool(), IPREC_STR_LEN, "%s " \
			iprec_fmt(fmt), iprec_tm(), ##__VA_ARGS__); \
			iprec_inc();} \
		spin_unlock_irqrestore(iprec_spinlock(), flags); \
	} while (0)

#else

#define iprec(fmt, ...)

#endif // LOG_IPREC_SWITCH


#endif // IPREC_H

