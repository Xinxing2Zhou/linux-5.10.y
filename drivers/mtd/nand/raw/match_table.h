/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#ifndef __MATCH_TABLE_H__
#define __MATCH_TABLE_H__

struct match_reg_type {
    int reg;
    int type;
};

struct match_type_str {
    int type;
    const char *str;
};

struct match_t {
    int type;
    int reg;
    void *data;
};

#define MATCH_SET_TYPE_REG(_type, _reg)   {(_type), (_reg), (void *)0}
#define MATCH_SET_TYPE_DATA(_type, _data) {(_type), 0, (void *)(_data)}
#define MATCH_SET(_type, _reg, _data)     {(_type), (_reg), (void *)(_data)}

int reg2type(struct match_reg_type *table, int length, int reg, int def);

int type2reg(struct match_reg_type *table, int length, int type, int def);

int str2type(struct match_type_str *table, int length, const char *str,
             int size, int def);

const char *type2str(struct match_type_str *table, int length, int type,
                     const char *def);

int match_reg_to_type(struct match_t *table, int nr_table, int reg, int def);

int match_type_to_reg(struct match_t *table, int nr_table, int type, int def);

int match_data_to_type(struct match_t *table, int nr_table,const char *data,
                       int size, int def);

void *match_type_to_data(struct match_t *table, int nr_table, int type,
                        void *def);

#endif /* End of __MATCH_TABLE_H__ */
