/*
 * VDDS_process.c
 *
 *  Created on: 2018年11月22日
 *      Author: Administrator
 */

#include "VDDS_process.h"

static double vdds_voltage = -1;


int set_mem_vdds(double vdds)
{
    vdds_voltage = vdds;

    return 0;
}


double get_mem_vdds()
{
    return vdds_voltage;
}

