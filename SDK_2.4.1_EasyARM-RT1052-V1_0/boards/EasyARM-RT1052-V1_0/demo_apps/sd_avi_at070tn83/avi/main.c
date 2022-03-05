/*******************************************************************************
*                                 Apollo
*                       ---------------------------
*                       innovating embedded platform
*
* Copyright (c) 2001-2012 Guangzhou ZHIYUAN Electronics Stock Co., Ltd.
* All rights reserved.
*
* Contact information:
* web site:    http://www.zlg.cn/
* e-mail:      apollo.support@zlg.cn
*******************************************************************************/

#include "aworks.h"
#include "aw_task.h"
#include "aw_serial.h"
#include "aw_clk.h"

extern void test_all (void);

int aw_main()
{
    extern int test_avi (void);
    test_avi();

    //test_all ();
    return 0;
}


/* end of file*/
