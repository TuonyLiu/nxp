/*********************************************************************
*                                                                    *
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
*                                                                    *
**********************************************************************
*                                                                    *
* C-file generated by:                                               *
*                                                                    *
*        GUI_Builder for emWin version 5.36                          *
*        Compiled Aug 31 2016, 10:53:09                              *
*        (c) 2016 Segger Microcontroller GmbH & Co. KG               *
*                                                                    *
**********************************************************************
*                                                                    *
*        Internet: www.segger.com  Support: support@segger.com       *
*                                                                    *
**********************************************************************
*/

// USER START (Optionally insert additional includes)
// USER END
#include "board.h"
#include "fsl_debug_console.h"

#include "jpeglib.h"
#include "ff.h"

#include "DIALOG.h"

/*********************************************************************
*
*       Defines
*
**********************************************************************
*/
#define ID_WINDOW_0   (GUI_ID_USER + 0x00)


// USER START (Optionally insert additional defines)
// USER END

/*********************************************************************
*
*       Static data
*
**********************************************************************
*/

// USER START (Optionally insert additional static data)
// USER END

/*********************************************************************
*
*       _aDialogCreate
*/
static const GUI_WIDGET_CREATE_INFO _aDialogCreate[] = {
  { WINDOW_CreateIndirect, "Window", ID_WINDOW_0, 120, 120, 220, 400, 0, 0x0, 0 },
  // USER START (Optionally insert additional widgets)
  // USER END
};

/*********************************************************************
*
*       Static code
*
**********************************************************************
*/

// USER START (Optionally insert additional static code)
// USER END

extern uint8_t __emWin_get_data_buf[];

extern uint8_t g_up_icon_index;
/*********************************************************************
*
*       _cbDialog
*/
static void _cbDialog(WM_MESSAGE * pMsg) {
  WM_HWIN hItem;
    FRESULT error;
    FSIZE_t image_size = 0;
    UINT bytesRead;
    FIL fileObject;
    
  switch (pMsg->MsgId) {
  case WM_INIT_DIALOG:
    //
    // Initialization of 'Window'
    //
    hItem = pMsg->hWin;
//    WM_SetHasTrans(hItem);
    // USER START (Optionally insert additional code for further widget initialization)
    // USER END
    break;
    case WM_PAINT:
        error = f_open(&fileObject, _T("/image.bmp"), (FA_READ) );
        f_lseek(&fileObject, 0);
        image_size = f_size(&fileObject);
        error = f_read(&fileObject, __emWin_get_data_buf, image_size, &bytesRead);
        f_close(&fileObject);
        GUI_BMP_Draw((void *)__emWin_get_data_buf,-120,-120);

        error = f_open(&fileObject, _T("/up.png"), (FA_READ) );
        image_size = f_size(&fileObject);
        error = f_read(&fileObject, __emWin_get_data_buf, image_size, &bytesRead);
        f_close(&fileObject);
        volatile int32_t png_xsize = 0;
        volatile int32_t png_ysize = 0;
        
        png_xsize = GUI_PNG_GetXSize((void *)__emWin_get_data_buf,image_size);
        png_ysize = GUI_PNG_GetYSize((void *)__emWin_get_data_buf,image_size);
        GUI_PNG_Draw((void *)__emWin_get_data_buf,image_size,0,0 + g_up_icon_index * 3);
    
        break;
  // USER START (Optionally insert additional message handling)
  // USER END
  default:
    WM_DefaultProc(pMsg);
    break;
  }
}

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
/*********************************************************************
*
*       CreateWindow
*/
WM_HWIN CreateWindow(void);
WM_HWIN CreateWindow(void) {
  WM_HWIN hWin;

  hWin = GUI_CreateDialogBox(_aDialogCreate, GUI_COUNTOF(_aDialogCreate), _cbDialog, WM_HBKWIN, 0, 0);

  return hWin;
}

// USER START (Optionally insert additional public code)
// USER END

/*************************** End of file ****************************/
