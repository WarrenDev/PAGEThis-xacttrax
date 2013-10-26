#ifndef __DOTAOPERATION_H__
#define __DOTAOPERATION_H__

#define SEND_OK_USB(x)    wm_sprintf(g_traceBuf, "OK %s %d\r\n", __FILE__, __LINE__); DumpMessage(g_traceBuf); adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_USB, "\n<usb>\r\nOK\r\n</usb>\n")

int recordFileSize(int FileSize);
void StartDOTA();
int ADWriteData(u16 DataSize, u8 *Data);
int CompleteADUpdate();
void recordFileSizeOTA(int FileSize);
#endif
