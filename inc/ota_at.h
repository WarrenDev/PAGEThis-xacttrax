#ifndef __OTA_AT_H__
#define __OTA_AT_H__

void OTAAT_eval(char *cmd);
void test_otaat_tmr(u8 timerID, void *Context);
void OTAAT_handle_at_cmd_response(char *response_string, bool isTerminal, size_t response_string_length);
void send_ota_response(char *resp_string);
void QueSMS(char* TxBuff);
void SendOTAResponse(u8 timerID, void *Context);
void DConfig_Rpt_handler(u8 timerID, void *Context);
void ResetDiagTxTimerCount(UINT8 Index);

#endif
