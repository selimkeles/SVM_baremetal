#ifndef APP_INC_CLI_H_
#define APP_INC_CLI_H_

/* FreeRTOS task entry point. Spawn via osThreadNew(cli_task, NULL, ...). */
void cli_task(void *arg);

#endif /* APP_INC_CLI_H_ */
