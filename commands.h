#ifndef COMMANDS_H
#define COMMANDS_H

extern int num_clients;
extern int telemetry_sent;
extern int cancelling_auto_focus;
extern void * camera_raw;
void * processClient(void * arg);

#endif