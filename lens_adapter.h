#ifndef LENS_ADAPTER_H
#define LENS_ADAPTER_H

// define function prototypes 
int initLensAdapter(char * path);                                   // function to initialize the lens adapter
int autofocus(struct tm * tm_info);                                 // function to perform auto-focusing
void adjustCameraHardware();                                        // perform user specifications on focus position and aperture
int runCommand(const char * command, int file, char * return_str);  // function run the input two-letter command

// define global structure for camera parameters
#pragma pack(push, 1)
struct camera_params {
  // focus and aperture fields
  int prev_focus_pos;
  int focus_position;
  int focus_inf;
  int aperture_steps;
  int max_aperture;
  int min_focus_pos;
  int max_focus_pos;
  int current_aperture;
  // camera parameter, not lens parameter, but adjustments to it are made in lens_adapter.c
  double exposure_time;
  double change_exposure_bool;
  // auto-focusing parameters
  int focus_mode;             // flag to enter auto-focusing mode (1 = enter focus mode, 0 = leave focus mode)
  int start_focus_pos;        // where to start the auto-focusing process
  int end_focus_pos;          // where to end the auto-focusing process
  int focus_step;             // granularity of auto-focusing checker
};
#pragma pack(pop)

// make all blob_params acessible from any file that includes camera.h
extern struct camera_params all_camera_params;

#endif 