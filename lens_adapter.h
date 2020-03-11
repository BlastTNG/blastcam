#ifndef LENS_ADAPTER_H
#define LENS_ADAPTER_H

// define function prototypes 
int init_lensAdapter(char * path);                                // function to initialize the lens adapter
void handleFocusAndAperture(int fileDescriptor);                  // perform user specifications on focus position and aperture
int runCommand(const char * command, int file, char* returnStr);  // function run the input two-letter command

// define global structure for camera parameters
#pragma pack(push, 1)
struct camera_params {
  int prev_focus_pos;
  int focus_position;
  int focus_inf;
  int aperture_steps;
  int max_aperture;
  int min_focus_pos;
  int max_focus_pos;
  int current_aperture;
};
#pragma pack(pop)

// make all blob_params acessible from any file that includes camera.h
extern struct camera_params all_camera_params;

#endif /* LENS_ADAPTER_H */