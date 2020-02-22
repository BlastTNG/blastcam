/* Header file for commands.c */

// global structure for raw image data
// #pragma pack(push, 1)
// struct image_bytes {
//     image_data = calloc(CAMERA_WIDTH*CAMERA_HEIGHT, sizeof(double));
// };
// #pragma pack(pop)

//extern struct image_bytes star_camera_image;
extern void * camera_raw;

// void printCharArr(unsigned char * arr);

// #pragma pack(push)
// #pragma pack(1)
// struct udp_data {
//     unsigned int curr_time;
//   	float ra;
//   	float dec;
//   	float fr;
// 	float az;
// 	float el;
// 	float ir;
//     //int imageHeight;
//     //int imageWidth;
//     //unsigned char image[1936][1216];
// };
// #pragma pack(pop)

// extern struct udp_data all_data;