all: test_camera

test_camera: commands.c camera.c camera.h lens_adapter.c lens_adapter.h astrometry.c astrometry.h solver.c 
	gcc -g commands.c camera.c lens_adapter.c astrometry.c solver.c -lsofa -lpthread -lastrometry -lueye_api -lm -I /usr/local/astrometry/include/ -o commands


.PHONY: clean

clean:
	rm -f *.o test_camera