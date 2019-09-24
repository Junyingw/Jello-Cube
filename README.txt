==== BUILD ON MAC OS X ====

Unzip the files

> cd jellocube
> cd make
> ./jello world/<worldfile.w>


===== FINISHED PARTS ======
 
1) Mass spring system: structural, shear, bend springs and damping forces
2) External forces: force field (trilinear interpolation)
3) Bouncing off the walls: box collision detection

=== EXTRA CREDITS =========

1) Inclined plane detection and rendering.
2) Added texture (haha.ppm) to jello cube, using 't' to enable or disable texture.
3) Added mouse drag forces of a certain simulation point, if hits = 0, that means you didn't hit the simulation point (Unstable for now...If you want to test other functions, please avoid clicking left mouse button, otherwise it will have segmentation fault sometimes, don't know why...Please pay attention to the drag distance, if the force is too big, the program will crash!)

For mouse drag force part, I followed this reference----> 
http://www.cs.mun.ca/av/old/teaching/cg/notes/select.pdf

====== ENJOY =========

