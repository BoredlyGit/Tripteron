# Tripteron

Code for my attempt at a [parallel axis tripteron](https://www.youtube.com/watch?v=6EtXycVGJg4).

## Works like this:
1. Convert an (x, y, z) position command for the head into positions for each slider (along its "x-axis").
2. Sends those commands to an arduino with the [gshield v5](https://synthetos.myshopify.com/products/gshield-v5) running [grbl](https://github.com/grbl/grbl). 

## The math:

Each slider (and its corresponding arms) provides a plane of freedom for the head. The intersection of these three planes of movement (with some offset) corresponds to the head of the tripteron.

In a "simulation" of the tripteron:

The intersection of the three planes is calculated. Each plane is then "translated" along the x-axis 1mm, and the effect of that movement on the head of the calculated (as in: moved --> recorded --> put back; repeat for each plane).
These effects can then be compiled into a matrix. 

Taking the inverse of that matrix yields a matrix that can transform changes in the coordinates of the head (x, y, z) into changes in the positions of each slider.

(I suspect that this explanation could use some improvement, so) **see main.py for exact details**.
