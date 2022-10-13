# blender-mirror-fit
Blender plugin for finding a best-fit mirror plane for a given mesh.


## Concept

Given a mesh consisting of vertices $\vec{v_i}$ and a mirror transformation $M$ reflecting the vertices across a plane.

Define a function $\vec{v}'(\vec{x})$ that computes the closest point of the mesh to $\vec{x}$.

Using this function we can define the "squared error" of $M$ as:

$$E(M) = \sum_i ||\vec{v}'(M\vec{v_i}) - \vec{v_i}||^2$$

The goal of this plugin is to minimize this error total.
