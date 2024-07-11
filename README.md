# PhysicSim
This project shows a fabric and solid simulation using Unity.

## Fabric
The fabric is a simple plane of triangles, with a node at each vertex and a spring at each edge. Using physic properties, the fabric can be animated threw time.\
It reacts to :
- Gravity
- Wind
- Solid objects and ground

## Solid
The solid is a high poly mesh of triangles, encapsulated in a lower resolution mesh of tetrahedrons. Tetrahedrons are better than triangles to make good solid simulations. They are generated from a mesh of triangles using the tool TetGen.\
The benefit of having two meshes (one that we can see and the other for the physic simulation) is that the simulation has better performance and the visual quality is not altered.\
It reacts to :
- Gravity
- Wind
- Solid objects and ground
