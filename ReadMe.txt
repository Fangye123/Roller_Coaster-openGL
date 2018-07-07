This animation is a roller coaster animation which is continuous B-spline curve.
Press 'r' to switch camera. First camera is a briefly view of rails. Second camera is on the rails
The program need a file called "Control_point.txt" to read control points. The first line of file is number of control points.
I use several array to store curve points, first&second deviation, u,v,n,up vectors.
So it is easy to find every poins's data.
All knowledge is from slids.
When drawing a curve, just find every point and contact each other.
I also draw the three rails, connector and column in this. it's easy to implement them, just +/- some amount of u,n,v vectors
Finally I use lighting system, but it might be dark. You can press 's' to disable it.