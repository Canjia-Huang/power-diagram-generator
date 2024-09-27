# Power-diagram-generator
![teaserfigure](pic/teaserfigure.png)
`PowerDiagramGenerator` is a lightweight and user-friendly program for computing or visualizing the power/voronoi diagram of the input (weighted) point cloud, to facilitate subsequent tasks such as rendering or reconstruction.

This program primarily relies on CPU-based computations. The data structures used in this program are partially derived from:

> Nicolas Ray, Dmitry Sokolov, Sylvain Lefebvre, and Bruno Lévy. 2018. Meshless voronoi on the GPU. ACM Trans. Graph. 37, 6, Article 265 (December 2018), 12 pages. https://doi.org/10.1145/3272127.3275092

Any bugs or feedback can be directed to the email canjia7@gmail.com
## Dependencies
All dependencies are listed below:
- [nanoflann] (already included)
---
## Building
In root directory:
```
mkdir build
cd build
cmake ..
make
```
---
## Usage
A command-line executable is provided, which reads in an `.xyz` file and generates either the power cell connection relationships or some visualizable `.obj` file.

For example (result in the teaserfigure, the default read and write paths are set to `/data/`):
```
./build/bin/release/PowerDiagramGenerator.exe -r=0.01 -visual=solid TEST.xyz
./build/bin/release/PowerDiagramGenerator.exe -bounding -visual=solid TEST.xyz
```
More usage instructions can be obtained by using the `-help` command.
### File format
Support point cloud input formats including `.obj`, `.off`, `.xyz`. It is worth noting that only the `.xyz` format supports input of weighted point cloud, with the format as follows:
```
x1 y1 z1 w1
x2 y2 z2 w2
...
```

The visualized output files are `.obj` format, these can be imported into rendering softwares which support n-gon polygon operations.

The output result information is a `.txt` file, which contains the information about the neighboring points for the power cell of each point. The output format is as follows:
```
0 point(0)'s neighbors(0) point(0)'s neighbors(1) point(0)'s neighbors(2) ...
1 point(1)'s neighbors(0) point(1)'s neighbors(1) point(1)'s neighbors(2) ...
...
```