# object_detection
## Introduccion
### Objectivo
En este repositorio se analizará la posibilidad de ejecutar las herramientas de paralelización para el procesamiento de información de un robot de exploración espacial, utilizando las herramientas de ROS, OpenCV, CUDA y oneTBB, para realizar esto se desarrollara un algoritmo de procesamiento de imagen de profundidad con el fin de obtener una imagen en escala de grises de los objetos que el robot pueda tener en su rango de visión, para que en un futuro pueda realizar la detección de obstáculos.

## Instalacion 
### Requicitos 
#### Hardware
- Computadora con targeta grafica de Nvidia.
- Camara realsense D435 p
#### Software
- [Ubuntu 20.04 64bits]( https://ubuntu.com/download/desktop)
- [ROS Noetic](http://wiki.ros.org/noetic)
- El paquete  [ROS Wrapper for Intel® RealSense™ Devices](https://github.com/IntelRealSense/realsense-ros)
- El paquete [cv_bridge](http://wiki.ros.org/cv_bridge)
- [CUDA V10.1.243](https://developer.nvidia.com/cuda-toolkit-archive), [tutorial de instalacion](https://linuxconfig.org/how-to-install-cuda-on-ubuntu-20-04-focal-fossa-linux)
- [OpenCV](https://opencv.org/),[tutorial de instalacion](https://linuxize.com/post/how-to-install-opencv-on-ubuntu-20-04/#installing-opencv-from-the-source)
- GCC una version igual o menor 8
### Intalacion del proyecto

Colaner el repository en tu work space:
```
cd ~/catkin_ws/src
git clone https://github.com/AlexGarciaG/ar_tracker_tensorflow.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
[Tutotial de como crear tu work space](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
## Ejecucion
Lanzar la camara y RVIZ:
```
roslaunch object_detection camera.launch 
```
Lanzar deteccion de obstaculos usando C++
```
roslaunch object_detection object_detection.launch 
```
 Lanzar deteccion de obstaculos usando TTB
```
roslaunch object_detection object_detection_launch.launch 
```
## Problemas
- [x] Instalar Cuda en Ubuntu
- [x] Compatibilidad de OpenCV y ROS
- [x] Incompativilidad de librerias en RQT 
- [x] Compatibilidad de Cuda y ROS
- [x] Compatibilidad de Cuda y ROS
- [ ] Instalar una version compatible de cuda y opencv

## Estatus del proyecto
- [x] Desarrollar algoritmo en c++
- [x] Implementar modificacion dinamico de algortimos
- [x] Desarrollar algoritmo en TTB
- [] Desarrollar algoritmo en CUDA



