^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dataset2bag
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* add support to load PNG or JPG images without decompressing them
* allow reading timestamps from video file; other fixes
* read image directory instead of opencv's pattern; camera calibration file now expects image size
* fix problem with newline at end of file; added improved checks
* agrego soporte para laser; arreglo problemas con la parte de imagenes
* agrego parametros extrinsecos a la calibracion. Agrego indicadores de progreso a los parsers.
* nothing
* soporte para IMU y ground-truth
* Merge branch 'KITTI'
* README.md; mejoras y fixes
* se corrigieron algunos comentarios
* agrego soporte para convertir dataset KITTI a ros
* se setio el headers de los mensajes
* se agrego que soporte stereo junto con mensajes canonicos de camera_info
* se pasa el skip por default para las imagenes
* ahora compila (antes necesitaba que exista el directorio include). tambien se agrego un breve comentario explicando como se debe pasar el parametro de imagen
* Contributors: Thomas Fischer, taihu, v01d
