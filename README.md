# robtique-exp-32-camera

Ce projet a pour but d'utiliser un ESP32-CAM afin d'identifer des arucos. 
Une deuxième partie est ensuite de pouvoir calculer leur distance ainsi que leur orientation.

## Structure du projet

### Dossier main 

Contient  l'ensemble du code:

* `aruco_dict_6x6_250.h` : Import de la bibliothèque des aruco 6x6_250
* `auco.c / aruco.h` : Code permettant la reconnaissance des arucos et de déssiné sur l'image leur emplacement
* `camera_pinout.h` : Fichier de config relatif à l'esp32-cam
* `take_picture.c` : Code principal, monte la carte SD, prend la photo, identifie les arucos et l'enregistre sur la carte SD

### Dossier exemple

Contient des exemples de photos enregistrés avec le liseré bleu entouran les arucos enregistré, ainsi qu'un carré rouge permettant de connaître son orientation.
