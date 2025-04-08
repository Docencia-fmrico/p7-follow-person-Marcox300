[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/3pU8xJ9A)
# follow_person

Implementa una aplicación ROS 2 que siga a una persona a un metro:
1. El robot debe acercarse y mantener la distancia de 1 metro a ella
2. Siempre ha de orientarse a la persona

1. Completa el subsistema de percepción visto en clase con un nuevo nodo que produzca TFs de un `vision_msgs/msg/Detection3DArray`. Puede usar el nodo de detección 3D que prefieras (basado en PointCloud2 o en imagen de profundidad).

![perception_3d drawio](https://github.com/Docencia-fmrico/2024-P6-FollowPerson/assets/3810011/6064ae73-ff19-4f78-baf8-701efb2783ff)

2. El nodo de control es un LifeCycle Node que empieza desactivado, y comienza su labor cuando está activo.
3. Haz que cuando no vea a una persona, la busque.
4. Una vez encontrada, usa un PID para aproximarse a la TF que haya producido la detección de la persona.

# Implementación de objetivos 

1. Código externo

Por parte del [ASR_2025](https://github.com/Docencia-fmrico/ASR_2025) utilizo los programas de *camera* lanzandolos desde mi launcher. Tambien se tiene que lanzar [Yolo](https://github.com/mgonzs13/yolo_ros) que se encuentra tambien en ARS,
esta parte no a sido comprobada actualmente.

2. Implementación propia

El nodo que transforma detecciones 3d a tf crea una tf de cámara a objetivo y la publica. En el segundo nodo LifeCycle cuando se activa recoge la tf obtenemos bf2target del árbol de transformadas de aqui sacamos la x e y dando la distancia al objetivo.

# Video



https://github.com/user-attachments/assets/7fcc417d-ccc8-4231-9cd4-5aaada3d5439

