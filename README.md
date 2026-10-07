# Cinemática Directa e Inversa - KUKA LBR iisy 11 R1300 (ROS 2)

Este repositorio contiene los paquetes de ROS 2 para la simulación y cálculo de Cinemática Directa (FK) y Cinemática Inversa (IK) del robot industrial **KUKA LBR iisy 11 R1300**.

---

## 📁 Estructura del Repositorio

```text
Grupo_8_Ramos_Miranda_kuka_lbr_iisy11_r1300_ws/
└── src/
    ├── grupo08_kuka_iisy11_bringup/    # Archivos de launch y configuración RViz
    │   └── launch/
    │       └── display.launch.py
    ├── grupo08_robot_kinematics/       # Nodos Python de cinemática
    │   └── grupo08_robot_kinematics/
    │       ├── __init__.py
    │       ├── fk_node.py              # Nodo de Cinemática Directa
    │       └── ik_node.py              # Nodo de Cinemática Inversa con animación suave
    └── kuka_robot_descriptions/        # Descripciones y mallas URDF del robot

# Clonar repositorio
git clone [https://github.com/francoramos312/Grupo_8_Ramos_Miranda_kuka_lbr_iisy11_r1300_ws.git](https://github.com/francoramos312/Grupo_8_Ramos_Miranda_kuka_lbr_iisy11_r1300_ws.git)
cd Grupo_8_Ramos_Miranda_kuka_lbr_iisy11_r1300_ws

# Compilar los paquetes
colcon build

# Cargar el entorno
source install/setup.bash


ros2 launch grupo08_kuka_iisy11_bringup display.launch.py


ros2 launch grupo08_kuka_iisy11_bringup display.launch.py


ros2 run grupo08_robot_kinematics ik_node.py
