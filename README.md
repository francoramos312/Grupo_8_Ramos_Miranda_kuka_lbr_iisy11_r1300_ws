# Cinemática del Robot KUKA LBR iisy 11 R1300 (ROS 2)

## 1. Información General
* **Proyecto:** Implementación, simulación y validación de los nodos de Cinemática Directa (FK) e Inversa (IK).
* **Modelo del Robot:** KUKA LBR iisy 11 R1300 (6 DOF).
* **Autores (Grupo 08):**
  * Franco Joaquin Ramos Mamani
  * Gustavo Miranda Espada

---

## 2. Software y Versiones Requeridas
* **Sistema Operativo:** Ubuntu 24.04 LTS (Noble Numbat)
* **Middleware ROS:** ROS 2 Jazzy Jalisco
* **Lenguaje:** Python 3.12+ (con librerías `numpy`, `rclpy` y `setuptools`)

---

## 3. Instalación y Configuración del Workspace
El repositorio incluye scripts automatizados para la gestión de dependencias de sistema, paquetes del robot y la compilación del espacio de trabajo.

### Clonación e Instalación:
```bash
# 1. Clonar el repositorio
git clone [https://github.com/francoramos312/Grupo_8_Ramos_Miranda_kuka_lbr_iisy11_r1300_ws.git](https://github.com/francoramos312/Grupo_8_Ramos_Miranda_kuka_lbr_iisy11_r1300_ws.git)
cd Grupo_8_Ramos_Miranda_kuka_lbr_iisy11_r1300_ws

# 2. Otorgar permisos e instalar dependencias
chmod +x *.sh
./instalar.sh

# 3. Carga del Entorno:
cd grupo_08_kuka_lbr_iisy11_r1300_ws/
source install/setup.bash
source entorno.sh
```
## Comandos de ejecucion y Pruebas
1. Lanzar simulacion (Terminal 1):
```bash
ros2 launch grupo08_kuka_iisy11_bringup display.launch.py
```
Resultado esperado: 
[Demostración RViz2](docs/captura_rviz.png)

2. Ejecutar el nodo de Cinemática Directa (Terminal 2):
```bash
#En la segunda terminal 
cd grupo_08_kuka_lbr_iisy11_r1300_ws/
source install/setup.bash
ros2 run grupo08_robot_kinematics fk_node

```
Resultado esperado:
[Cinematica_Directa](docs/captura_cinematicadirecta.png)

3. Ejecutar el nodo de Cinemática Directa (Terminal 3):
```bash
#Antes de ejecutar el sugundo nodo se debe cerrar el nodo de cinematica directa 
#En la tercera terminal 
cd grupo_08_kuka_lbr_iisy11_r1300_ws/
source install/setup.bash
ros2 run grupo08_robot_kinematics ik_node
```
Resultado esperado:
[Cinematica_Inversa](docs/captura_cinematicainversa.png)

4. Tópicos Utilizados:
* ./joint_states (sensor_msgs/msg/JointState): Leído por fk_node para obtener la posición de las articulaciones, y publicado por ik_node para animar el robot en RViz
* ./target (geometry_msgs/msg/Point): Tópico donde se publican las coordenadas cartesianas $(X, Y, Z)$ deseadas
* ./robot_description (std_msgs/msg/String): Publicado por robot_state_publisher para cargar el modelo URDF en RViz2.

5. Estructura del Repositorio

```text
Grupo_8_Ramos_Miranda_kuka_lbr_iisy11_r1300_ws/
├── .gitignore
├── README.md
├── abrir.sh
├── dependencias.repos
├── entorno.sh
├── instalar.sh
├── recompilar.sh
├── requirements.txt
├── verificar.sh
└── src/
    ├── grupo08_kuka_iisy11_bringup/      # Launch y configuración de RViz
    │   ├── launch/
    │   │   └── display.launch.py
    │   ├── CMakeLists.txt
    │   └── package.xml
    ├── grupo08_robot_kinematics/         # Nodos Python de cinemática
    │   ├── grupo08_robot_kinematics/
    │   │   ├── __init__.py
    │   │   ├── fk_node.py                # Ejecutable: ros2 run grupo08_robot_kinematics fk_node
    │   │   └── ik_node.py                # Ejecutable: ros2 run grupo08_robot_kinematics ik_node
    │   ├── package.xml
    │   ├── setup.cfg
    │   └── setup.py
    └── kuka_robot_descriptions/         
```
