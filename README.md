# Lab 3 - Inverse Kinematics of Phantom X with ROS

### Team Members:
- Christian Camilo Cuestas Ibáñez
- Fabian Steven Galindo Peña


## MATLAB + RTB Toolbox:

## Análisis:

Sabiendo que el robot Phantom X posee 4 GDL, de los cuales 3 corresponden a posición, el GDL restante proporciona una medida independiente para un ángulo de orientación (asuma orientación en ángulos fijos).

### ¿De qué ángulo de orientación se trata?

### ¿Cuántas soluciones posibles existen para la cinemática inversa del manipulador Phantom X ?

### ¿En qué consiste el espacio diestro de un manipulador?

## ROS - Aplicación de Pick and place:

## ROS - Aplicación de movimiento en el espacio de la tarea

Control de la posición del robot de manera escalada, desde el espacio de la tarea del efector final. Se realiza el script Punto3.py que permite realizar movimientos en los ejes x,y,z de TCP (ToolCenterPoint).  

### Avance:
En el codigo definimos ```trayectories(move kind, step, T0, n)```

#### Traslación: 
Para la translación se fija un avance de 1 mm tanto en el sentido positivo como en el sentido negativo.


#### Orientación: 
Para la orientación se define una rotación de 15° y -15° del TCP alrededor del eje Y o tambien del eje O.

### Tipos de movimiento:

Acontinuación estan enlistados los tipos de movimiento en su respectivo orden.
1. Traslación en X -> trax
2. Traslación en Y -> tray
3. Traslación en Z -> traz
4. Rotación sobre el eje O del TCP -> rot

### Control por teclado:

|Tecla  |Función                                |
| ----- | -----                                 |
|'W'    |Pasa al siguiente tipo de movimiento   |
|'S'    |Pasa al anterior tipo de movimiento    |
|'D'    |Movimiento con avance preestablecido en el sentido positivo de acuerdo al tipo de movimiento seleccionado    |
|'A'    |Movimiento con avance preestablecido en el sentido negativo de acuerdo al tipo de movimiento seleccionado    |

### Visualización en RViz:

## Video:
