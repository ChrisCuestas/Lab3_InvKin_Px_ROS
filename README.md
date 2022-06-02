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
En el codigo definimos ```trayectories(move_kind, step, T0, n)```. Donde `move_kind` define si el movimiento es translacional en x, y, ó z, o rotacional, el `step` sera el valor de avance o retroceso que se fijo, tanto de translacion como de rotación. un T0 que es la pose actual del TCP. N es el numero de trayectorias intermedias entre la pose Actual y la pose objetivo. Esta funcion retorna una matriz de poses que conformaran la ruta que hare el TCP para realizar la translacion o rotacion.
Una vez definida la ruta `Camino` se utiliza la funcion `q_invs(l, Camino[i])` donde l sera la longitud entre la posicion de la muñeca del robot , y el TCP, Camino[i] sera la pose iesima de la matriz de poses de la ruta. Y la funcion dara las 4 diferentes soluciones que cada una contien los 4 valores de articulacion para alcanzar la pose de `Camino[i]`.
Se realiza un ciclo para obtener las articulaciones de una solucion en especifica de cada una de las poses de Camino[i], se llama en cada ciclo la funcion move(qs) que recibe los valores de articulacion y genern el moviento del robot.
Por ultimo la pose objetivo pasa a ser la pose actual para repetir el proceso cuando sea necesario. 

#### Traslación: 
Para la translación se fija un avance de 1 cm tanto en el sentido positivo como en el sentido negativo. Si es un translación en x la pose T0 que entra en la funcion se modifica para obtener la pose T1 que es la pose objetivo. Se le suma el paso a la componente en x del vector de desplazamiento de la pose actual TO y se obtine de esa forma la pose objetivo T1. De esta misma manera se realiza si la translacion es en el eje y o en el eje z, cambiando la componente del vector de desplazamiento a la que se le suma el paso.
Una vez obtenida la pose objetivo se utiliza `rtb.ctraj(T0,T1,n)` funcion que genera una matriz de n poses intermedias desde T0 hasta T1.

#### Orientación: 
Para la orientación se define una rotación de 15° y -15° del TCP alrededor del eje Y o tambien del eje O.
Para la orientacion se multiplica la pose actiual T0 por la matriz de rotacion en el eje y del el paso definido, y se obtine la matriz objetivo de T1.
Una vez obtenida la pose objetivo se utiliza `rtb.ctraj(T0,T1,n)` funcion que genera una matriz de n poses intermedias desde T0 hasta T1.
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
