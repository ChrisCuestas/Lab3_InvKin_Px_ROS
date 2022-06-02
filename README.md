# Lab 3 - Inverse Kinematics of Phantom X with ROS

### Team Members:
- Christian Camilo Cuestas Ibáñez
- Fabian Steven Galindo Peña


## MATLAB + RTB Toolbox:

### Espacio de Trabajo

[![EspacioDeTrabajo](https://github.com/ChrisCuestas/Lab3_InvKin_Px_ROS/blob/main/matlab/EspacioTrabajo.png)]

## Análisis:

Sabiendo que el robot Phantom X posee 4 GDL, de los cuales 3 corresponden a posición, el GDL restante proporciona una medida independiente para un ángulo de orientación (asuma orientación en ángulos fijos).

### ¿De qué ángulo de orientación se trata?
El GDl restante es la orientacion de la herramienta, esta orientacion va a ser con respecto al eje Y (Open) de la herramienta, esta orientacion en angulos fijos es el pitch.
### ¿Cuántas soluciones posibles existen para la cinemática inversa del manipulador Phantom X ?
Se tendran dos soluciones posibles codo arriba y codo abajo.
### ¿En qué consiste el espacio diestro de un manipulador?
El espacio diestro del manipulador consiste los puntos en que el robot puede alcanzar todas las orientaciones posibles del TCP con al menos una solucion.
## ROS - Aplicación de Pick and place:

## ROS - Aplicación de movimiento en el espacio de la tarea

Control de la posición del robot de manera escalada, desde el espacio de la tarea del efector final. Se realiza el script Punto3.py que permite realizar movimientos en los ejes x,y,z de TCP (ToolCenterPoint).  

### Avance:
En el codigo definimos ```Camino=trayectories(move_kind, step, T0, n)```. Donde `move_kind` define si el movimiento es translacional en x, y, ó z, o rotacional. El `step` sera el valor de avance o retroceso que se fijó, tanto de translacion como de rotación. El `T0` es la pose actual del TCP. `n` es el numero de trayectorias intermedias entre la pose Actual y la pose Objetivo. Esta función retorna una matriz de poses `Camino` que conformaran la ruta que hara el TCP para realizar la translación o rotación.
Una vez definida la ruta `Camino` se utiliza la funcion `qs=q_invs(l, Camino[i])` donde `l` sera la longitud entre la posición de la muñeca del robot , y el TCP, `Camino[i]` sera la pose iesima de la matriz de poses de la ruta. Y la funcion dara 4 diferentes soluciones para alcanzar la pose de `Camino[i]` donde cada una contiene 4 valores de articulación.
Se realiza un ciclo para obtener los valores de articulacón de una solución en especifico de cada una de las poses de `Camino[i]`. se llama en cada ciclo la funcion `move(qs)` que recibe los valores de articulacion y generan el moviento del robot.
Por ultimo la pose objetivo pasa a ser la pose actual para repetir el proceso cuando sea necesario. 

```python
def trajectories(move_kind, step, T0, n):
    if move_kind == "trax":
        print("estoy moviendo en x")
        T1= T0*1
        T1[0,3]=T0[0,3] + step
        T1=SE3(T1)
        T0=SE3(T0)
        # + SE3(np.array([[0, 0, 0, step],[0, 0, 0, 0],[0, 0, 0, 0],[0, 0, 0, 1]]))
    elif move_kind == "tray":
        print("estoy moviendo en y")
        # T1 = T0 @ SE3(0,step,0)
        T1=T0*1
        T1[1,3]=T0[1,3] + step
        T1=SE3(T1)
        T0=SE3(T0)
    elif move_kind == "traz":
        print("estoy moviendo en z")
        T1=T0*1
        T1[2,3]=T0[2,3] + step
        T1=SE3(T1)
        T0=SE3(T0)
    else:
        T1 = SE3(T0) @ SE3(troty(step,"deg"))   # Rotation over the TCP's O axis
        T1=SE3(T1)
        T0=SE3(T0)
    print("prueba T0")
    print(T0)
    print("prueba T1")
    print(T1)
    return rtb.ctraj(T0,T1,n)
```  
```python
def q_invs(l, T):
    
    Pw = T[0:3, 3]-(float(l[3])*T[0:3, 2])
    q1a=np.arctan2(T[1,3],T[0,3])
    q1b=np.arctan2(-T[1,3],-T[0,3])
    pxy = np.sqrt((float(Pw[0]))**2 +(float(Pw[1]))**2)
    z = float(Pw[2]) - float(l[0])
    r = np.sqrt(pxy**2 + z**2)
    the3 = np.arccos((r**2 - (float(l[1]))**2 - (float(l[2]))**2)/(2*float(l[1])*float(l[2])))
    if np.isreal(the3):
        alp = np.arctan2(z,pxy)
        the2a = alp - np.arctan2(float(l[2])*np.sin(the3),float(l[1])+float(l[2])*np.cos(the3))
        the2b = alp + np.arctan2(float(l[2])*np.sin(the3),l[1]+float(l[2])*np.cos(the3))
        # Codo abajo
        q2a = -np.pi/2 + the2a      # q2
        q3a = the3;                 # q3

        #Codo arriba
        q2b = -np.pi/2 + the2b      # q2
        q3b = -the3;                # q3

        # Orientacion
        
        R_p = np.transpose(rotz(q1a)*rotx(pi/2)*rotz(q2a+q3a))@T[0:3,0:3]
        pitch = np.arctan2(float(R_p[0,2]),float(R_p[0,0]))
        # Codo abajo
        q4a = pitch - (q2a + q3a)   # q4
        # Codo arriba
        q4b = pitch - (q2b + q3b)   # q4
    else:
        q2a = np.NaN
        q2b = np.NaN
        q3a = np.NaN
        q3b = np.NaN
        q4a = np.NaN
        q4b = np.NaN

    q_inv=np.array([[q1a, q2a, q3a, q4a],
                    [q1a, q2b, q3b, q4b],
                    [q1b, -q2a, -q3a, -q4a],
                    [q1b, -q2b, -q3b, -q4b]],dtype=object)
    for i in range(3):
        for o in q_inv[i,:]:
            if np.isnan(o):
                q_inv[i,:] = np.array([np.NaN,np.NaN,np.NaN,np.NaN])
    return (q_inv)
```
#### Traslación: 
Para la translación se fija un avance de 1 cm tanto en el sentido positivo como en el sentido negativo. Si es una translación en x la pose T0 que entra en la función se modifica para obtener la pose T1 que es la pose objetivo. Se le suma el `step` a la componente en x del vector de desplazamiento de la pose actual TO y se obtiene de esa forma la pose objetivo T1. Si la translación es en el eje y o en el eje z, se realiza de esta misma manera cambiando la componente del vector de desplazamiento a la que se le suma el `step`.
Una vez obtenida la pose objetivo se utiliza `rtb.ctraj(T0,T1,n)` funcion que genera una matriz de n poses intermedias desde T0 hasta T1.

#### Orientación: 
Para la orientación se define una rotación de 15° y -15° del TCP alrededor del eje Y o tambien del eje O.
Para la orientación se multiplica la pose actiual T0 por la matriz de rotación en el eje y del el paso definido, y se obtine la matriz objetivo de T1.
Una vez obtenida la pose objetivo se utiliza `rtb.ctraj(T0,T1,n)` funcion que genera una matriz de n poses intermedias desde T0 hasta T1.
### Tipos de movimiento:

Acontinuación estan enlistados los tipos de movimiento en su respectivo orden.
1. Traslación en X -> trax
2. Traslación en Y -> tray
3. Traslación en Z -> traz
4. Rotación sobre el eje O del TCP -> rot

### Control por teclado:
Para controlar con el teclado se utilizo las mismas funciones que en en Lab1, como `letter=getkey()` que lee el valor de la tecla que se oprime y guarda el valor en `letter`. Se realiza unos condicionales para realizar la funcion dependiendo de la tecla seleccionada, 
Si la tecla es W se suma 1 a un `pointer` que va a indicar el tipo de movimiento que se quiere realizar, si este sobrepasa el valor de 4 retorna a 1 con la herramienta de modulo, para realizar una seleccion ciclica. Si la tecla es S se resta 1 al `pointer` para devolverse en la indicación del tipo de movimiento.
Si la tecla es D va a realizar el avance explicado anteriormente, y se le especifica el sentido positivo del `step`. Si la tecla es A va a realizar el avance pero especificando el sentido negativo del `step`.
|Tecla  |Función                                |
| ----- | -----                                 |
|'W'    |Pasa al siguiente tipo de movimiento   |
|'S'    |Pasa al anterior tipo de movimiento    |
|'D'    |Movimiento con avance preestablecido en el sentido positivo de acuerdo al tipo de movimiento seleccionado    |
|'A'    |Movimiento con avance preestablecido en el sentido negativo de acuerdo al tipo de movimiento seleccionado    |
```python
    while 1:
        letter = getkey()
        if(letter ==b'w' or letter == b'W'):
            # Change kind of movement forwards with W key
            pointer = (pointer + 1)%4
            print('Changed to {}'.format(movement_kind[pointer]))
        elif(letter ==b's' or letter == b'S'):
            # Change kind of movement backwards with S key
            pointer = (pointer - 1) if pointer>0 else 3
            print('Changed to {}'.format(movement_kind[pointer]))
        elif(letter ==b'd' or letter == b'D'):
            # Do the selected movement in the positive direction with D key
            print('{} +{}'.format(movement_kind[pointer],rotation_step if pointer==3 else translation_step))
            # function(movement_kind[pointer],rotation_step if pointer==3 else translation_step))
            Camino=trajectories(movement_kind[pointer],rotation_step if pointer==3 else translation_step,(Tactual),n)
            i=0
            while i<n:
                qs=q_invs(l, np.matrix(Camino[i]))[1]
                qss=np.array([rad2deg(qs[0]),rad2deg(qs[1]),rad2deg(qs[2]),rad2deg(qs[3])])
                move(qss)
                i=i+1
            
            Tactual=np.array(Camino[n-1])

        elif(letter ==b'a' or letter == b'A'):
            # Do the selected movement in the negative direction with A key
            print('{} -{}'.format(movement_kind[pointer],rotation_step if pointer==3 else translation_step))
            Camino=trajectories(movement_kind[pointer],-rotation_step if pointer==3 else -translation_step,(Tactual),n)
            i=0
            while i<n:
                qs=q_invs(l, np.matrix(Camino[i]))[1]
                qss=np.array([rad2deg(qs[0]),rad2deg(qs[1]),rad2deg(qs[2]),rad2deg(qs[3])])
                move(qss)
                i=i+1
            
            Tactual=np.array(Camino[n-1])
            # function(movement_kind[pointer],rotation_step if pointer==3 else (-translation_step)))
        if (letter==b'\x1b'):
            # Escape with crtl+z
            break
```
## Video:
Python y ROS - Aplicación de movimiento en el espacio de la tarea: https://youtu.be/pk0PcC_Z0Ls
En el video se puede observar como se realizan las translaciones en cada uno de los ejes, ademas de la rotacion para el cambio de la orientacion del TCP
Tambien se puede ver que se realizo la solución codo arriba.
## Conclusiones:
- Se necesito realizar un acercamiento al lenguaje de pyton para la manipulacion de matrices y poses, ya que es diferente al manejo que se les ha dado con matlab. Donde se tuvieron problemas con la creacion de la funcion `q_invs` y con la realizacion del pick an place.
- Se debio recalcular las distancias del eslabon dependiendo del robo que se utilizara, distancias que afectaban las linealidad de las trayectorias del TCP. 
- El robot utilizado tenia Id's distintos, asi que se reconfiguro el codigo para adaptar los ids al robot disponible. 

