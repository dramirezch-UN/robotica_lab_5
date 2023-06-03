# Laboratorio 5
## Código de la solución
El código de la solución se puede ver en los archivos del repositorio.
## Imágenes del HMI
Para interactuar con el robot se implementó una aplicación de terminal. Los comandos disponibles se pueden apreciar en el menú de la aplicación:
![cli](https://hackmd.io/_uploads/H1YC-kOU3.png)
Por ejemplo, si se quiere llevar el robot a la posición de home se utiliza el siguiente comando:
`python ~/catkin_ws/src/lab_5/scripts/lab_5.py home`
Todos los demás comandos se llaman de la misma manera.

## Portaherramientas
Se implementó el diseño de la herramienta utilizada en el laboratorio 1 y se resumirá su proceso de diseño.

El objetivo principal en el diseño de la herramienta fue asegurar que el eje z del plano de montaje y el eje z de la herramienta no estuvieran alienados. Para poder evitar problemas de singularidad en futuros laboratorios. También se tuvo en cuenta las limitaciones de la fabricación por impresión 3D. Por lo cual, se imprimió de manera modular para ensamblar las piezas como la herramienta final.

![inventor](https://i.imgur.com/RCDXKkv.png)


Se utilizó un tubo de PVC de 1/2 para recibir el marcador, aunque debido a un error por no tener en cuenta las tolerancias de fabricación se tuvo que cortar en la parte inferior para poderlo insertar. Se usó un resorte compensar la curvatura e imperfecciones del tablero.

A partir de este diseño se realizó una versión que simula el tubo de PVC junto con el marcador dentro de este para poder tener un modelo fidedigno con el cual trabajar en Robot Studio.

![inventor_herramienta_final](https://i.imgur.com/SFDi1mr.png)



Tras imprimir las piezas diseñadas y comprar los demás materiales, se ensambló la herramienta que sostiene al marcador. Se realizaron unos pequeños ajustes para facilitar su uso:

1. Al tener la herramienta ensamblada nos dimos cuenta de que era probable que el marcador se saliera de su sitio, ya que no estaba amarrado al tubo de PVC. Por esto, el primer ajuste que se hizo fue agregar cinta que conectara al marcador con el tubo de tal forma que no fuera posible que el marcador se saliera, pero que la cinta permitiera el movimiento del marcador hacia adentro del tubo.
2. Nos dimos cuenta de que los agujeros para los tornillos M6 eran muy pequeños. Por esto, tuvimos que ampliarlos. 

![herramienta](https://i.imgur.com/ISQIKxP.png)

## Videos
Debido al límite de tamaño de archivo de github, no fue posible agregar los videos al repositorio. Por esta razón se subieron a [youtube]().
## Descripción de la solución
Se escribió un único script, como se indicó en el enunciado.

Lo primero que se hace es definir una serie de constantes que serán usadas a lo largo del programa. Estos valores incluyen offsets de altura, límites del espacio de trabajo y las coordenadas que debe alcanzar la herramienta para dibujar las diferentes figuras.

Tras definir las constantes, se escribe el código relacionado a ROS. En este se inicializa el nodo junto al subscriber y se escriben algunas funciones de ayuda para comunicarse más facilmente con el robot.

Luego, se define la cinemática inversa de todas las articulaciones. También se definió la cinemática directa pero esta se usó principalmente para probar la inversa.

Tras esto, se escriben un par de funciones de ayuda para mejorar algunos aspectos del código y se programa la CLI. Esta se programó usando la librería `click`.

Para finalizar, en la función main se definen los torques de cada motor y se llama la función para que el programa escuche input del usuario a través del CLI.
