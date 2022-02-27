# Trabajo Práctico Final

Autores: Leonel Mendoza, Denise Gayet, Francisco Rossi.

Se puede encontrar la consigna y la resolución del trabajo práctico final de la materia 86.48 - Robótica Móvil. Todo el codigo fue desarrollado entorno al simulador dado por la cátedra, puede encontrarse dentro de la carpeta `simulator/src/`.

## Ejercicio 1 [`main.m`]

### Introducción
En el desafío de este ejercicio, se tiene un mapa conocido `map` y el robot debe, desde una pose inicial desconocida a priori `initPose` poder lograr el siguiente recorrido:

1. Ir hacia el punto A en las coordenadas (3, 1).
2. Esperar tres segundos.
3. Ir hacia el punto B y detenerse.

Estos son los tres pasos a llevar acabo en la misión, sin embargo, en la implementación tendremos pasos intermedios, una versión corta del algoritmo es el siguiente:

1. **Localización**: El robot debe localizarse en el entorno a partir de la posición inicial donde comienza la misión.
2. **Planeamiento de trayectoria 1**: Una vez `localizado` el robot debe encontrar un camino óptimo, sin obstáculos hacia el punto B.
3. **Movimiento hacia "A"** El robot se dirige hacia el punto A en las coordenadas (3, 1), a través del camino definido en el punto anterior.
4. **Espera** Esperar tres segundos en el punto A.
5. **Planeamiento de trayectoria 2**  Planea la trayectoria desde el punto A, hacia el punto B.
6. **Movimiento hacia "B"** Ir hacia el punto B
7. **Detenerse**

### Desarrollo

Este problema de localización al ser la pose inicial desconocida, se clasifica como `global localization` o de `localización global`.

Para resolver este primer paso del desafío, se utiliza un filtro de partículas regenerativas para disminuir la probabilidad de que se ubique mal dentro del mapa, con remuestreo selectivo para evitar el problema de agotamiento de partículas. Para la regeneracion de partículas se utilizo una mezcla de dos distribuciones: normal y uniforme.

- 75% de las particulas se regeneran muestreando una normal centrada en la pose estimada (utilizando todas las particulas) con una varianza con momento proporcional que se va actualizando a razon de 0.95.
- 25% de las particulas restantes se regeneran muestreando una distribución uniforme en todas las celdas libres dejando la posibilidad de que el robot se reubique en caso de estar mal ubicado (caso de falso positivo).

Para el planeamiento de caminos, se utilizó el algoritmo `A*`, el cual permite planear trayectorias evitando obstáculosm definiendo una serie de pasos que consiste en una lista de celdas del mapa de ocupación el cual debe recorrerse en orden. Esto fue acompañado de un mapa alternativo que fue convolucionado mediante funciones del paquete `robotics`. De esta forma se logra que el costo cerca de las paredes sea más alto y por lo tanto, menos probable de que se acerque a los límites físicos del entorno y que el robot choque, lo cual implicaría el fracaso de la misión.

Para el momento de realizar pruebas, se inicializa el robot en una pose aleatoria.
Para que la pose inicial sea físicamente posible, se obtuvieron las celdas libres del mapa, posteriormente se muestrearon variables aleatorias para inicializar la pose inicial del robot (que es interna, pues no sabemos a priori cual es la posición inicial del robot).

El movimiento inicial consta entonces de 3 etapas: localización, planeamiento de caminos y ejecución de caminos:

1. Primero el robot intentará ubicarse mediante el filtro de particulas con **n=180** partículas. Mientras se intenta localizar, ejecuta un algoritmo diseñado por nosotros, bautizado IRFE (Initial Random Free Exploration), moviéndose libremente por todo el mapa evadiendo obstáculos. El comportamiento esperado (y confirmado en el simulador) es que esquive los obstáculos a su derecha o izquierda, cuando detecta un obstáculo, el robot responde girando a la izquierda o derecha respectivamente. En caso de observar un obstáculo directamente de frente, se decide girar en algún sentido según una distribución tipo bernoulli equiprobable (`p=0.5`). A medida que se mueve en el mapa el filtro de particulas va estimando la posición del robot con mayor precisión, hasta que crea estar localizado (utilizando la varianza de las particulas).
2. Luego de localizarse, se detiene y planea un camino hacia algún punto, desde la pose estimada en la etapa anterior, utilizando A* y el mapa convolucionado, generado teniendo en cuenta el radio del robot.
3. Finalmente se ejecuta ese movimiento recorriendo todo el camino planeado, trantando de mantenerse cerca del camino con una cierta tolerancia. Mientras esté ejecutando el viaje hacia el punto, el filtro de particulas continúa estimando su pose e informándosela al algoritmo para evitar errores y choques, pero con menos partículas `p=45`, pues ya está `localizado` y no hace falta muestrear tantas partículas. El robot no detectará obstáculos una vez que este en viaje.

La localización o no del robot se define a partir de la varianza de la distribución de las partículas en el mapa.

## Ejercicio 2 [`main_desafio2.m`]

### Introducción

El desafío planteado en este ejercicio consiste en que el robot aparece en un lugar desconocido, y el robot debe conseguir de alguna manera un mapa del entorno. Dado quue no existe un mapa, ni tenes forma de localizar al robot a priori en ese mapa, este problema resoluble vía SLAM (Simultaneous Localization And Mapping).

### Desarrollo

El robot comienza el desafío, moviendose de manera aleatoria evitando obstaculos (mediante `IRFE`) y  utilizando `fastSLAM` (cuyo algoritmo fue implementado por el grupo) basado en `Rao-Blackwell` para generar un mapa del entorno, utilizando 15 partículas con sus respectivos mapas. Se implementó a su vez `scan-match` para reducir los errores de odometría, y en lugar de este método se base en realizar la maximizacion de verosimilitud en todo el dominio, se realizó en un entorno de la pose estimada muestreando 15 poses mediante una distribucion normal. El mapa final se guarda como `estimated_map.mat`.

## Posibles mejoras

- Integrar deteccion de obstáculos a la ejecucion del viaje luego de planear el camino.
- Eliminar la generación de uniformes cuando el robot este localizado.
- Si cree estar localizado y ejecuta viaje, luego detecta un obstáculo, probablemente estaba mal ubicado, realizar IRFE de nuevo.
- Mejoras de resampleo utilizadas en 1, implementar en 2.
- Distribuciones propuestas -> óptima
- Remuestreo con particulas efectivas
