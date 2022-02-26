# Trabajo Práctico Final

Autores: Leonel Mendoza, Denise Gayet, Francisco Rossi.

Se puede encontrar la consigna y la resolución del trabajo práctico final de la materia 86.48 - Robótica Móvil. Todo el codigo fue desarrollado entorno al simulador dentro de la carpeta simulator/src/

## Ejercicio 1 `main.m`

En este ejercicio el robot se localiza desde una pose inicial desconocida, luego planea un camino hacia un punto A, va hacia el punto, espera 3 segundos y luego planea un camino hacia B y va hacia ese punto, terminando su recorrido.

Se utilizo un filtro de partículas regenerativas para disminuir la probabilidad de que se ubique mal dentro del mapa, con remuestreo selectivo para evitar el problema de agotamiento de partículas. Para la regeneracion de partículas se utilizo una mezcla de dos distribuciones: normal y uniforme.

- 75% de las particulas se regeneran muestreando una normal centrada en la pose estimada (utilizando todas las particulas) con una varianza con momento proporcional que se va actualizando a razon de 0.95.
- 25% de las particulas restantes se regeneran muestreando una distribución uniforme en todas las celdas libres dejando la posibilidad de que el robot se reubique en caso de estar mal ubicado (caso de falso positivo).

Para el planeamiento de caminos, se utilizó A* para planear las trayectorias evitando obstáculos y movimientos mediante controladores proporcionales. Esto fue acompañado de un mapa alternativo que fue convolucionado mediante funciones del paquete `robotics`. De esta forma se logra que el costo cerca de las paredes sea más alto y por lo tanto, menos probable de que se acerque a los límites físicos del entorno.

Para inicializar el robot del simulador se obtuvieron las celdas libres del mapa, posteriormente se muestrearon variables aleatorias para inicializar la pose inicial del robot (que es interna, pues no sabemos a priori cual es la posición inicial del robot).

El ejercicio consta entonces de 3 etapas: localización, planeamiento de caminos y ejecución de caminos:

- Primero el robot intentará ubicarse mediante el filtro de particulas con 180 partículas. Mientras se intenta localizar, ejecuta un algoritmo diseñado por el grupo llamado IRFE (Initial Random Free Exploration), moviéndose libremente por todo el mapa evadiendo obstáculos. El comportamiento esperado (y confirmado en el simulador) es que esquive los obstáculos a su derecha o izquierda, girando a la izquierda o derecha respectivamente. En caso de observar un obstáculo directamente de frente, se decide girar en algún sentido según una bernoulli equiprobable. A medida que se mueve en el mapa el filtro de particulas va estimando la posición del robot con mayor precisión, hasta que crea estar localizado (utilizando la varianza de las particulas).
- Luego de localizarse, se detiene y planea un camino hacia algún punto, desde la pose estimada en la etapa anterior, utilizando A* y el mapa convolucionado teniendo en cuenta el radio del robot.
- Finalmente se ejecuta ese movimiento recorriendo todo el camino planeado, trantando de mantenerse cerca del camino con una cierta tolerancia. Mientras esté ejecutando el viaje hacia el punto, el filtro de particulas continúa estimando su pose e informándosela al algoritmo para evitar errores y choques, pero con menos partículas, pues ya está localizado y no hace falta muestrear tantas partículas. El robot no detectará obstáculos una vez que este en viaje.

## Ejercicio 2 `main_desafio2.m`

En este ejercicio el robot aparece en un lugar desconocido y se mueve de manera aleatoria evitando obstaculos (mediante IRFE), utilizando fastSLAM (implementado por el grupo) basado en `Rao-Blackwell` para generar un mapa del entorno, utilizando 15 partículas con sus respectivos mapas. Se implementó scan-match para reducir los errores de odometría, y en lugar de realizar la maximizacion de verosimilitud en todo el dominio, se realizó en un entorno de la pose estimada muestreando 15 poses mediante una distribucion normal. El mapa final se guarda como `estimated_map.mat`.

## Posibles mejoras

- Integrar deteccion de obstáculos a la ejecucion del viaje luego de planear el camino.
- Eliminar la generación de uniformes cuando el robot este localizado.
- Si cree estar localizado y ejecuta viaje, luego detecta un obstáculo, probablemente estaba mal ubicado, realizar IRFE de nuevo.
- Mejoras de resampleo utilizadas en 1, implementar en 2.
- Distribuciones propuestas -> óptima
- Remuestreo con particulas efectivas
