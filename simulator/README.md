## Simulador MATLAB del curso Robotica Movil, FIUBA 2021-2022

El script `main.m` tiene el simulador de robot diferencial.
Es compatible con MATLAB R2020a y con MATLAB 2016b. No es compatible con versiones anteriores a R2016b y seguramente es compatible con versiones posteriores a R2016b, pero no fue verificado.

El script debe usarse como template para desarrollar el trabajo ya que luego cambiando el valor de use_roomba a true puede usarse con el robot real.

Tener en cuenta que nuevas versiones de este simulador pueden llegar a ser publicadas por lo que se recomienda el uso modular que permita actualizar el simulador facilmente.

### Code

#### Odometria (`DifferentialDrive.m`)
En este archivo, se definen 3 metodos para la clase cuya funcion es poder simular la odometria de un robot diferencial. 

1. `DifferentialDrive`: El constructor que define los parametros del robot (Radio de la ruedas y la distancia entre ruedas).
2. `forwardKinematics`: tiene como entrada velocidades de las ruedas (`wL`, `wR`), y como salida tiene la velocidad lineal y la velocidad angular (`v`, `w`).
3. `inverseKinematics`: tiene como entrada la velocidad lineal y la velocidad angular (`v`, `w`) y como salida la velocidad de las ruedas (`wL`, `wR`).


#### Transformacion homogenea de velocidad (`bodyToWorld.m`)

Esta funcion aplica una transformacion homogenea para velocidades (no hay translacion).

- Input: velocidades con respecto al robot y pose del robot. (`velB`, `pose`).
- Output: velocidades con respecto al mundo (`velW`).

#### Modelo de Lidar (`LidarSensor.m`)

- Valores fuera del rango maximo retornan `NaN`.

#### Visualizador (`Visualizer2D.m`)

#### Simulator main (`main.m`)
