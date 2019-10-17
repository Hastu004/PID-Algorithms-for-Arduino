Estudio comparativo de algoritmos de control PID clásico para el control angular de un brazo electromecánico

Comparative study of classical PID control algorithms for the angular control of an electromechanical arm
 
Hernán Astudillo Roblero 1 
José Gallardo Arancibiam 2*
Claudio Ayala Bravom 3

1 Universidad Católica del Norte. Departamento de Ingeniería de Sistemas Y Computación. Antofagasta, Chile. har004@alumnos.ucn.cl: 
2 Universidad Católica del Norte. Departamento de Ingeniería de Sistemas Y Computación. Antofagasta, Chile. jgallardo@ucn.cl:
3 Universidad de Antofagasta. Departamento de Ingeniería Eléctrica. Antofagasta, Chile. claudio.ayala@uantof.cl:

* José Gallardo Arancibia2: jgallardo@ucn.cl

RESUMEN:
De acuerdo a diversos reportes, se estima que alrededor del 70% de los lazos de control industrial utilizan los clásicos algoritmos de control PID (Proporcional Integral Derivativo), el más utilizado de ellos es el PID Paralelo, pero existen variantes tales como los PID seriales, I-PD y PI-D, cada uno de ellos posee diferentes propiedades. En este estudio se evalúa el desempeño de estas diversas implementaciones en una planta de control angular que consta de un brazo manipulado por un motor Brushless. Para el control angular del brazo, se ensayaron las diversas implementaciones de controladores PID reportados en la literatura, obteniendo el valor de sus parámetros en función del modelo de la planta. Para lograr diseñar los algoritmos fue necesario discretizar las ecuaciones diferenciales que definen a los controladores PID, transformando sus ecuaciones diferenciales a ecuaciones de diferencias las que posteriormente se programaron en C++. En una primera instancia los esquemas de control se implementaron en modo simulación para posteriormente implementarlos en tiempo real. Los resultados logrados fueron bastante similares y la posterior evaluación de las diversas implementaciones ensayadas demostró que el algoritmo PID paralelo cumplió más eficazmente los objetivos de control. 

Palabras clave: Control automático, Algoritmos de control, Control angular, Control PID.

ABSTRACT:
According to various reports, it is estimated that around 70% of industrial control loops use the classical PID control algorithms (Proportional Integral Derivative), the most used of them is the Parallel PID, but there are variants such as serial PID, I-PD and PI-D, each of them has different properties. In this study, the performance of these diverse implementations is evaluated in an angular control plant that consists of an arm manipulated by a Brushless motor. For the angular control of the arm, the various implementations of PID controllers reported in the literature were tested, obtaining the value of their parameters according to the model of the plant. In order to design the algorithms it was necessary to discretize the differential equations that define the PID controllers, transforming their differential equations to equations of differences which were later programmed in C ++. In a first instance, the control schemes were implemented in simulation mode to subsequently implement them in real time. The results obtained were quite similar and the subsequent evaluation of the various implementations tested showed that the parallel PID algorithm fulfilled the control objectives more effectively.

Keywords: Automatic control, control algorithms, angular control, PID control.
