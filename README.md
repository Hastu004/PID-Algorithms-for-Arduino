<div align="center">

  ### Comparative study of classical PID control algorithms for the angular control of an electromechanical arm

</div>

<div align="center">

![](https://img.shields.io/badge/Contributions-Welcome-brightgreen.svg)
![](https://img.shields.io/badge/Maintained%3F-No-brightgreen.svg)

</div>

![](pid-2.gif)

<!-- TABLE OF CONTENTS -->

<details>
  <summary>Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <a href="#abstract">Abstract</a>
      </ul>
      <ul>
        <a href="#getting-started">Getting Started</a>
      </ul>
      <ul>
        <a href="#resources">Resources</a>
      </ul>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li><a href="#license">License</a></li>
  </ol>
</details>

## About The Project

This was my thesis project, I am a fan of robotics I share the arduino code, the paper and the video


## Abstract

According to various reports, it is estimated that around 70% of industrial control loops use the classical PID control algorithms (Proportional Integral Derivative), the most used of them is the Parallel PID, but there are variants such as serial PID, I-PD and PI-D, each of them has different properties. In this study, the performance of these diverse implementations is evaluated in an angular control plant that consists of an arm manipulated by a Brushless motor. For the angular control of the arm, the various implementations of PID controllers reported in the literature were tested, obtaining the value of their parameters according to the model of the plant. In order to design the algorithms it was necessary to discretize the differential equations that define the PID controllers, transforming their differential equations to equations of differences which were later programmed in C ++. In a first instance, the control schemes were implemented in simulation mode to subsequently implement them in real time. The results obtained were quite similar and the subsequent evaluation of the various implementations tested showed that the parallel PID algorithm fulfilled the control objectives more effectively.

Keywords: Automatic control; control algorithms; angular control; PID control


## Getting Started

- Download Arduino IDE
- Connect the USB to the Arduino
- Paste te code in the IDE and compile it
- Run it

### Built With
- Patience
- [Arduino](https://www.arduino.cc/)

## Resources

- [Paper](https://scielo.conicyt.cl/pdf/ingeniare/v28n4/0718-3305-ingeniare-28-04-612.pdf)
- [Full video](https://drive.google.com/file/d/16bK7dAosLCTXEhoV1iuj8vgqMc72WxBu/view?usp=sharing)

1. Fork the project

2. Clone the repository

```bash
git clone https://github.com/@username/PID-Algorithms-for-Arduino
```

3. Create your Feature Branch

```bash
git checkout -b feature/newFeature
```

4. Push to the Branch

```bash
git push origin feature/newFeature
```

5. Open a Pull Request

## License

Distributed under the MIT License. See `LICENSE.txt` for more information.

 




