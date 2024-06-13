# TFG

Este proyecto tiene por finalidad el montaje del coche OSOYOO V2.1 y su posterior programación con un protocolo de IoT llamado Constrained Application Protocol (CoAP), el cual permite un control del vehículo a través de internet. Para ello, se ha creado un servidor CoAP en una placa Arduino (la cual está integrada en el vehículo) y se han creado unos clientes en Node-Red, que se encargan de enviar solicitudes al servidor. El servidor responde a estas solicitudes y lleva a cabo las correspondientes acciones solicitadas por ellas, lo que permite el movimiento del coche.

## Introducción por versiones

- En la primera versión se procede con el montaje del vehículo y la programación básica de los movimientos principales.
- En la segunda versión se introduce el control de los movimientos del coche mediante un mando remoto infrarrojo.
- En la versión número tres, el coche se conecta al wifi y se controla mediante una app desde el teléfono móvil.
- En la cuarta versión se introduce el protocolo CoAP y el coche será controlado desde un navegador (en Node-Red), sin necesidad de aplicación.
- En la quinta y última versión se mejorará a una interfaz más accesible (dashboard) para un control más sencillo e intuitivo.
