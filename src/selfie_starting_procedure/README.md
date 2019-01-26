# Selfie_starting_procedure

This node is responsible for detecting qr code in starting box. Detection begins after button is clicked. If there is no qr code detected, the car should then drive straight a given distance and after that send flag to other nodes to perform normal drive on track. 

## Requirements
This node uses [Zbar](http://zbar.sourceforge.net/) barcode processing library. To install it as global library on Ubuntu type

`sudo apt-get install libzbar-dev`

### Params

`preview` - if is set to 1, node enables to see the camera preview with marked detected qr code. Default value is 0

`tresh` - defines threshold of camera frames without qr code detected. If number of no qr frames exceeds tresh, this means the box gate is open and the car starts to drive. Default value is 5

`distance` - defines distance to drive, before sending sendig flag to other nodes. 


### Implementation 
The node contain two independent classes: 


##### - `Qr_decoder`
responsible for detecting qr_code


Subscibed topics:

`/image_row` - row camera frames

<br></br>

##### - `Starting_procedure` 
responsible for handling button click and drive event. 


Subscribed topics:

`/distance` - distance driven by a car 

`/odom` - odometry info

`/start_button` - info about pressed button

Published topics:

`/drive` - steering commands

`/start` - flag send to inform that the car is out the box
