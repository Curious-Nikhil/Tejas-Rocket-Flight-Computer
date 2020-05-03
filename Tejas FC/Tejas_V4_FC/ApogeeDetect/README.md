# APOGEE DETECT

### To correct delayed detection of apogee, the APOGEE DETECT process should be used. This process analyses altitude readings in order to determine if a decceleration pattern is present. This is done by compiling current-state altimeter readings along with their timestamps to find the speed of increase between each pair of values. The values are then analysed to see if they follow a deccelerating pattern. 

### ex:(98.5 -> 105.2 -> 108.6) <-- Each pair of values is increasing although the rate of increase is slower.

### This is then used to make a predictive estimate on the number of seconds till apogee.


#### !!! This process does not account for outside forces, gravitational influence or mass of rocket !!!