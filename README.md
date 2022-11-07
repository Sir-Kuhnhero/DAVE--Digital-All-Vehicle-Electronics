# DAVE--Digital-All-Vehicle-Electronics

DAVE is my attempt at creating an alternative to a regular RC remote. But in addition to that it should also be able to handle some sort of intelligent flight system.

If you want to know more about this project and how it was made check out my video:

This project ended in failure. For one, it's quite big and heavy, but the main issue is wireless communication. For some reason, the NRF24L01 modules I used were unusable and as soon as I'm more than a dozen meters away, the connection drops. I got my hands on an XBee SX and as far as I can tell it works very well. I plugged it in where I planed to connect the bluetooth module. Also, there are four stepper motor channels which are kind of useless and take up a lot of space.

At the moment I would strongly advise against using the files directly without modifications.
