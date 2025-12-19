- UART IS FULL DUPLEX DUMBASS, but stick to the prompting, sending info EXCEPT FOR ERRORS

- add actual sensors

- add code for the other side
    - impl logic -- DONE 
    - impl update in non blocking way with timer ints
    - impl error interrupt triggering telemetry call
    - impl resend older command if ack not recieved IFF HIGHER PRIORITY
        - use a fifo queue 
    
    - impl parsing
        - telemetry --DONE
        - ack -- Have to impl the queue first 
        - nack -- ^^