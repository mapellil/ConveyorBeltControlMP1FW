# ConveyorBeltControlMP1FW
## Conveyor Belt Control FW for Avenger(MP1)+6180+IHMxx 

-- It requires STM32CubeIDE to be compiled
-- it requires the 96Boards "Avenger96", the 96 board "sensors-mezzanine", the "x-nucleo-ihm03a1", the "X-NUCLEO-6180A1" and a Stepper Motor 

*** Cmds to be sent from A7 Linux application on /dev/ttyRPMSG0, string terminator ';' ***


### *** Normal cycle operation ***
==================================

           <--- "ready;" 

"run_fwd;"  --->

           <--- "running_forward;" or "refused"
           <--- "stopped;"  (means piece positioned for snapshot)
"discard_piece;" --->  (if bad piece detected)

           <--- "discarded;"  (after 1 Sec) or "failed;" if piece was not removed
           
"run_fwd;"  ---> (start a new cycle)

"run_bwd;"  ---> (same as above but belt running in reverse direction)

### *** Manual cycle op ***
===========================

"run_fwd;" ---> 

            <--- "running_forward;"
            
"stop;"      --->

            <--- "stopped"

### *** Get status cmd ***
==========================

"get_status;"   --->

               <---   "running_forward;" or "stopped;"

### *** Error msgs ***
======================

           <--- "refused;"  answ for any cmd sent while in not coherent state
           <--- "unknown;"  answ for any unknown cmd
           
*** Spontaneous notifications sent from M4 FW on /dev/ttyRPMSG1, string terminator ';' ***           
