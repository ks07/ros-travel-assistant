// Agent wheely in project wheely.mas2j

/* Initial beliefs and rules */
location(a).
last_cmd_time(0).

/* Initial goals */


/* Plans */

@printloc
+location(L)
	<- .print("Now at location: ", L).

// If we desire to be where we are, done!
//@l0
//+!location(Y)
//	: location(Y)
//	<- .print("Already at our destination.").

@la
+!location(Y)
	: not traffic(red) & last_cmd_time(LT) & LT > 0
	<- .print("interrupt command rcvd");
	wheely.unixtime(T);
	?last_cmd_time(LT);
	D = T - LT;
	if (D < (0.4 * 60000)) {
		// Less than 0.4 pcnt_prog so go back
		.print("New cmd but turning back");
		wheely.unixtime(MY_T);
		?last_cmd_time(REC_T);
		?REC_T <= MY_T;
		-+last_cmd_time(MY_T);			
		.wait(60000);
		?last_cmd_time(NEW_T);
		?NEW_T <= MY_T;
		-+last_cmd_time(0);
		!!location(Y);
	} else {
		// Too late to turn around, carry on
		.print("New cmd, pushing on");
		wheely.unixtime(MY_T);
		?last_cmd_time(REC_T);
		?REC_T <= MY_T;
		-+last_cmd_time(MY_T);			
		.wait(60000);
		?last_cmd_time(NEW_T);
		?NEW_T <= MY_T;
		-+last_cmd_time(0);
		?location(X);
		if (X == a) {
			Z = b;
		} else {
		    Z = a;
		}
		-+location(Z);
		!!location(Y);
	}.
	
	
// If we desire to be elsewhere, and the traffic is moving, need to wait.
@l1
+!location(Y)
//	: not location(Y) & not traffic(red)
	: not traffic(red)
	<- .print("Asking lights to stop traffic.");
	+wantToCross(Y);
	.send(lights,achieve,traffic(red)).
	
	
// If we desire to be elsewhere, and the traffic is stopped, move.
@l2
+!location(Y)
//	: not location(Y)
	<- ?traffic(red); 
	.print("Crossing...");
	wheely.unixtime(MY_T);
	?last_cmd_time(REC_T);
	?REC_T <= MY_T;
	-+last_cmd_time(MY_T);			
	.wait(60000);
	?last_cmd_time(NEW_T);
	?NEW_T <= MY_T;
	-+location(Y);
	-+last_cmd_time(0);
	.print("Now at location ", Y).

// When we come to learn that the traffic has been stopped, we can do goal
@red
+traffic(red)[source(lights)]
	: wantToCross(Y)
	<- .print("Traffic has stopped for us!");
	!location(Y);
	-wantToCross(Y).
